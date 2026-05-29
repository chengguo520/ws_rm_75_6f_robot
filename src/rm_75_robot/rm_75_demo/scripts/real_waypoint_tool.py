#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import datetime
import math
import os
import sys
import xml.etree.ElementTree as ET

import moveit_commander
import rospy
import yaml
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState


# =============================================================================
# 1. 工程配置层
# =============================================================================
#
# 这个脚本是一个“点位保存与复现工具”：
# - save：把当前真实机械臂关节角保存成命名点位
# - list/show/delete：管理点位库
# - plan：只规划到某个点位，不执行
# - go：规划并慢速执行到某个点位，执行后检查真实反馈误差
#
# 安全策略：
# - 默认不执行，go 才会执行
# - go 默认需要键盘确认，加 --yes 才跳过确认
# - 限制速度/加速度缩放
# - 默认允许保存点位之间的大范围回放，可用 --max-delta-deg 额外收紧
# - 校验目标是否在 URDF 关节上下限内
# - 执行后比较 /joint_states 是否接近目标
GROUP_NAME = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_STORE = "/home/ros/ws_rm_75_6f_robot/study/rm75_6f_waypoints.yaml"
MAX_SCALING = 0.20
DEFAULT_VELOCITY = 0.05
DEFAULT_ACCELERATION = 0.05
DEFAULT_MAX_DELTA_DEG = 180.0
DEFAULT_VERIFY_TOLERANCE_DEG = 1.0


# =============================================================================
# 2. 通用格式化与 MoveIt 兼容层
# =============================================================================

def now_iso():
    return datetime.datetime.now().replace(microsecond=0).isoformat()


def deg(rad_value):
    return math.degrees(rad_value)


def rad(deg_value):
    return math.radians(deg_value)


def fmt_rad(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt_deg(values):
    return "[" + ", ".join("{:.1f}".format(deg(v)) for v in values) + "]"


def normalize_plan_result(plan_result):
    """兼容不同 MoveIt 版本的 group.plan() 返回格式。"""
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code

    return bool(plan_result.joint_trajectory.points), plan_result, None


def error_name(error_code):
    if error_code is None:
        return "unknown"

    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


# =============================================================================
# 3. 点位库读写层
# =============================================================================

def empty_store():
    return {
        "version": 1,
        "joint_names": list(JOINT_NAMES),
        "waypoints": {},
    }


def load_store(path):
    if not os.path.exists(path):
        return empty_store()

    with open(path, "r") as f:
        data = yaml.safe_load(f) or empty_store()

    data.setdefault("version", 1)
    data.setdefault("joint_names", list(JOINT_NAMES))
    data.setdefault("waypoints", {})
    return data


def save_store(path, data):
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)

    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)


def positions_to_dict(values):
    return {name: float(values[index]) for index, name in enumerate(JOINT_NAMES)}


def positions_from_waypoint(waypoint):
    positions = waypoint.get("positions", {})
    missing = [name for name in JOINT_NAMES if name not in positions]
    if missing:
        raise ValueError("Waypoint is missing joints: {}".format(", ".join(missing)))

    return [float(positions[name]) for name in JOINT_NAMES]


# =============================================================================
# 4. ROS / MoveIt 初始化与状态读取层
# =============================================================================

def init_moveit_node(args, node_name):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(node_name, anonymous=True)

    wait_for_joint_states()
    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(args.planning_time)
    group.set_num_planning_attempts(args.planning_attempts)
    set_scaling(group, args.velocity, args.acceleration)

    rospy.sleep(0.5)
    return group


def wait_for_joint_states():
    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
    except rospy.ROSException:
        raise RuntimeError("No /joint_states received. Is rm_driver connected to the robot?")

    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("/joint_states is missing joints: {}".format(", ".join(missing)))


def set_scaling(group, velocity, acceleration):
    if not 0.0 < velocity <= MAX_SCALING:
        raise ValueError("--velocity must be in (0, {:.2f}].".format(MAX_SCALING))

    if not 0.0 < acceleration <= MAX_SCALING:
        raise ValueError("--acceleration must be in (0, {:.2f}].".format(MAX_SCALING))

    group.set_max_velocity_scaling_factor(velocity)
    group.set_max_acceleration_scaling_factor(acceleration)


def current_joints(group):
    values = group.get_current_joint_values()
    if len(values) != len(JOINT_NAMES):
        raise RuntimeError("Expected {} joints, got {}.".format(len(JOINT_NAMES), len(values)))
    return list(values)


def load_joint_limits_from_robot_description():
    """从 /robot_description 读取 URDF 关节上下限。"""
    robot_description = rospy.get_param("/robot_description", "")
    if not robot_description:
        return {}

    limits = {}
    root = ET.fromstring(robot_description)
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if name in JOINT_NAMES and limit is not None:
            lower = limit.attrib.get("lower")
            upper = limit.attrib.get("upper")
            if lower is not None and upper is not None:
                limits[name] = (float(lower), float(upper))
    return limits


# =============================================================================
# 5. 安全校验层
# =============================================================================

def validate_target(current, target, args):
    """执行前校验：关节限位和可配置的单次变化量。"""
    errors = []
    limits = load_joint_limits_from_robot_description()
    max_delta = rad(args.max_delta_deg)

    for index, name in enumerate(JOINT_NAMES):
        value = target[index]
        delta = abs(target[index] - current[index])

        if delta > max_delta:
            errors.append(
                "{} delta {:.1f} deg exceeds limit {:.1f} deg".format(
                    name, deg(delta), args.max_delta_deg
                )
            )

        if name in limits:
            lower, upper = limits[name]
            if value < lower or value > upper:
                errors.append(
                    "{} target {:.1f} deg is outside URDF limit {:.1f}..{:.1f} deg".format(
                        name, deg(value), deg(lower), deg(upper)
                    )
                )

    if errors:
        raise ValueError("Target validation failed:\n- " + "\n- ".join(errors))


def print_delta_table(current, target):
    print("")
    print("Joint delta table:")
    print("{:<8} {:>12} {:>12} {:>12}".format("joint", "current", "target", "delta"))
    for index, name in enumerate(JOINT_NAMES):
        delta = target[index] - current[index]
        print(
            "{:<8} {:>9.2f}deg {:>9.2f}deg {:>+9.2f}deg".format(
                name, deg(current[index]), deg(target[index]), deg(delta)
            )
        )
    print("")


def verify_feedback(group, target, tolerance_deg):
    after = current_joints(group)
    tolerance = rad(tolerance_deg)
    errors = [abs(after[index] - target[index]) for index in range(len(JOINT_NAMES))]
    max_error = max(errors)

    print("After joints rad:", fmt_rad(after))
    print("After joints deg:", fmt_deg(after))
    print("Max feedback error: {:.2f} deg".format(deg(max_error)))

    if max_error > tolerance:
        print("WARNING: robot feedback did not reach target within {:.1f} deg.".format(tolerance_deg))
        print_delta_table(after, target)
        return False

    print("Feedback check OK.")
    return True


# =============================================================================
# 6. MoveIt 规划与执行层
# =============================================================================

def plan_to_target(group, target):
    group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
    success, plan, error_code = normalize_plan_result(group.plan())
    point_count = len(plan.joint_trajectory.points)

    print("Plan success:", success)
    print("MoveIt code:  ", error_name(error_code))
    print("Trajectory points:", point_count)

    if not success or point_count == 0:
        return None

    return plan


def confirm_execution(args, waypoint_name):
    if args.yes:
        return True

    print("")
    print("About to execute waypoint '{}' on the real robot.".format(waypoint_name))
    print("Keep the physical E-stop within reach.")
    answer = input("Type 'yes' to execute: ").strip().lower()
    return answer == "yes"


def execute_plan(group, plan):
    print("Executing planned trajectory...")
    executed = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)
    print("MoveIt execute result:", executed)
    return bool(executed)


# =============================================================================
# 7. 子命令实现层
# =============================================================================

def cmd_list(args):
    data = load_store(args.store)
    waypoints = data.get("waypoints", {})

    if not waypoints:
        print("No waypoints saved in {}".format(args.store))
        return 0

    print("Waypoint store:", args.store)
    print("{:<20} {:<20} {}".format("name", "created_at", "note"))
    for name in sorted(waypoints):
        item = waypoints[name]
        print("{:<20} {:<20} {}".format(name, item.get("created_at", ""), item.get("note", "")))
    return 0


def cmd_show(args):
    data = load_store(args.store)
    waypoint = data.get("waypoints", {}).get(args.name)
    if waypoint is None:
        print("Waypoint '{}' not found.".format(args.name))
        return 1

    values = positions_from_waypoint(waypoint)
    print("Waypoint:", args.name)
    print("Created:", waypoint.get("created_at", ""))
    print("Note:   ", waypoint.get("note", ""))
    print("Joints rad:", fmt_rad(values))
    print("Joints deg:", fmt_deg(values))
    return 0


def cmd_delete(args):
    data = load_store(args.store)
    if args.name not in data.get("waypoints", {}):
        print("Waypoint '{}' not found.".format(args.name))
        return 1

    del data["waypoints"][args.name]
    save_store(args.store, data)
    print("Deleted waypoint '{}'.".format(args.name))
    return 0


def cmd_current(args):
    group = init_moveit_node(args, "real_waypoint_current")
    values = current_joints(group)
    print("Current joints rad:", fmt_rad(values))
    print("Current joints deg:", fmt_deg(values))
    moveit_commander.roscpp_shutdown()
    return 0


def cmd_save(args):
    group = init_moveit_node(args, "real_waypoint_save")
    values = current_joints(group)

    data = load_store(args.store)
    if args.name in data["waypoints"] and not args.force:
        print("Waypoint '{}' already exists. Use --force to overwrite.".format(args.name))
        moveit_commander.roscpp_shutdown()
        return 1

    data["joint_names"] = list(JOINT_NAMES)
    data["waypoints"][args.name] = {
        "created_at": now_iso(),
        "positions": positions_to_dict(values),
        "note": args.note or "",
    }
    save_store(args.store, data)

    print("Saved waypoint '{}' to {}".format(args.name, args.store))
    print("Joints rad:", fmt_rad(values))
    print("Joints deg:", fmt_deg(values))
    moveit_commander.roscpp_shutdown()
    return 0


def load_waypoint_target(args):
    data = load_store(args.store)
    waypoint = data.get("waypoints", {}).get(args.name)
    if waypoint is None:
        raise ValueError("Waypoint '{}' not found in {}".format(args.name, args.store))
    return positions_from_waypoint(waypoint)


def cmd_plan(args):
    group = init_moveit_node(args, "real_waypoint_plan")
    current = current_joints(group)
    target = load_waypoint_target(args)
    validate_target(current, target, args)
    print_delta_table(current, target)

    plan = plan_to_target(group, target)
    moveit_commander.roscpp_shutdown()
    return 0 if plan is not None else 1


def cmd_go(args):
    group = init_moveit_node(args, "real_waypoint_go")
    current = current_joints(group)
    target = load_waypoint_target(args)
    validate_target(current, target, args)
    print_delta_table(current, target)

    plan = plan_to_target(group, target)
    if plan is None:
        moveit_commander.roscpp_shutdown()
        return 1

    if not confirm_execution(args, args.name):
        print("Execution cancelled.")
        moveit_commander.roscpp_shutdown()
        return 1

    executed = execute_plan(group, plan)
    ok = executed and verify_feedback(group, target, args.verify_tolerance_deg)
    moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


# =============================================================================
# 8. 命令行入口层
# =============================================================================

def add_common_runtime_args(parser):
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY)
    parser.add_argument("--acceleration", type=float, default=DEFAULT_ACCELERATION)
    parser.add_argument("--planning-time", type=float, default=8.0)
    parser.add_argument("--planning-attempts", type=int, default=10)


def add_safety_args(parser):
    parser.add_argument("--max-delta-deg", type=float, default=DEFAULT_MAX_DELTA_DEG)
    parser.add_argument("--verify-tolerance-deg", type=float, default=DEFAULT_VERIFY_TOLERANCE_DEG)


def add_store_arg(parser, default=argparse.SUPPRESS):
    parser.add_argument("--store", default=default, help="Waypoint YAML file.")


def build_parser():
    parser = argparse.ArgumentParser(
        description="RM75-6F real-robot waypoint save and replay tool."
    )
    add_store_arg(parser, default=DEFAULT_STORE)

    subparsers = parser.add_subparsers(dest="command")

    list_parser = subparsers.add_parser("list", help="List saved waypoints.")
    add_store_arg(list_parser)
    list_parser.set_defaults(func=cmd_list)

    show_parser = subparsers.add_parser("show", help="Show one waypoint.")
    show_parser.add_argument("name")
    add_store_arg(show_parser)
    show_parser.set_defaults(func=cmd_show)

    delete_parser = subparsers.add_parser("delete", help="Delete one waypoint.")
    delete_parser.add_argument("name")
    add_store_arg(delete_parser)
    delete_parser.set_defaults(func=cmd_delete)

    current_parser = subparsers.add_parser("current", help="Print current real robot joints.")
    add_store_arg(current_parser)
    add_common_runtime_args(current_parser)
    current_parser.set_defaults(func=cmd_current)

    save_parser = subparsers.add_parser("save", help="Save current real robot joints as a waypoint.")
    save_parser.add_argument("name")
    add_store_arg(save_parser)
    save_parser.add_argument("--note", default="")
    save_parser.add_argument("--force", action="store_true")
    add_common_runtime_args(save_parser)
    save_parser.set_defaults(func=cmd_save)

    plan_parser = subparsers.add_parser("plan", help="Plan to a saved waypoint without execution.")
    plan_parser.add_argument("name")
    add_store_arg(plan_parser)
    add_common_runtime_args(plan_parser)
    add_safety_args(plan_parser)
    plan_parser.set_defaults(func=cmd_plan)

    go_parser = subparsers.add_parser("go", help="Plan and execute to a saved waypoint.")
    go_parser.add_argument("name")
    add_store_arg(go_parser)
    go_parser.add_argument("--yes", action="store_true", help="Skip interactive confirmation.")
    add_common_runtime_args(go_parser)
    add_safety_args(go_parser)
    go_parser.set_defaults(func=cmd_go)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:])

    if not args.command:
        parser.print_help()
        return 2

    try:
        return args.func(args)
    except Exception as exc:
        print("ERROR:", exc)
        try:
            moveit_commander.roscpp_shutdown()
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
