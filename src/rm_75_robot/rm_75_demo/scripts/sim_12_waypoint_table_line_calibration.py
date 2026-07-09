#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F sim_12：从示教器保存点位生成擦桌子直线与桌面标定。

这个脚本是当前桌面擦拭标定主线：
1. 读取 study/rm75_6f_waypoints.yaml 中的两个点位；
2. 根据两点 link7/TCP 位姿计算擦拭线中心、长度和桌面高度；
3. 把同一张桌子加入 MoveIt PlanningScene，保证 RViz/MoveIt 碰撞物和
   Gazebo world 使用同一组几何参数；
4. 可选 MoveJ 到起点，再可选沿保存的 TCP 直线 MoveL 到终点。

默认只做“报告 + 加桌子”，不会执行机械臂运动。真正执行需要显式加
--go-start 或 --execute-line。
"""

from __future__ import print_function

import argparse
import inspect
import math
import os
import sys

import moveit_commander
import rosgraph
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import ObjectColor, PlanningScene
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


GROUP_NAME = "arm"
EEF_LINK = "link7"
AUTO_REFERENCE_FRAME = "auto"
FALLBACK_REFERENCE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_STORE = "/home/ros/ws_rm_75_6f_robot/study/rm75_6f_waypoints.yaml"
DEFAULT_START = "cazhuozi1"
DEFAULT_END = "cazhuozi2"
TABLE_ID = "rm75_wiping_table"


def fmt(values, digits=4):
    return "[" + ", ".join(("{:." + str(digits) + "f}").format(v) for v in values) + "]"


def deg(rad_value):
    return math.degrees(rad_value)


def distance(a, b):
    return math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))


def normalize_plan_result(plan_result):
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code
    return bool(plan_result.joint_trajectory.points), plan_result, None


def load_store(path):
    if not os.path.exists(path):
        raise RuntimeError("Waypoint store does not exist: {}".format(path))
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("waypoints", {})
    return data


def get_waypoint(data, name):
    waypoint = data.get("waypoints", {}).get(name)
    if waypoint is None:
        raise RuntimeError("Waypoint '{}' not found in store.".format(name))
    return waypoint


def waypoint_joints(waypoint):
    positions = waypoint.get("positions", {})
    missing = [name for name in JOINT_NAMES if name not in positions]
    if missing:
        raise RuntimeError("Waypoint missing joints: {}".format(", ".join(missing)))
    return [float(positions[name]) for name in JOINT_NAMES]


def waypoint_xyz_rpy(waypoint):
    pose_data = waypoint.get("tcp_pose") or {}
    xyz_rpy = pose_data.get("xyz_rpy")
    if not isinstance(xyz_rpy, list) or len(xyz_rpy) != 6:
        raise RuntimeError("Waypoint has no tcp_pose.xyz_rpy. Save it again with the updated teach pendant.")
    return [float(value) for value in xyz_rpy]


def waypoint_frame(waypoint):
    return (waypoint.get("tcp_pose") or {}).get("frame_id") or ""


def resolve_reference_frame(args, start_wp, end_wp):
    if args.reference_frame != AUTO_REFERENCE_FRAME:
        return args.reference_frame

    start_frame = waypoint_frame(start_wp)
    end_frame = waypoint_frame(end_wp)
    if start_frame and end_frame and start_frame != end_frame:
        raise RuntimeError(
            "Start/end waypoints use different frames: {} vs {}. Pass --reference-frame explicitly.".format(
                start_frame,
                end_frame,
            )
        )
    return start_frame or end_frame or FALLBACK_REFERENCE_FRAME


def sim09_base_link_surface_z(reference_frame, surface_z):
    # 当前 Gazebo 模型里 dummy -> base_link 是固定 +0.03 m。
    # sim_09 用 base_link->link7 的 TF 做导纳，因此给 sim_09 的 surface_z 要换到 base_link。
    if reference_frame == "dummy":
        return surface_z - 0.030
    return surface_z


def pose_stamped_from_xyz_rpy(frame_id, xyz_rpy):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = xyz_rpy[0]
    pose.pose.position.y = xyz_rpy[1]
    pose.pose.position.z = xyz_rpy[2]
    quat = quaternion_from_euler(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5])
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


def derive_geometry(start_pose, end_pose, args):
    p1 = start_pose[:3]
    p2 = end_pose[:3]
    line_center = [(p1[i] + p2[i]) * 0.5 for i in range(3)]
    line_delta = [p2[i] - p1[i] for i in range(3)]
    line_length = distance(p1, p2)
    surface_z = line_center[2] - args.tool_offset_z + args.surface_z_adjust
    moveit_surface_z = surface_z - abs(args.moveit_table_clearance_z)

    table_size_x = args.table_size_x
    if table_size_x <= 0.0:
        table_size_x = max(0.30, line_length + 2.0 * args.table_margin_x)

    table_size = [table_size_x, args.table_size_y, args.table_size_z]
    table_center = [
        line_center[0] + args.table_center_x_offset,
        line_center[1] + args.table_center_y_offset,
        moveit_surface_z - table_size[2] * 0.5,
    ]
    return {
        "p1": p1,
        "p2": p2,
        "line_center": line_center,
        "line_delta": line_delta,
        "line_length": line_length,
        "surface_z": surface_z,
        "moveit_surface_z": moveit_surface_z,
        "table_center": table_center,
        "table_size": table_size,
    }


def print_report(args, start_wp, end_wp, start_joints, end_joints, start_pose, end_pose, geometry):
    print("RM75-6F sim_12 waypoint table-line calibration")
    print("Waypoint store:       ", args.store)
    print("Start/end:            ", args.start, "->", args.end)
    print("Start joints rad:     ", fmt(start_joints))
    print("Start joints deg:     ", fmt([deg(v) for v in start_joints], digits=2))
    print("End joints rad:       ", fmt(end_joints))
    print("End joints deg:       ", fmt([deg(v) for v in end_joints], digits=2))
    print("Start link7 xyz:      ", fmt(start_pose[:3]))
    print("End link7 xyz:        ", fmt(end_pose[:3]))
    print("Line center xyz:      ", fmt(geometry["line_center"]))
    print("Line delta xyz:       ", fmt(geometry["line_delta"]))
    print("Line length [m]:      ", "{:.4f}".format(geometry["line_length"]))
    print("Tool offset z [m]:    ", "{:.4f}".format(args.tool_offset_z))
    print("Virtual/Gazebo surface z:", "{:.4f}".format(geometry["surface_z"]))
    print("MoveIt collision top z:  ", "{:.4f}".format(geometry["moveit_surface_z"]))
    print("MoveIt table clearance:  ", "{:.4f}".format(args.moveit_table_clearance_z))
    print("Reference frame used: ", args.reference_frame)
    print("Table center xyz:     ", fmt(geometry["table_center"]))
    print("Table size xyz:       ", fmt(geometry["table_size"]))
    print("Start stored frame:   ", (start_wp.get("tcp_pose") or {}).get("frame_id", ""))
    print("End stored frame:     ", (end_wp.get("tcp_pose") or {}).get("frame_id", ""))
    print("Suggested sim_09 args:", "--surface-z {:.3f} --line-center-x {:.3f} --line-center-y {:.3f} --line-length {:.3f}".format(
        sim09_base_link_surface_z(args.reference_frame, geometry["surface_z"]),
        geometry["line_center"][0],
        geometry["line_center"][1],
        max(0.03, geometry["line_length"]),
    ))


def wait_for_known(scene, object_id, timeout):
    start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
        if object_id in scene.get_known_object_names():
            return True
        rospy.sleep(0.1)
    return False


def make_pose_stamped(frame_id, xyz):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.w = 1.0
    return pose


def set_scene_color(scene_pub, object_id, rgba):
    color = ObjectColor()
    color.id = object_id
    color.color.r = rgba[0]
    color.color.g = rgba[1]
    color.color.b = rgba[2]
    color.color.a = rgba[3]

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.object_colors.append(color)
    scene_pub.publish(planning_scene)


def add_table(scene, scene_pub, args, geometry):
    scene.remove_world_object(args.table_id)
    rospy.sleep(0.2)
    scene.add_box(
        args.table_id,
        make_pose_stamped(args.reference_frame, geometry["table_center"]),
        size=geometry["table_size"],
    )
    known = wait_for_known(scene, args.table_id, args.scene_timeout)
    set_scene_color(scene_pub, args.table_id, (0.22, 0.28, 0.25, 1.0))
    print(
        "PlanningScene table:  ",
        args.table_id,
        "known=",
        known,
        "collision_top_z={:.4f}".format(geometry["moveit_surface_z"]),
    )
    return known


def remove_table_for_contact_motion(scene, args):
    """擦拭点位靠近桌面，硬碰撞桌会把合法接触姿态判成碰撞。

    Gazebo 里的可视桌仍然存在；这里只临时移除 MoveIt PlanningScene 中的
    collision object，让 MoveJ/MoveL 能验证保存点位和控制闭环。
    """
    scene.remove_world_object(args.table_id)
    rospy.sleep(0.5)
    print("PlanningScene table temporarily removed for contact-line motion.")


def plan_movej(group, joints):
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(JOINT_NAMES, joints)))
    success, plan, error_code = normalize_plan_result(group.plan())
    points = len(plan.joint_trajectory.points)
    print("MoveJ plan success={} points={} error_code={}".format(success, points, getattr(error_code, "val", error_code)))
    return success and points > 0, plan


def plan_movel(group, args, end_pose_stamped):
    group.set_start_state_to_current_state()
    waypoints = [end_pose_stamped.pose]
    signature = inspect.getfullargspec(group.compute_cartesian_path)
    if "jump_threshold" in signature.args:
        # 不同 MoveIt Python 绑定的 compute_cartesian_path 参数顺序不完全一致。
        # 旧接口支持 jump_threshold；本机 Noetic 接口不支持，所以运行时判断。
        plan, fraction = group.compute_cartesian_path(
            waypoints,
            args.eef_step,
            args.jump_threshold,
            args.avoid_collisions,
        )
    else:
        plan, fraction = group.compute_cartesian_path(
            waypoints,
            args.eef_step,
            args.avoid_collisions,
        )
    points = len(plan.joint_trajectory.points)
    print("MoveL plan fraction={:.3f} points={} avoid_collisions={}".format(fraction, points, args.avoid_collisions))
    return fraction, points, plan


def parse_args(argv):
    parser = argparse.ArgumentParser(description="RM75-6F sim_12: derive table line from saved waypoints.")
    parser.add_argument("--store", default=DEFAULT_STORE)
    parser.add_argument("--start", default=DEFAULT_START)
    parser.add_argument("--end", default=DEFAULT_END)
    parser.add_argument(
        "--command",
        choices=["setup", "start", "plan-line", "execute-line", "start-plan-line", "start-execute-line"],
        default="setup",
        help="High-level action: setup adds the table only; start moves to start; line commands plan/execute MoveL.",
    )
    parser.add_argument("--report-only", action="store_true", help="Only parse waypoint YAML and print derived geometry; no ROS needed.")

    parser.add_argument("--group", default=GROUP_NAME)
    parser.add_argument("--eef-link", default=EEF_LINK)
    parser.add_argument(
        "--reference-frame",
        default=AUTO_REFERENCE_FRAME,
        help="Pose/table frame. auto uses the frame saved in the waypoint YAML, usually dummy in Gazebo.",
    )
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--table-id", default=TABLE_ID)
    parser.add_argument("--scene-timeout", type=float, default=3.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--planning-time", type=float, default=12.0)
    parser.add_argument("--planning-attempts", type=int, default=20)
    parser.add_argument("--velocity", type=float, default=0.12)
    parser.add_argument("--acceleration", type=float, default=0.10)

    parser.add_argument("--tool-offset-z", type=float, default=0.040)
    parser.add_argument("--surface-z-adjust", type=float, default=0.0)
    parser.add_argument(
        "--moveit-table-clearance-z",
        type=float,
        default=0.080,
        help="Lower the MoveIt collision tabletop below the virtual/Gazebo surface, m.",
    )
    parser.add_argument("--table-center-x-offset", type=float, default=0.15)
    parser.add_argument("--table-center-y-offset", type=float, default=0.0)
    parser.add_argument("--table-size-x", type=float, default=0.82, help="Use <=0 to derive from line length.")
    parser.add_argument("--table-size-y", type=float, default=0.44)
    parser.add_argument("--table-size-z", type=float, default=0.05)
    parser.add_argument("--table-margin-x", type=float, default=0.12)

    parser.add_argument("--plan-start", action="store_true", help="Plan MoveJ from current state to start waypoint.")
    parser.add_argument("--go-start", action="store_true", help="Execute MoveJ to start waypoint after planning.")
    parser.add_argument("--plan-line", action="store_true", help="Plan Cartesian MoveL from current pose to end waypoint pose.")
    parser.add_argument("--execute-line", action="store_true", help="Execute Cartesian MoveL. Requires line fraction to pass.")
    parser.add_argument("--min-fraction", type=float, default=0.95)
    parser.add_argument("--eef-step", type=float, default=0.005)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--avoid-collisions", action="store_true", default=True)
    parser.add_argument("--no-avoid-collisions", dest="avoid_collisions", action="store_false")
    parser.add_argument(
        "--remove-table-collision-during-motion",
        action="store_true",
        help="Diagnosis only: remove the MoveIt table collision object before moving.",
    )
    parser.add_argument(
        "--keep-table-collision-during-motion",
        dest="remove_table_collision_during_motion",
        action="store_false",
        help="Deprecated compatibility flag. Keeping collision is now the default.",
    )
    return parser.parse_args(argv)


def wait_for_moveit_ready(args):
    """等待 launch 中的 MoveIt/JointState 真正就绪，避免一启动就规划导致空跑。"""
    rospy.wait_for_service("/get_planning_scene", timeout=args.startup_timeout)
    rospy.wait_for_message(args.joint_states_topic, JointState, timeout=args.startup_timeout)


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    data = load_store(args.store)
    start_wp = get_waypoint(data, args.start)
    end_wp = get_waypoint(data, args.end)
    start_joints = waypoint_joints(start_wp)
    end_joints = waypoint_joints(end_wp)
    args.reference_frame = resolve_reference_frame(args, start_wp, end_wp)
    start_pose = waypoint_xyz_rpy(start_wp)
    end_pose = waypoint_xyz_rpy(end_wp)
    geometry = derive_geometry(start_pose, end_pose, args)
    print_report(args, start_wp, end_wp, start_joints, end_joints, start_pose, end_pose, geometry)
    print("Command mode:         ", args.command)

    if args.report_only:
        return 0

    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running. Start Gazebo + MoveIt first.")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_12_waypoint_table_line_calibration", anonymous=True)
    wait_for_moveit_ready(args)
    scene = PlanningSceneInterface()
    scene_pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=5, latch=True)
    group = MoveGroupCommander(args.group)
    group.set_pose_reference_frame(args.reference_frame)
    group.set_planning_time(args.planning_time)
    group.set_num_planning_attempts(args.planning_attempts)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)
    rospy.sleep(1.0)

    add_table(scene, scene_pub, args, geometry)

    do_plan_start = args.plan_start or args.go_start or args.command in ("start", "start-plan-line", "start-execute-line")
    do_go_start = args.go_start or args.command in ("start", "start-plan-line", "start-execute-line")
    do_plan_line = args.plan_line or args.execute_line or args.command in (
        "plan-line",
        "execute-line",
        "start-plan-line",
        "start-execute-line",
    )
    do_execute_line = args.execute_line or args.command in ("execute-line", "start-execute-line")
    do_motion = do_plan_start or do_plan_line

    if do_motion and args.remove_table_collision_during_motion:
        remove_table_for_contact_motion(scene, args)

    if do_plan_start:
        ok, plan = plan_movej(group, start_joints)
        if do_go_start:
            if not ok:
                print("Refusing start execution because MoveJ plan failed.")
                if args.remove_table_collision_during_motion:
                    add_table(scene, scene_pub, args, geometry)
                return 3
            print("Executing MoveJ to start waypoint...")
            executed = group.execute(plan, wait=True)
            group.stop()
            rospy.sleep(1.0)
            print("MoveJ execute result:", executed)
            if not executed:
                if args.remove_table_collision_during_motion:
                    add_table(scene, scene_pub, args, geometry)
                return 4

    if do_plan_line:
        end_pose_stamped = pose_stamped_from_xyz_rpy(args.reference_frame, end_pose)
        fraction, points, plan = plan_movel(group, args, end_pose_stamped)
        if do_execute_line:
            if fraction < args.min_fraction or points == 0:
                print("Refusing line execution because fraction/points did not pass.")
                if args.remove_table_collision_during_motion:
                    add_table(scene, scene_pub, args, geometry)
                return 5
            print("Executing MoveL to end waypoint...")
            executed = group.execute(plan, wait=True)
            group.stop()
            rospy.sleep(1.0)
            print("MoveL execute result:", executed)
            if not executed:
                if args.remove_table_collision_during_motion:
                    add_table(scene, scene_pub, args, geometry)
                return 6

    if do_motion and args.remove_table_collision_during_motion:
        add_table(scene, scene_pub, args, geometry)

    return 0


if __name__ == "__main__":
    sys.exit(main())
