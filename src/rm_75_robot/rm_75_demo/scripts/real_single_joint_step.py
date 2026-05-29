#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import math
import sys

import moveit_commander
import rospy
from moveit_msgs.msg import MoveItErrorCodes


# =========================
# 1. 全局配置层
# =========================
#
# 这个脚本只用于“实物机械臂单关节小步测试”：
# - 只控制 MoveIt 规划组 arm
# - 只允许 joint1~joint7 中的一个关节做相对运动
# - 默认只 Plan，不 Execute；带 --execute 才真正运动
# - 限制单次角度、速度、加速度，避免误输入造成大动作
GROUP_NAME = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
MAX_DEGREES = 20.0
MAX_SCALING = 0.20
VERIFY_TOLERANCE_DEG = 1.0


# =========================
# 2. 小工具函数层
# =========================

def fmt(values):
    """把关节角列表格式化成短一点的字符串，方便看日志。"""
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def normalize_plan_result(plan_result):
    """兼容不同 MoveIt 版本的 group.plan() 返回格式。

    ROS Noetic 下常见返回：
        (success, plan, planning_time, error_code)

    部分旧接口可能直接返回 RobotTrajectory。
    统一整理成：
        success: bool
        plan: RobotTrajectory
        error_code: MoveItErrorCodes 或 None
    """
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code

    return bool(plan_result.joint_trajectory.points), plan_result, None


def error_name(error_code):
    """把 MoveItErrorCodes 数字结果转成人能看懂的名字。"""
    if error_code is None:
        return "unknown"

    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


# =========================
# 3. 命令行参数层
# =========================

def parse_args(argv):
    """解析用户在命令行里输入的控制参数。"""
    parser = argparse.ArgumentParser(
        description="Plan or execute a bounded single-joint step on the real robot."
    )
    parser.add_argument(
        "--joint",
        choices=JOINT_NAMES,
        required=True,
        help="Joint to move, for example joint6.",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        required=True,
        help="Relative joint step in degrees. Positive and negative are both allowed.",
    )
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.05)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute the planned trajectory. Without this flag, only plans.",
    )
    return parser.parse_args(argv)


def validate_args(args):
    """实物安全保护层：先挡住危险输入，再接触 MoveIt。

    注意：
    - --degrees 是角度 degree，不是 rad
    - 单次最多允许 +/-20 度
    - 速度/加速度缩放最大 0.20，也就是 20%
    """
    if abs(args.degrees) > MAX_DEGREES:
        print(
            "Refusing to run: --degrees {:.2f} exceeds +/-{:.1f}.".format(
                args.degrees, MAX_DEGREES
            )
        )
        return 2

    if not 0.0 < args.velocity <= MAX_SCALING:
        print("Refusing to run: --velocity must be in (0, {:.2f}].".format(MAX_SCALING))
        return 2

    if not 0.0 < args.acceleration <= MAX_SCALING:
        print(
            "Refusing to run: --acceleration must be in (0, {:.2f}].".format(
                MAX_SCALING
            )
        )
        return 2

    return 0


# =========================
# 4. MoveIt 初始化层
# =========================

def make_move_group(args):
    """初始化 MoveIt commander，并拿到 arm 规划组句柄。

    MoveGroupCommander(GROUP_NAME) 后面所有动作都围绕这个 group：
    - 读取当前关节角
    - 设置目标关节角
    - 调用 plan()
    - 调用 execute()
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("real_single_joint_step", anonymous=True)

    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(8.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)

    # 等待 move_group、robot_state_publisher、/joint_states 等数据稍微稳定。
    rospy.sleep(1.0)
    return group


# =========================
# 5. 目标关节角生成层
# =========================

def build_single_joint_target(group, args):
    """从当前真实关节角出发，只修改用户指定的那一个关节。

    这里用相对运动：
        target[joint] = current[joint] + delta

    这样比直接写绝对角度更适合初学调试，因为每次都是从当前姿态小步移动。
    """
    delta = math.radians(args.degrees)

    current = group.get_current_joint_values()
    target = list(current)
    joint_index = JOINT_NAMES.index(args.joint)
    target[joint_index] += delta
    return current, target, delta


def print_request_summary(args, current, target, delta):
    """执行前打印本次请求，方便人眼确认目标有没有离谱。"""
    print("Planning group:", GROUP_NAME)
    print("Mode:", "EXECUTE" if args.execute else "PLAN ONLY")
    print("Joint:", args.joint)
    print("Delta: {:.2f} deg / {:.4f} rad".format(args.degrees, delta))
    print("Velocity scaling:", args.velocity)
    print("Acceleration scaling:", args.acceleration)
    print("Current joints:", fmt(current))
    print("Target joints: ", fmt(target))


# =========================
# 6. MoveIt 规划层
# =========================

def plan_target(group, target):
    """把目标关节角交给 MoveIt，然后只做规划。

    set_joint_value_target() 会先检查目标是否在关节上下限内。
    如果目标超限，MoveIt 会在这里直接抛异常，不会执行真实机械臂。
    """
    group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
    success, plan, error_code = normalize_plan_result(group.plan())

    point_count = len(plan.joint_trajectory.points)
    print("Plan success:", success)
    print("MoveIt code:  ", error_name(error_code))
    print("Trajectory points:", point_count)

    if not success or point_count == 0:
        print("No executable plan was generated.")
        return None

    return plan


# =========================
# 7. 执行层
# =========================

def execute_if_requested(group, plan, execute):
    """根据 --execute 决定是否真的把轨迹发给实物机械臂。

    不带 --execute：
        只验证 MoveIt 能规划，不会运动。

    带 --execute：
        group.execute() 会把轨迹交给 MoveIt controller manager，
        然后发到 /rm_75/follow_joint_trajectory，
        再由 rm_75_control -> rm_75_driver -> 实物机械臂。
    """
    if not execute:
        print("Dry run complete. Re-run with --execute to move the real robot.")
        return True, None

    print("Executing single-joint trajectory...")
    executed = group.execute(plan, wait=True)

    # stop() 和 clear_pose_targets() 是执行后的收尾动作：
    # - stop() 确保 MoveIt 不再继续残留运动
    # - clear_pose_targets() 清除本次目标，避免影响下一次命令
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    after = group.get_current_joint_values()
    print("Execute result:", executed)
    print("After joints:  ", fmt(after))
    return bool(executed), after


def verify_feedback(args, target, after):
    """用真实 /joint_states 反馈校验指定关节是否真的到位。

    rm_75_control 当前会在 action 流程结束后 setSucceeded()，但这个成功
    并不等价于机械臂已经到达目标。因此这里额外比较目标角和反馈角。
    """
    joint_index = JOINT_NAMES.index(args.joint)
    error = abs(after[joint_index] - target[joint_index])
    tolerance = math.radians(VERIFY_TOLERANCE_DEG)

    print(
        "Feedback check {} error: {:.2f} deg".format(
            args.joint, math.degrees(error)
        )
    )

    if error > tolerance:
        print(
            "WARNING: action finished, but {} did not reach the target within {:.1f} deg.".format(
                args.joint, VERIFY_TOLERANCE_DEG
            )
        )
        print("Check /rm_driver/JointPos, rm_driver logs, robot mode, enable state, and joint limits.")
        return False

    return True


# =========================
# 8. 主流程编排层
# =========================

def main():
    args = parse_args(sys.argv[1:])

    ret = validate_args(args)
    if ret != 0:
        return ret

    group = make_move_group(args)
    current, target, delta = build_single_joint_target(group, args)
    print_request_summary(args, current, target, delta)

    plan = plan_target(group, target)
    if plan is None:
        moveit_commander.roscpp_shutdown()
        return 1

    ok, after = execute_if_requested(group, plan, args.execute)
    if ok and after is not None:
        ok = verify_feedback(args, target, after)
    moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
