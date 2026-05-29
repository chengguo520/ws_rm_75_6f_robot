#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import sys

import moveit_commander
import rospy
from moveit_msgs.msg import MoveItErrorCodes


GROUP_NAME = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
WRIST_JOINTS = ["joint5", "joint6", "joint7"]
MAX_DELTA = 0.10
MAX_SCALING = 0.20


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def normalize_plan_result(plan_result):
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


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Plan or execute a tiny real-robot wrist joint step."
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=0.03,
        help="Joint delta in radians for joint5, joint6, and joint7.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute the planned trajectory. Without this flag, only plans.",
    )
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.05)
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])

    if abs(args.delta) > MAX_DELTA:
        print(
            "Refusing to run: --delta {:.4f} rad is too large for this safety test.".format(
                args.delta
            )
        )
        print("Use a value within +/-{:.2f} rad. Example: --delta 0.03".format(MAX_DELTA))
        return 2

    if not 0.0 < args.velocity <= MAX_SCALING:
        print(
            "Refusing to run: --velocity must be in (0, {:.2f}].".format(MAX_SCALING)
        )
        return 2

    if not 0.0 < args.acceleration <= MAX_SCALING:
        print(
            "Refusing to run: --acceleration must be in (0, {:.2f}].".format(MAX_SCALING)
        )
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("real_wrist_small_step", anonymous=True)

    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(8.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)

    rospy.sleep(1.0)

    current = group.get_current_joint_values()
    target = list(current)
    for joint_name in WRIST_JOINTS:
        target[JOINT_NAMES.index(joint_name)] += args.delta

    print("Planning group:", GROUP_NAME)
    print("Mode:", "EXECUTE" if args.execute else "PLAN ONLY")
    print("Velocity scaling:", args.velocity)
    print("Acceleration scaling:", args.acceleration)
    print("Delta for joint5-7: {:.4f} rad".format(args.delta))
    print("Current joints:", fmt(current))
    print("Target joints: ", fmt(target))

    group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
    success, plan, error_code = normalize_plan_result(group.plan())

    point_count = len(plan.joint_trajectory.points)
    print("Plan success:", success)
    print("MoveIt code:  ", error_name(error_code))
    print("Trajectory points:", point_count)

    if not success or point_count == 0:
        print("No executable plan was generated.")
        moveit_commander.roscpp_shutdown()
        return 1

    if not args.execute:
        print("Dry run complete. Re-run with --execute to move the real robot.")
        moveit_commander.roscpp_shutdown()
        return 0

    print("Executing tiny wrist trajectory...")
    executed = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    after = group.get_current_joint_values()
    print("Execute result:", executed)
    print("After joints:  ", fmt(after))

    moveit_commander.roscpp_shutdown()
    return 0 if executed else 1


if __name__ == "__main__":
    sys.exit(main())
