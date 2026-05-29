#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys

import moveit_commander
import rospy
from moveit_msgs.msg import MoveItErrorCodes


GROUP_NAME = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def normalize_plan_result(plan_result):
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code

    points = plan_result.joint_trajectory.points
    return bool(points), plan_result, None


def error_name(error_code):
    if error_code is None:
        return "unknown"

    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_joint_goal_demo", anonymous=True)

    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    rospy.sleep(1.0)

    current = group.get_current_joint_values()
    print("Planning group:", GROUP_NAME)
    print("Current joints:", fmt(current))

    target = list(current)
    target[1] = 0.25
    target[3] = 0.35
    target[5] = 0.20
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

    print("Executing trajectory...")
    executed = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    after = group.get_current_joint_values()
    print("Execute result:", executed)
    print("After joints:  ", fmt(after))

    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
