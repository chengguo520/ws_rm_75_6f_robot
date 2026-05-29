#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import math
import sys

import moveit_commander
import rosgraph
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes


GROUP_NAME = "arm"
EEF_LINK = "gripper_tcp_link"
BASE_FRAME = "base_link"

TABLE_ID = "pick_table"
OBJECT_ID = "pick_object"
TABLE_SIZE = (0.60, 0.80, 0.04)
OBJECT_SIZE = (0.05, 0.05, 0.05)
TABLE_CENTER = (0.45, 0.00, 0.28)
OBJECT_CENTER = (0.45, 0.10, 0.325)
PRE_GRASP_OFFSET_X = 0.04
GRASP_OFFSET_X = 0.00
STALE_SCENE_IDS = ("vision_table", "vision_cube")


def error_name(error_code):
    if error_code is None:
        return "unknown"
    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


def normalize_plan_result(plan_result):
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code
    return bool(plan_result.joint_trajectory.points), plan_result, None


def make_pose_stamped(frame_id, xyz):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.w = 1.0
    return pose


def copy_pose(pose):
    result = Pose()
    result.position.x = pose.position.x
    result.position.y = pose.position.y
    result.position.z = pose.position.z
    result.orientation.x = pose.orientation.x
    result.orientation.y = pose.orientation.y
    result.orientation.z = pose.orientation.z
    result.orientation.w = pose.orientation.w
    return result


def quaternion_from_rpy(roll, pitch, yaw):
    pose = Pose()
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    return pose.orientation


def side_grasp_orientation(tcp_roll, tcp_yaw):
    return quaternion_from_rpy(tcp_roll, 0.0, tcp_yaw)


def add_scene(scene, object_center):
    for object_id in [TABLE_ID, OBJECT_ID] + list(STALE_SCENE_IDS):
        scene.remove_world_object(object_id)
    rospy.sleep(0.5)

    table_pose = make_pose_stamped(BASE_FRAME, TABLE_CENTER)
    object_pose = make_pose_stamped(BASE_FRAME, object_center)
    scene.add_box(TABLE_ID, table_pose, size=TABLE_SIZE)
    scene.add_box(OBJECT_ID, object_pose, size=OBJECT_SIZE)
    rospy.sleep(0.8)
    print("Added pick table/object to MoveIt planning scene.")
    print("- table xyz:", TABLE_CENTER)
    print("- object xyz:", object_center)
    return object_pose


def describe_pose(label, pose):
    print(
        "{} target xyz: {:.3f}, {:.3f}, {:.3f}".format(
            label,
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
    )


def plan_pose(group, target_pose, label, allow_position_fallback):
    describe_pose(label, target_pose)
    group.set_start_state_to_current_state()
    group.set_pose_target(target_pose, EEF_LINK)
    success, plan, error_code = normalize_plan_result(group.plan())
    group.clear_pose_targets()
    point_count = len(plan.joint_trajectory.points)
    print(label, "plan:", success, error_name(error_code), "points:", point_count)
    if allow_position_fallback and (not success or point_count == 0):
        print(label, "pose plan failed. Retrying position-only target...")
        group.set_start_state_to_current_state()
        group.set_position_target(
            [target_pose.position.x, target_pose.position.y, target_pose.position.z],
            EEF_LINK,
        )
        success, plan, error_code = normalize_plan_result(group.plan())
        group.clear_pose_targets()
        point_count = len(plan.joint_trajectory.points)
        print(label, "position-only plan:", success, error_name(error_code), "points:", point_count)
    if not success or point_count == 0:
        return None
    return plan


def plan_cartesian(group, waypoints, label, velocity, acceleration):
    plan, fraction = group.compute_cartesian_path(waypoints, 0.005, True)
    point_count = len(plan.joint_trajectory.points)
    print(label, "cartesian fraction:", "{:.1f}%".format(fraction * 100.0), "points:", point_count)
    if point_count == 0 or fraction < 0.95:
        return None
    try:
        return group.retime_trajectory(group.get_current_state(), plan, velocity, acceleration)
    except Exception as exc:
        rospy.logwarn("Could not retime %s trajectory: %s", label, exc)
        return plan


def execute_plan(group, plan, execute):
    if plan is None:
        return False
    if not execute:
        return True
    ok = group.execute(plan, wait=True)
    group.stop()
    print("Execute result:", ok)
    return bool(ok)


def publish_gripper(open_gripper):
    try:
        from rm_75_msgs.msg import Gripper_Set
    except Exception as exc:
        rospy.logwarn("rm_75_msgs/Gripper_Set is unavailable: %s", exc)
        return

    pub = rospy.Publisher("/rm_driver/Gripper_Set", Gripper_Set, queue_size=1)
    rospy.sleep(0.3)
    msg = Gripper_Set()
    msg.position = 1000 if open_gripper else 1
    pub.publish(msg)
    print("Published gripper", "open" if open_gripper else "close", "command.")


def attach_object(scene, group, object_pose):
    touch_links = group.get_link_names()
    scene.attach_box(EEF_LINK, OBJECT_ID, object_pose, size=OBJECT_SIZE, touch_links=touch_links)
    rospy.sleep(0.5)
    print("Attached object to", EEF_LINK)


def detach_object(scene):
    scene.remove_attached_object(EEF_LINK, name=OBJECT_ID)
    rospy.sleep(0.5)
    scene.remove_world_object(OBJECT_ID)
    rospy.sleep(0.5)
    print("Detached and removed object from planning scene.")


def run_pick_sequence(
    group,
    scene,
    object_pose,
    execute,
    velocity,
    acceleration,
    use_gripper_topic,
    pre_grasp_offset,
    grasp_offset,
    grasp_z_offset,
    tcp_roll,
    tcp_yaw,
    allow_position_fallback,
):
    pre_grasp = Pose()
    pre_grasp.position.x = object_pose.pose.position.x - pre_grasp_offset
    pre_grasp.position.y = object_pose.pose.position.y
    pre_grasp.position.z = object_pose.pose.position.z + grasp_z_offset
    pre_grasp.orientation = side_grasp_orientation(tcp_roll, tcp_yaw)

    grasp = copy_pose(pre_grasp)
    grasp.position.x = object_pose.pose.position.x - grasp_offset
    grasp.position.y = object_pose.pose.position.y
    grasp.position.z = object_pose.pose.position.z + grasp_z_offset

    lift = copy_pose(grasp)
    lift.position.z += 0.12

    if use_gripper_topic:
        publish_gripper(open_gripper=True)

    pre_plan = plan_pose(group, pre_grasp, "MoveJ pre-grasp", allow_position_fallback)
    if not execute_plan(group, pre_plan, execute):
        return False
    if not execute:
        print("Plan-only mode: checked pre-grasp only. Use --execute to move there before Cartesian grasp/lift.")
        return True

    # The final grasp intentionally places the TCP at the object center.
    # Remove the world collision object before this contact motion, then attach
    # it after the gripper reaches the grasp pose.
    scene.remove_world_object(OBJECT_ID)
    rospy.sleep(0.3)
    grasp_plan = plan_cartesian(group, [grasp], "MoveL descend", velocity, acceleration)
    if not execute_plan(group, grasp_plan, execute):
        return False

    if use_gripper_topic:
        publish_gripper(open_gripper=False)
        rospy.sleep(0.8)

    if execute:
        attach_object(scene, group, object_pose)
    else:
        print("Plan-only mode: object attach is skipped.")

    lift_plan = plan_cartesian(group, [lift], "MoveL lift", velocity, acceleration)
    if not execute_plan(group, lift_plan, execute):
        return False

    print("Pick sequence finished in", "execute" if execute else "plan-only", "mode.")
    return True


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Simulation step 03: gripper pick framework.")
    parser.add_argument("--execute", action="store_true", help="Execute the planned pick sequence.")
    parser.add_argument("--use-gripper-topic", action="store_true", help="Publish /rm_driver/Gripper_Set open/close commands.")
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--acceleration", type=float, default=0.2)
    parser.add_argument("--pre-grasp-offset", type=float, default=PRE_GRASP_OFFSET_X)
    parser.add_argument("--grasp-offset", type=float, default=0.0)
    parser.add_argument("--grasp-z-offset", type=float, default=0.02)
    parser.add_argument("--object-x", type=float, default=OBJECT_CENTER[0])
    parser.add_argument("--object-y", type=float, default=OBJECT_CENTER[1])
    parser.add_argument("--object-z", type=float, default=OBJECT_CENTER[2])
    parser.add_argument("--tcp-roll-deg", type=float, default=0.0)
    parser.add_argument("--tcp-yaw-deg", type=float, default=0.0)
    parser.add_argument("--allow-position-fallback", action="store_true")
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])

    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt first, for example:")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_03_gripper_pick", anonymous=True)

    scene = PlanningSceneInterface()
    group = MoveGroupCommander(GROUP_NAME)
    group.set_end_effector_link(EEF_LINK)
    group.set_pose_reference_frame(BASE_FRAME)
    group.set_planning_time(15.0)
    group.set_num_planning_attempts(20)
    group.set_goal_position_tolerance(0.02)
    group.set_goal_orientation_tolerance(0.30)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)

    rospy.sleep(1.0)
    object_pose = add_scene(scene, (args.object_x, args.object_y, args.object_z))
    ok = run_pick_sequence(
        group,
        scene,
        object_pose,
        execute=args.execute,
        velocity=args.velocity,
        acceleration=args.acceleration,
        use_gripper_topic=args.use_gripper_topic,
        pre_grasp_offset=args.pre_grasp_offset,
        grasp_offset=args.grasp_offset,
        grasp_z_offset=args.grasp_z_offset,
        tcp_roll=math.radians(args.tcp_roll_deg),
        tcp_yaw=math.radians(args.tcp_yaw_deg),
        allow_position_fallback=args.allow_position_fallback,
    )

    if args.execute:
        detach_object(scene)

    moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
