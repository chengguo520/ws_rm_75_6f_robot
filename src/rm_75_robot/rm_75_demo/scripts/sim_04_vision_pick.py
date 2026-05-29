#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import math
import sys

import moveit_commander
import rosgraph
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import CameraInfo, Image


GROUP_NAME = "arm"
EEF_LINK = "gripper_tcp_link"
BASE_FRAME = "base_link"

TABLE_ID = "vision_pick_table"
OBJECT_ID = "vision_pick_object"
STALE_SCENE_IDS = (
    "vision_table",
    "vision_cube",
    "pick_table",
    "pick_object",
)

TABLE_SIZE = (0.60, 0.80, 0.04)
TABLE_CENTER = (0.45, 0.00, 0.28)
OBJECT_SIZE = (0.05, 0.05, 0.05)


def import_cv_tools():
    try:
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        return cv2, np, CvBridge
    except Exception as exc:
        raise RuntimeError("OpenCV/cv_bridge is required. Missing import: {}".format(exc))


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


def wait_msg(topic, msg_type, timeout):
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except rospy.ROSException:
        raise RuntimeError("Timeout waiting for topic '{}'.".format(topic))


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
    # TCP X points toward the cube, TCP Y is finger opening direction.
    return quaternion_from_rpy(tcp_roll, 0.0, tcp_yaw)


def color_mask(cv2, np, bgr_image, color_name):
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    if color_name == "red":
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 80, 80]), np.array([180, 255, 255]))
        return cv2.bitwise_or(mask1, mask2)
    if color_name == "blue":
        return cv2.inRange(hsv, np.array([95, 70, 60]), np.array([130, 255, 255]))
    if color_name == "green":
        return cv2.inRange(hsv, np.array([40, 70, 60]), np.array([85, 255, 255]))
    raise ValueError("Unsupported color '{}'. Use red, blue, or green.".format(color_name))


def detect_centroid(cv2, np, bgr_image, color_name, min_area):
    mask = color_mask(cv2, np, bgr_image, color_name)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("No '{}' object found in image.".format(color_name))

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        raise RuntimeError("Detected '{}' area {:.1f} is too small.".format(color_name, area))

    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        raise RuntimeError("Invalid image moment for detected object.")
    u = int(moments["m10"] / moments["m00"])
    v = int(moments["m01"] / moments["m00"])
    return u, v, area


def depth_at_pixel(np, depth_image, u, v, window):
    height, width = depth_image.shape[:2]
    u0 = max(0, u - window)
    u1 = min(width, u + window + 1)
    v0 = max(0, v - window)
    v1 = min(height, v + window + 1)
    roi = depth_image[v0:v1, u0:u1].astype("float32")
    valid = roi[np.isfinite(roi)]
    valid = valid[valid > 0.0]
    if valid.size == 0:
        raise RuntimeError("No valid depth around detected pixel.")
    depth = float(np.median(valid))
    if depth > 20.0:
        depth = depth / 1000.0
    return depth


def pixel_to_camera_point(camera_info, u, v, depth):
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    if fx == 0.0 or fy == 0.0:
        raise RuntimeError("CameraInfo intrinsics are invalid.")

    point = PointStamped()
    point.header = camera_info.header
    point.point.x = (u - cx) * depth / fx
    point.point.y = (v - cy) * depth / fy
    point.point.z = depth
    return point


def transform_point(tf_buffer, point, target_frame):
    try:
        return tf_buffer.transform(point, target_frame, timeout=rospy.Duration(1.0))
    except Exception as exc:
        raise RuntimeError("TF transform to '{}' failed: {}".format(target_frame, exc))


def detect_object_pose(args, bridge, tf_buffer):
    cv2, np, _unused = import_cv_tools()
    image_msg = wait_msg(args.image_topic, Image, args.timeout)
    depth_msg = wait_msg(args.depth_topic, Image, args.timeout)
    camera_info = wait_msg(args.camera_info_topic, CameraInfo, args.timeout)

    bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    u, v, area = detect_centroid(cv2, np, bgr, args.color, args.min_area)
    depth_m = depth_at_pixel(np, depth, u, v, args.depth_window)
    camera_point = pixel_to_camera_point(camera_info, u, v, depth_m)
    base_point = transform_point(tf_buffer, camera_point, BASE_FRAME)

    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = base_point.point.x
    pose.pose.position.y = base_point.point.y
    pose.pose.position.z = base_point.point.z
    pose.pose.orientation.w = 1.0

    print("Detected pixel:", u, v, "area:", "{:.1f}".format(area), "depth:", "{:.3f} m".format(depth_m))
    print(
        "Detected object base xyz:",
        "{:.3f}, {:.3f}, {:.3f}".format(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ),
    )
    return pose


def prepare_scene(scene, object_pose):
    for object_id in [TABLE_ID, OBJECT_ID] + list(STALE_SCENE_IDS):
        scene.remove_world_object(object_id)
    scene.remove_attached_object(EEF_LINK, name=OBJECT_ID)
    rospy.sleep(0.5)

    scene.add_box(TABLE_ID, make_pose_stamped(BASE_FRAME, TABLE_CENTER), size=TABLE_SIZE)
    scene.add_box(OBJECT_ID, object_pose, size=OBJECT_SIZE)
    rospy.sleep(0.8)
    print("MoveIt planning scene prepared:")
    print("- table:", TABLE_ID, TABLE_CENTER)
    print(
        "- object:",
        OBJECT_ID,
        "({:.3f}, {:.3f}, {:.3f})".format(
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z,
        ),
    )


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


def cleanup_scene(scene):
    scene.remove_attached_object(EEF_LINK, name=OBJECT_ID)
    rospy.sleep(0.3)
    for object_id in [OBJECT_ID, TABLE_ID]:
        scene.remove_world_object(object_id)
    rospy.sleep(0.3)


def make_pick_poses(object_pose, args):
    orientation = side_grasp_orientation(math.radians(args.tcp_roll_deg), math.radians(args.tcp_yaw_deg))

    pre_grasp = Pose()
    pre_grasp.position.x = object_pose.pose.position.x - args.pre_grasp_offset
    pre_grasp.position.y = object_pose.pose.position.y
    pre_grasp.position.z = object_pose.pose.position.z + args.grasp_z_offset
    pre_grasp.orientation = orientation

    grasp = copy_pose(pre_grasp)
    grasp.position.x = object_pose.pose.position.x - args.grasp_offset

    lift = copy_pose(grasp)
    lift.position.z += args.lift_distance
    return pre_grasp, grasp, lift


def run_vision_pick(group, scene, object_pose, args):
    prepare_scene(scene, object_pose)
    pre_grasp, grasp, lift = make_pick_poses(object_pose, args)

    if args.use_gripper_topic:
        publish_gripper(open_gripper=True)

    pre_plan = plan_pose(group, pre_grasp, "MoveJ pre-grasp", args.allow_position_fallback)
    if not execute_plan(group, pre_plan, args.execute):
        return False

    if not args.execute:
        print("Plan-only mode: checked vision + pre-grasp. Use --execute for grasp/lift.")
        return True

    scene.remove_world_object(OBJECT_ID)
    rospy.sleep(0.3)
    grasp_plan = plan_cartesian(group, [grasp], "MoveL grasp", args.velocity, args.acceleration)
    if not execute_plan(group, grasp_plan, args.execute):
        return False

    if args.use_gripper_topic:
        publish_gripper(open_gripper=False)
        rospy.sleep(0.8)

    attach_object(scene, group, object_pose)
    lift_plan = plan_cartesian(group, [lift], "MoveL lift", args.velocity, args.acceleration)
    if not execute_plan(group, lift_plan, args.execute):
        return False

    print("Vision pick sequence finished.")
    return True


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Simulation step 04: vision-guided side pick with gripper TCP.")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--color", choices=["red", "blue", "green"], default="blue")
    parser.add_argument("--min-area", type=float, default=200.0)
    parser.add_argument("--depth-window", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--use-gripper-topic", action="store_true")
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--acceleration", type=float, default=0.2)
    parser.add_argument("--pre-grasp-offset", type=float, default=0.04)
    parser.add_argument("--grasp-offset", type=float, default=0.0)
    parser.add_argument("--grasp-z-offset", type=float, default=0.02)
    parser.add_argument("--lift-distance", type=float, default=0.12)
    parser.add_argument("--tcp-roll-deg", type=float, default=0.0)
    parser.add_argument("--tcp-yaw-deg", type=float, default=0.0)
    parser.add_argument("--allow-position-fallback", action="store_true")
    parser.add_argument("--keep-scene", action="store_true", help="Keep table/object in planning scene after the script exits.")
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt + RGB-D camera first:")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch \\")
        print("    world_name:=$(rospack find rm_75_gazebo)/worlds/rm75_table_cube.world \\")
        print("    use_rgbd_camera:=true use_two_finger_gripper:=true rviz_software_rendering:=true")
        return 2

    cv2, np, CvBridge = import_cv_tools()
    _unused = (cv2, np)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_04_vision_pick", anonymous=True)

    bridge = CvBridge()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

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
    rospy.sleep(0.8)

    object_pose = detect_object_pose(args, bridge, tf_buffer)
    ok = False
    try:
        ok = run_vision_pick(group, scene, object_pose, args)
    finally:
        if not args.keep_scene and args.execute:
            cleanup_scene(scene)
        moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
