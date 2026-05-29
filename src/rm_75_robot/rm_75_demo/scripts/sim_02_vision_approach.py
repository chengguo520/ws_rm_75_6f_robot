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
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from sensor_msgs.msg import CameraInfo, Image


GROUP_NAME = "arm"
DEFAULT_EEF_LINK = "gripper_tcp_link"
BASE_FRAME = "base_link"
OBJECT_ID = "vision_cube"
OBJECT_SIZE = (0.05, 0.05, 0.05)
TABLE_ID = "vision_table"
TABLE_SIZE = (0.60, 0.80, 0.04)
TABLE_CENTER = (0.45, 0.00, 0.28)
DEMO_CUBE_CENTER = (0.45, 0.10, 0.325)


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    pose = PoseStamped()
    pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
    pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
    return pose.pose.orientation


def side_grasp_orientation(tcp_roll, tcp_yaw):
    # TCP frame for side grasp:
    #   X points from the gripper toward the cube.
    #   Y is the finger opening direction, parallel to the table.
    #   Z is vertical, so the gripper's main flat plane stays parallel to the table.
    return quaternion_from_rpy(tcp_roll, 0.0, tcp_yaw)


def import_cv_tools():
    try:
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        return cv2, np, CvBridge
    except Exception as exc:
        raise RuntimeError(
            "OpenCV/cv_bridge is required for this script. Missing import: {}".format(exc)
        )


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


def spawn_gazebo_box(model_name, xyz, size, rgba):
    try:
        from gazebo_msgs.srv import DeleteModel, SpawnModel
    except Exception as exc:
        rospy.logwarn("gazebo_msgs is not available: %s", exc)
        return False

    try:
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=3.0)
    except rospy.ROSException:
        rospy.logwarn("Gazebo spawn service is not available. Skipping Gazebo model spawn.")
        return False

    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=1.0)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        rospy.sleep(0.2)
    except Exception:
        pass

    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    sdf = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
        name=model_name,
        sx=size[0],
        sy=size[1],
        sz=size[2],
        r=rgba[0],
        g=rgba[1],
        b=rgba[2],
        a=rgba[3],
    )

    pose = make_pose_stamped("world", xyz).pose
    try:
        spawn_model(model_name, sdf, "", pose, "world")
        print("Spawned Gazebo model:", model_name)
        return True
    except Exception as exc:
        rospy.logwarn("Could not spawn Gazebo model '%s': %s", model_name, exc)
        return False


def setup_demo_scene(scene, spawn_gazebo):
    scene.remove_world_object(TABLE_ID)
    scene.remove_world_object(OBJECT_ID)
    rospy.sleep(0.3)

    table_pose = make_pose_stamped(BASE_FRAME, TABLE_CENTER)
    cube_pose = make_pose_stamped(BASE_FRAME, DEMO_CUBE_CENTER)
    scene.add_box(TABLE_ID, table_pose, size=TABLE_SIZE)
    scene.add_box(OBJECT_ID, cube_pose, size=OBJECT_SIZE)

    if spawn_gazebo:
        spawn_gazebo_box(TABLE_ID, TABLE_CENTER, TABLE_SIZE, (0.45, 0.28, 0.14, 1.0))
        spawn_gazebo_box(OBJECT_ID, DEMO_CUBE_CENTER, OBJECT_SIZE, (0.0, 0.0, 1.0, 1.0))

    rospy.sleep(1.0)
    print("Demo scene prepared:")
    print("- table xyz:", TABLE_CENTER)
    print("- blue cube xyz:", DEMO_CUBE_CENTER)


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
        raise RuntimeError(
            "No '{}' object found in image. Check rqt_image_view and make sure the colored cube exists in Gazebo, "
            "not only in MoveIt/RViz planning scene.".format(color_name)
        )

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


def add_detected_object(scene, base_point):
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = base_point.point.x
    pose.pose.position.y = base_point.point.y
    pose.pose.position.z = base_point.point.z
    pose.pose.orientation.w = 1.0

    scene.remove_world_object(OBJECT_ID)
    rospy.sleep(0.2)
    scene.add_box(OBJECT_ID, pose, size=OBJECT_SIZE)
    return pose


def plan_approach(
    group,
    object_pose,
    approach_z,
    side_offset,
    approach_mode,
    eef_link,
    tcp_roll,
    tcp_yaw,
    allow_position_fallback,
    execute,
):
    current_pose = group.get_current_pose(eef_link).pose
    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    if approach_mode == "side":
        target.pose.position.x = object_pose.pose.position.x - side_offset
        target.pose.position.y = object_pose.pose.position.y
        target.pose.position.z = object_pose.pose.position.z
        target.pose.orientation = side_grasp_orientation(tcp_roll, tcp_yaw)
    else:
        target.pose.position.x = object_pose.pose.position.x
        target.pose.position.y = object_pose.pose.position.y
        target.pose.position.z = object_pose.pose.position.z + approach_z
        target.pose.orientation = current_pose.orientation

    group.set_start_state_to_current_state()
    group.set_pose_target(target, eef_link)
    success, plan, error_code = normalize_plan_result(group.plan())
    group.clear_pose_targets()

    point_count = len(plan.joint_trajectory.points)
    print("Vision approach pose plan:", success, error_name(error_code), "points:", point_count)
    print(
        "TCP target xyz:",
        "{:.3f}, {:.3f}, {:.3f}".format(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ),
        "mode:",
        approach_mode,
        "eef:",
        eef_link,
    )

    if allow_position_fallback and (not success or point_count == 0):
        print("Pose target failed. Retrying with position-only target...")
        group.set_start_state_to_current_state()
        group.set_position_target(
            [
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            ],
            eef_link,
        )
        success, plan, error_code = normalize_plan_result(group.plan())
        group.clear_pose_targets()
        point_count = len(plan.joint_trajectory.points)
        print("Vision approach position-only plan:", success, error_name(error_code), "points:", point_count)

    print(
        "Detected object base xyz:",
        "{:.3f}, {:.3f}, {:.3f}".format(
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z,
        ),
    )

    if execute and success and point_count:
        ok = group.execute(plan, wait=True)
        group.stop()
        print("Execute result:", ok)
    return success and point_count > 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Simulation step 02: color/depth vision detection and approach plan.")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--color", choices=["red", "blue", "green"], default="blue")
    parser.add_argument("--min-area", type=float, default=200.0)
    parser.add_argument("--depth-window", type=int, default=3)
    parser.add_argument("--approach-z", type=float, default=0.12)
    parser.add_argument("--side-offset", type=float, default=0.04)
    parser.add_argument("--approach-mode", choices=["side", "top"], default="side")
    parser.add_argument("--eef-link", default=DEFAULT_EEF_LINK)
    parser.add_argument("--tcp-roll-deg", type=float, default=0.0)
    parser.add_argument("--tcp-yaw-deg", type=float, default=0.0)
    parser.add_argument("--allow-position-fallback", action="store_true")
    parser.add_argument("--no-setup-scene", action="store_true", help="Do not add demo table/cube collision objects to MoveIt.")
    parser.add_argument("--spawn-gazebo", action="store_true", help="Spawn demo table/cube into Gazebo if they are not in the world file.")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--acceleration", type=float, default=0.2)
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt first, and make sure camera topics exist.")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch")
        return 2

    cv2, np, CvBridge = import_cv_tools()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_02_vision_approach", anonymous=True)

    bridge = CvBridge()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    scene = PlanningSceneInterface()
    scene_pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=5)
    _unused = scene_pub
    group = MoveGroupCommander(GROUP_NAME)
    group.set_end_effector_link(args.eef_link)
    group.set_pose_reference_frame(BASE_FRAME)
    group.set_planning_time(15.0)
    group.set_num_planning_attempts(20)
    group.set_goal_position_tolerance(0.02)
    group.set_goal_orientation_tolerance(0.30)
    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)
    rospy.sleep(0.5)

    if not args.no_setup_scene:
        setup_demo_scene(scene, spawn_gazebo=args.spawn_gazebo)

    image_msg = wait_msg(args.image_topic, Image, args.timeout)
    depth_msg = wait_msg(args.depth_topic, Image, args.timeout)
    camera_info = wait_msg(args.camera_info_topic, CameraInfo, args.timeout)

    bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    u, v, area = detect_centroid(cv2, np, bgr, args.color, args.min_area)
    depth_m = depth_at_pixel(np, depth, u, v, args.depth_window)
    camera_point = pixel_to_camera_point(camera_info, u, v, depth_m)
    base_point = transform_point(tf_buffer, camera_point, BASE_FRAME)

    print("Detected pixel:", u, v, "area:", "{:.1f}".format(area), "depth:", "{:.3f} m".format(depth_m))

    object_pose = add_detected_object(scene, base_point)
    ok = plan_approach(
        group,
        object_pose,
        args.approach_z,
        args.side_offset,
        args.approach_mode,
        args.eef_link,
        math.radians(args.tcp_roll_deg),
        math.radians(args.tcp_yaw_deg),
        args.allow_position_fallback,
        args.execute,
    )

    moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
