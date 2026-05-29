#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import sys

import moveit_commander
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes, ObjectColor, PlanningScene


GROUP_NAME = "arm"
EEF_LINK = "link7"
REFERENCE_FRAME = "base_link"

TABLE_ID = "demo_table"
CUBE_ID = "demo_cube"

TABLE_SIZE = (0.60, 0.80, 0.04)
CUBE_SIZE = (0.05, 0.05, 0.05)
TABLE_CENTER = (0.45, 0.00, 0.28)
CUBE_CENTER = (0.45, 0.10, 0.325)


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


def wait_for_known(scene, object_id, timeout=3.0):
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < timeout and not rospy.is_shutdown():
        if object_id in scene.get_known_object_names():
            return True
        rospy.sleep(0.1)
    return False


def add_moveit_scene(scene, scene_pub):
    for object_id in [TABLE_ID, CUBE_ID]:
        scene.remove_world_object(object_id)
    rospy.sleep(0.5)

    table_pose = make_pose_stamped(REFERENCE_FRAME, TABLE_CENTER)
    cube_pose = make_pose_stamped(REFERENCE_FRAME, CUBE_CENTER)

    scene.add_box(TABLE_ID, table_pose, size=TABLE_SIZE)
    scene.add_box(CUBE_ID, cube_pose, size=CUBE_SIZE)

    wait_for_known(scene, TABLE_ID)
    wait_for_known(scene, CUBE_ID)

    set_scene_color(scene_pub, TABLE_ID, (0.45, 0.28, 0.14, 1.0))
    set_scene_color(scene_pub, CUBE_ID, (0.1, 0.45, 0.95, 1.0))

    print("MoveIt planning scene:")
    print("- table:", TABLE_CENTER, "size:", TABLE_SIZE)
    print("- cube: ", CUBE_CENTER, "size:", CUBE_SIZE)


def spawn_gazebo_box(model_name, xyz, size, rgba):
    try:
        from gazebo_msgs.srv import SpawnModel
    except Exception as exc:
        rospy.logwarn("gazebo_msgs is not available: %s", exc)
        return False

    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    sdf = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <inertial><mass>1.0</mass></inertial>
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
        spawn(model_name, sdf, "", pose, "world")
        print("Spawned Gazebo model:", model_name)
        return True
    except Exception as exc:
        rospy.logwarn("Could not spawn Gazebo model '%s': %s", model_name, exc)
        return False


def spawn_gazebo_scene():
    spawn_gazebo_box(TABLE_ID, TABLE_CENTER, TABLE_SIZE, (0.45, 0.28, 0.14, 1.0))
    spawn_gazebo_box(CUBE_ID, CUBE_CENTER, CUBE_SIZE, (0.1, 0.45, 0.95, 1.0))


def plan_to_cube_approach(group, execute=False):
    current_pose = group.get_current_pose(EEF_LINK).pose
    target = PoseStamped()
    target.header.frame_id = REFERENCE_FRAME
    target.pose.position.x = CUBE_CENTER[0]
    target.pose.position.y = CUBE_CENTER[1]
    target.pose.position.z = CUBE_CENTER[2] + 0.12
    target.pose.orientation = current_pose.orientation

    group.set_start_state_to_current_state()
    group.set_pose_target(target, EEF_LINK)
    success, plan, error_code = normalize_plan_result(group.plan())
    group.clear_pose_targets()

    point_count = len(plan.joint_trajectory.points)
    print("Plan to cube approach:", success, error_name(error_code), "points:", point_count)
    print(
        "Approach pose xyz:",
        "{:.3f}, {:.3f}, {:.3f}".format(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ),
    )

    if execute and success and point_count:
        ok = group.execute(plan, wait=True)
        group.stop()
        print("Execute result:", ok)

    return success and point_count > 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Simulation step 01: table/cube planning scene and approach plan.")
    parser.add_argument("--spawn-gazebo", action="store_true", help="Also spawn simple SDF table/cube in Gazebo.")
    parser.add_argument("--execute", action="store_true", help="Execute the planned approach trajectory.")
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--acceleration", type=float, default=0.2)
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
    rospy.init_node("sim_01_scene_plan", anonymous=True)

    scene = PlanningSceneInterface()
    scene_pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=5)
    group = MoveGroupCommander(GROUP_NAME)
    group.set_end_effector_link(EEF_LINK)
    group.set_pose_reference_frame(REFERENCE_FRAME)
    group.set_planning_time(8.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)

    rospy.sleep(1.0)
    add_moveit_scene(scene, scene_pub)
    if args.spawn_gazebo:
        spawn_gazebo_scene()

    ok = plan_to_cube_approach(group, execute=args.execute)
    moveit_commander.roscpp_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
