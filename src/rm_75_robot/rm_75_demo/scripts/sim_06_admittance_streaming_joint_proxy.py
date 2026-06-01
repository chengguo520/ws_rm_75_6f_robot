#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import sys

import moveit_commander
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


GROUP_NAME = "arm"
EEF_LINK = "link7"
REFERENCE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_PREPARE_JOINTS = [0.0, 0.25, 0.0, 0.35, 0.0, 0.20, 0.0]

WRENCH_TOPIC = "/rm75_admittance_stream/target_wrench"
TARGET_POSE_TOPIC = "/rm75_admittance_stream/target_pose"
STATE_TOPIC = "/rm75_admittance_stream/state"
COMMAND_TOPIC = "/arm/arm_joint_controller/command"
JOINT_STATES_TOPIC = "/joint_states"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt3(values):
    return "[{:.4f}, {:.4f}, {:.4f}]".format(values[0], values[1], values[2])


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


def copy_pose_stamped(source):
    copied = PoseStamped()
    copied.header.frame_id = source.header.frame_id
    copied.header.stamp = source.header.stamp
    copied.pose.position.x = source.pose.position.x
    copied.pose.position.y = source.pose.position.y
    copied.pose.position.z = source.pose.position.z
    copied.pose.orientation = source.pose.orientation
    return copied


class WrenchInput(object):
    """Keep the newest virtual wrench command for topic-driven tests.

    Topic:
      /rm75_admittance_stream/target_wrench
    Type:
      geometry_msgs/WrenchStamped
    Convention:
      force.x/y/z are Newtons in base_link. This early experiment ignores torque
      because we are still learning translational admittance only.
    """

    def __init__(self, expected_frame, timeout):
        self.expected_frame = expected_frame
        self.timeout = timeout
        self.force = [0.0, 0.0, 0.0]
        self.last_stamp = rospy.Time(0)
        self.warned_frame = False
        self.sub = rospy.Subscriber(WRENCH_TOPIC, WrenchStamped, self._callback, queue_size=1)

    def _callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.expected_frame and not self.warned_frame:
            rospy.logwarn(
                "Received wrench in frame '%s', but this demo assumes '%s'. No TF transform is applied.",
                msg.header.frame_id,
                self.expected_frame,
            )
            self.warned_frame = True

        self.force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.last_stamp = rospy.Time.now()

    def current_force(self):
        if self.last_stamp == rospy.Time(0):
            return [0.0, 0.0, 0.0]
        if (rospy.Time.now() - self.last_stamp).to_sec() > self.timeout:
            return [0.0, 0.0, 0.0]
        return list(self.force)


class TranslationalAdmittance(object):
    """Three-axis virtual mass-damper-spring model.

    Equation:
      M * x_ddot + D * x_dot + K * x = F_ext

    x is not an absolute robot pose. It is the virtual Cartesian offset from the
    equilibrium pose. The streaming backend below maps this small offset into a
    smooth joint-space visualization command.
    """

    def __init__(self, mass, damping, stiffness, max_offset, max_velocity):
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.max_offset = max_offset
        self.max_velocity = max_velocity
        self.offset = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

    def step(self, force, dt):
        dt = clamp(dt, 0.001, 0.2)
        acceleration = [0.0, 0.0, 0.0]

        for i in range(3):
            acceleration[i] = (
                force[i]
                - self.damping[i] * self.velocity[i]
                - self.stiffness[i] * self.offset[i]
            ) / self.mass[i]
            self.velocity[i] += acceleration[i] * dt
            self.velocity[i] = clamp(self.velocity[i], -self.max_velocity[i], self.max_velocity[i])
            self.offset[i] += self.velocity[i] * dt
            self.offset[i] = clamp(self.offset[i], -self.max_offset[i], self.max_offset[i])

        return acceleration


def make_target_pose(equilibrium_pose, offset, reference_frame):
    target = copy_pose_stamped(equilibrium_pose)
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = reference_frame
    target.pose.position.x = equilibrium_pose.pose.position.x + offset[0]
    target.pose.position.y = equilibrium_pose.pose.position.y + offset[1]
    target.pose.position.z = equilibrium_pose.pose.position.z + offset[2]
    return target


def demo_force(args, elapsed):
    if elapsed < args.demo_force_duration:
        return [args.demo_force_x, args.demo_force_y, args.demo_force_z]
    return [0.0, 0.0, 0.0]


def move_to_prepare_joints(group, prepare_joints):
    """Use MoveIt once to get away from the all-zero vertical pose.

    This is intentionally not part of the continuous compliance loop. It is just
    a setup motion so the demo starts from a visible, less singular posture.
    """
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(JOINT_NAMES, prepare_joints)))
    success, plan, error_code = normalize_plan_result(group.plan())

    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo(
        "Prepare joint plan success=%s, points=%d, code=%s",
        success,
        point_count,
        error_name(error_code),
    )
    if not success or point_count == 0:
        raise RuntimeError("Prepare joint plan failed: {}".format(error_name(error_code)))

    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("Prepare joint execute returned: %s", ok)
    if not ok:
        raise RuntimeError("Prepare joint execution failed")


def wait_for_command_connection(pub, timeout):
    """Wait until the JointTrajectoryController subscribes to /command."""
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if pub.get_num_connections() > 0:
            rospy.loginfo("Command topic connected: %s subscribers=%d", COMMAND_TOPIC, pub.get_num_connections())
            return True
        rospy.sleep(0.05)

    rospy.logwarn("No subscriber connected to %s within %.1fs", COMMAND_TOPIC, timeout)
    return False


def wait_for_joint_state(timeout):
    msg = rospy.wait_for_message(JOINT_STATES_TOPIC, JointState, timeout=timeout)
    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("/joint_states missing joints: {}".format(", ".join(missing)))
    rospy.loginfo("Preflight OK: received %d joints from %s", len(msg.name), JOINT_STATES_TOPIC)
    return msg


def map_offset_to_joint_target(equilibrium_joints, offset):
    """Convert virtual Cartesian offset to a small, visible joint motion.

    This is still a learning proxy, not an IK/Jacobian solution. The important
    improvement over sim_05 is not the mapping; it is that the mapped joint target
    is streamed continuously through /arm/arm_joint_controller/command.
    """
    target = list(equilibrium_joints)
    x = clamp(offset[0], -0.10, 0.10)
    y = clamp(offset[1], -0.05, 0.05)
    z = clamp(offset[2], -0.04, 0.04)

    target[0] += 1.0 * y
    target[1] += 1.2 * x - 0.8 * z
    target[3] += -2.8 * x + 1.6 * z
    target[5] += 1.6 * x - 0.8 * z
    return target


def publish_joint_command(pub, target_joints, horizon):
    """Publish one short-horizon JointTrajectory target.

    JointTrajectoryController accepts a new trajectory on /command without the
    action handshake. Re-publishing short-horizon targets at 20-50 Hz feels more
    like a continuous outer-loop command than waiting for one action goal to end.
    """
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = target_joints
    point.velocities = [0.0] * len(JOINT_NAMES)
    point.time_from_start = rospy.Duration(horizon)
    msg.points = [point]

    pub.publish(msg)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="RM75-6F simulation experiment 06: streaming admittance joint proxy."
    )
    parser.add_argument("--duration", type=float, default=16.0, help="Total experiment duration in seconds.")
    parser.add_argument("--rate", type=float, default=30.0, help="Streaming command rate in Hz.")
    parser.add_argument("--command-horizon", type=float, default=0.12, help="JointTrajectory point time_from_start in seconds.")
    parser.add_argument("--prepare", action="store_true", default=True, help="Move to a small bent joint posture first.")
    parser.add_argument("--no-prepare", dest="prepare", action="store_false", help="Skip the initial MoveIt prepare motion.")
    parser.add_argument(
        "--prepare-joints",
        type=float,
        nargs=7,
        default=list(DEFAULT_PREPARE_JOINTS),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
    )
    parser.add_argument("--mass", type=float, nargs=3, default=[2.0, 2.0, 2.0], metavar=("MX", "MY", "MZ"))
    parser.add_argument("--damping", type=float, nargs=3, default=[35.0, 40.0, 40.0], metavar=("DX", "DY", "DZ"))
    parser.add_argument("--stiffness", type=float, nargs=3, default=[45.0, 80.0, 80.0], metavar=("KX", "KY", "KZ"))
    parser.add_argument("--max-offset", type=float, nargs=3, default=[0.10, 0.05, 0.04], metavar=("X", "Y", "Z"))
    parser.add_argument("--max-velocity", type=float, nargs=3, default=[0.06, 0.03, 0.025], metavar=("VX", "VY", "VZ"))
    parser.add_argument("--wrench-timeout", type=float, default=0.5)
    parser.add_argument("--demo-force-x", type=float, default=8.0)
    parser.add_argument("--demo-force-y", type=float, default=0.0)
    parser.add_argument("--demo-force-z", type=float, default=0.0)
    parser.add_argument("--demo-force-duration", type=float, default=6.0)
    parser.add_argument("--use-topic-wrench", action="store_true")
    parser.add_argument("--group", default=GROUP_NAME)
    parser.add_argument("--eef-link", default=EEF_LINK)
    parser.add_argument("--reference-frame", default=REFERENCE_FRAME)
    parser.add_argument("--velocity-scaling", type=float, default=0.10)
    parser.add_argument("--acceleration-scaling", type=float, default=0.10)
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])

    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt first:")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_06_admittance_streaming_joint_proxy", anonymous=True)

    wait_for_joint_state(timeout=3.0)

    # ROS command output:
    #   topic: /arm/arm_joint_controller/command
    #   type:  trajectory_msgs/JointTrajectory
    # This is the streaming path. Unlike FollowJointTrajectoryAction, the node
    # does not wait for each goal to finish before publishing the next update.
    command_pub = rospy.Publisher(COMMAND_TOPIC, JointTrajectory, queue_size=1)
    wait_for_command_connection(command_pub, timeout=3.0)

    group = MoveGroupCommander(args.group)
    group.set_end_effector_link(args.eef_link)
    group.set_pose_reference_frame(args.reference_frame)
    group.set_planning_time(3.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)

    rospy.sleep(0.5)
    if args.prepare:
        move_to_prepare_joints(group, args.prepare_joints)

    equilibrium_pose = group.get_current_pose(args.eef_link)
    equilibrium_pose.header.frame_id = args.reference_frame
    equilibrium_joints = group.get_current_joint_values()

    wrench_input = WrenchInput(args.reference_frame, args.wrench_timeout)
    admittance = TranslationalAdmittance(
        mass=args.mass,
        damping=args.damping,
        stiffness=args.stiffness,
        max_offset=args.max_offset,
        max_velocity=args.max_velocity,
    )

    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)

    print("RM75-6F streaming admittance demo")
    print("Command topic:     ", COMMAND_TOPIC)
    print("Streaming rate:    ", args.rate)
    print("Command horizon:   ", args.command_horizon)
    print("Equilibrium xyz:   ", fmt3([
        equilibrium_pose.pose.position.x,
        equilibrium_pose.pose.position.y,
        equilibrium_pose.pose.position.z,
    ]))
    print("Equilibrium joints:", fmt(equilibrium_joints))
    print("M/D/K:             ", args.mass, args.damping, args.stiffness)
    print("Max offset [m]:    ", args.max_offset)

    start_time = rospy.Time.now()
    last_time = start_time
    rate = rospy.Rate(args.rate)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()
        if elapsed > args.duration:
            break

        dt = (now - last_time).to_sec()
        last_time = now

        if args.use_topic_wrench:
            force = wrench_input.current_force()
        else:
            force = demo_force(args, elapsed)

        acceleration = admittance.step(force, dt)
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)
        target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)

        publish_joint_command(command_pub, target_joints, args.command_horizon)
        target_pub.publish(target_pose)
        state_pub.publish(String(data="force_N={} offset_m={} velocity_mps={} target_joints={}".format(
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt(target_joints),
        )))

        rospy.loginfo_throttle(
            0.5,
            "force[N]=%s offset[m]=%s velocity[m/s]=%s acceleration[m/s^2]=%s target_joints=%s",
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt3(acceleration),
            fmt(target_joints),
        )
        rate.sleep()

    # Keep streaming the final equilibrium-ish command briefly so the controller
    # has time to settle after the force is released.
    settle_until = rospy.Time.now() + rospy.Duration(1.0)
    while not rospy.is_shutdown() and rospy.Time.now() < settle_until:
        target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)
        publish_joint_command(command_pub, target_joints, args.command_horizon)
        rate.sleep()

    print("Experiment finished.")
    print("Final offset [m]:  ", fmt3(admittance.offset))
    print("Final velocity [m/s]:", fmt3(admittance.velocity))

    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
