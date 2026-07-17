#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import sys

import actionlib
import moveit_commander
import rosgraph
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


GROUP_NAME = "arm"
EEF_LINK = "link7"
REFERENCE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_PREPARE_JOINTS = [0.0, 0.25, 0.0, 0.35, 0.0, 0.20, 0.0]

WRENCH_TOPIC = "/rm75_admittance/target_wrench"
TARGET_POSE_TOPIC = "/rm75_admittance/target_pose"
STATE_TOPIC = "/rm75_admittance/state"
GAZEBO_JOINT_STATES_TOPIC = "/joint_states"
GAZEBO_TRAJECTORY_ACTION = "/arm/arm_joint_controller/follow_joint_trajectory"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def fmt3(values):
    return "[{:.4f}, {:.4f}, {:.4f}]".format(values[0], values[1], values[2])


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
    """Store the most recent virtual wrench command.

    The first experiment intentionally uses a virtual wrench topic instead of a
    Gazebo contact sensor. That keeps the control loop easy to debug: the only
    input is a commanded force vector in the MoveIt reference frame.
    """

    def __init__(self, expected_frame, timeout):
        self.expected_frame = expected_frame
        self.timeout = timeout
        self.force = [0.0, 0.0, 0.0]
        self.last_stamp = rospy.Time(0)
        self.warned_frame = False

        # ROS interface:
        #   topic: /rm75_admittance/target_wrench
        #   type:  geometry_msgs/WrenchStamped
        #   units: force is Newton in base_link by convention for this demo.
        # Torque is ignored in experiment 1 because orientation compliance is
        # intentionally disabled until translation compliance is understood.
        self.sub = rospy.Subscriber(WRENCH_TOPIC, WrenchStamped, self._callback, queue_size=1)

    def _callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.expected_frame and not self.warned_frame:
            rospy.logwarn(
                "Received wrench in frame '%s', but this demo assumes '%s'. "
                "No TF transform is applied in experiment 1.",
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
    """Three-axis Cartesian admittance model.

    The model is the virtual mass-damper-spring equation:

        M * x_ddot + D * x_dot + K * x = F_ext

    Here x is the Cartesian offset from the equilibrium pose, not the absolute
    robot pose. The output is a small pose correction that MoveIt can execute
    through the existing position-based Gazebo trajectory controller.
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
            # Semi-implicit Euler integration:
            #   acceleration comes from the admittance equation,
            #   velocity is limited for safety,
            #   offset is limited so a bad force command cannot run the arm away.
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


def execute_cartesian_path(group, target_pose, eef_link, eef_step, min_fraction):
    # Cartesian-path fallback:
    #   For tiny local compliance motions, asking OMPL for a full global plan can
    #   be slower and less reliable than interpolating a short straight TCP path.
    #   compute_cartesian_path still uses MoveIt's IK and collision checking, then
    #   execute() sends the resulting joint trajectory to the Gazebo controller.
    group.set_start_state_to_current_state()
    waypoints = [target_pose.pose]
    plan, fraction = group.compute_cartesian_path(
        waypoints,
        eef_step,
        True,  # avoid_collisions: keep MoveIt's collision checking enabled.
    )

    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo(
        "Cartesian path target xyz=[%.4f, %.4f, %.4f], fraction=%.3f, points=%d",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        fraction,
        point_count,
    )
    if fraction < min_fraction or point_count == 0:
        rospy.logwarn(
            "Cartesian path failed: fraction=%.3f, points=%d, min_fraction=%.3f",
            fraction,
            point_count,
            min_fraction,
        )
        return False

    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("Cartesian execute returned: %s", ok)
    return bool(ok)


def plan_and_execute_position(group, target_pose, eef_link):
    # Position-only target:
    #   The first RM75 Gazebo posture is close to a vertical singular pose, and
    #   keeping the exact wrist orientation can make IK fail for small Cartesian
    #   offsets. For this learning demo we only need to show translational
    #   admittance, so position-only planning is a better match.
    group.set_start_state_to_current_state()
    group.set_position_target(
        [
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ],
        eef_link,
    )
    success, plan, error_code = normalize_plan_result(group.plan())
    group.clear_pose_targets()

    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo(
        "Position plan target xyz=[%.4f, %.4f, %.4f], success=%s, points=%d, code=%s",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        success,
        point_count,
        error_name(error_code),
    )
    if not success or point_count == 0:
        return False

    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("Position execute returned: %s", ok)
    return bool(ok)


def plan_and_execute_pose(group, target_pose, eef_link, planner_mode, cartesian_eef_step, cartesian_min_fraction):
    # MoveIt interface:
    #   set_start_state_to_current_state() makes every small plan start from the
    #   real Gazebo joint state.
    #   set_pose_target() asks MoveIt for IK and a joint-space trajectory.
    #   execute() sends that trajectory to /arm/arm_joint_controller/follow_joint_trajectory.
    if planner_mode == "position":
        return plan_and_execute_position(group, target_pose, eef_link)

    if planner_mode == "cartesian":
        return execute_cartesian_path(
            group,
            target_pose,
            eef_link,
            cartesian_eef_step,
            cartesian_min_fraction,
        )

    group.set_start_state_to_current_state()
    group.set_pose_target(target_pose, eef_link)
    success, plan, error_code = normalize_plan_result(group.plan())
    group.clear_pose_targets()

    point_count = len(plan.joint_trajectory.points)
    if not success or point_count == 0:
        rospy.logwarn("MoveIt pose plan failed: %s, points=%d", error_name(error_code), point_count)
        if planner_mode == "auto":
            rospy.logwarn("Trying Cartesian fallback for this small local admittance step.")
            return execute_cartesian_path(
                group,
                target_pose,
                eef_link,
                cartesian_eef_step,
                cartesian_min_fraction,
            )
        return False

    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("MoveIt pose execute returned: %s", ok)
    return bool(ok)


def move_to_prepare_joints(group, prepare_joints):
    # Move away from the all-zero vertical posture before doing Cartesian
    # compliance. This gives IK a healthier seed and also makes motion visible.
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(JOINT_NAMES, prepare_joints)))
    success, plan, error_code = normalize_plan_result(group.plan())

    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo(
        "Prepare joint plan success=%s, points=%d, code=%s, target=%s",
        success,
        point_count,
        error_name(error_code),
        fmt3(prepare_joints[:3]) + " ...",
    )
    if not success or point_count == 0:
        rospy.logwarn("Prepare joint plan failed; continuing from current pose.")
        return False

    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("Prepare joint execute returned: %s", ok)
    return bool(ok)


def execute_joint_proxy(controller_client, equilibrium_joints, offset, duration):
    """Send a small joint-space surrogate motion directly to Gazebo ros_control.

    This backend is intentionally labeled as a proxy: it does not solve Cartesian
    IK. It maps the admittance x-offset onto a gentle bend/unbend pattern that
    visibly moves the simulated arm. Use it to verify ROS topics, timing, and the
    admittance response before moving on to a better IK/Jacobian controller.
    """
    target = list(equilibrium_joints)
    x = clamp(offset[0], -0.10, 0.10)
    y = clamp(offset[1], -0.05, 0.05)
    z = clamp(offset[2], -0.04, 0.04)

    # Empirical visualization mapping around DEFAULT_PREPARE_JOINTS.
    # Positive x bends the elbow/wrist in a way that makes the link7 motion
    # obvious in Gazebo. y and z get small secondary motions for future tests.
    target[0] += 1.0 * y
    target[1] += 1.2 * x - 0.8 * z
    target[3] += -2.8 * x + 1.6 * z
    target[5] += 1.6 * x - 0.8 * z

    point = JointTrajectoryPoint()
    point.positions = target
    point.velocities = [0.0] * len(JOINT_NAMES)
    point.time_from_start = rospy.Duration(duration)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINT_NAMES
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
    goal.trajectory.points = [point]

    rospy.loginfo("Joint-proxy target joints: %s", fmt(target))
    controller_client.send_goal(goal)
    finished = controller_client.wait_for_result(rospy.Duration(duration + 1.0))
    if not finished:
        rospy.logwarn("Joint-proxy trajectory did not finish before timeout.")
        controller_client.cancel_goal()
        return False

    result = controller_client.get_result()
    rospy.loginfo("Joint-proxy result error_code=%s", getattr(result, "error_code", "unknown"))
    return True


def run_execute_preflight():
    """Check the minimum ROS interfaces needed before trying to move Gazebo.

    This does not prove MoveIt can solve every pose target, but it separates
    "ROS/Gazebo is not wired up" from "the planner/IK could not solve this step".
    """
    try:
        joint_state = rospy.wait_for_message(GAZEBO_JOINT_STATES_TOPIC, JointState, timeout=3.0)
        rospy.loginfo("Preflight OK: received %d joints from %s", len(joint_state.name), GAZEBO_JOINT_STATES_TOPIC)
    except rospy.ROSException:
        rospy.logwarn(
            "Preflight warning: no %s message within 3s. Start Gazebo + ros_control first.",
            GAZEBO_JOINT_STATES_TOPIC,
        )

    client = actionlib.SimpleActionClient(GAZEBO_TRAJECTORY_ACTION, FollowJointTrajectoryAction)
    if client.wait_for_server(rospy.Duration(3.0)):
        rospy.loginfo("Preflight OK: trajectory action server is available: %s", GAZEBO_TRAJECTORY_ACTION)
    else:
        rospy.logwarn(
            "Preflight warning: action server not available: %s. "
            "Check arm_75_trajectory_controller.launch and controller_spawner.",
            GAZEBO_TRAJECTORY_ACTION,
        )
    return client


def demo_force(args, elapsed):
    if elapsed < args.demo_force_duration:
        return [args.demo_force_x, args.demo_force_y, args.demo_force_z]
    return [0.0, 0.0, 0.0]


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="RM75-6F simulation experiment 05: virtual-force Cartesian admittance."
    )
    parser.add_argument("--execute", action="store_true", help="Actually execute target poses through MoveIt.")
    parser.add_argument(
        "--visible-demo",
        action="store_true",
        help=(
            "Use a larger, slower-to-release x-axis demo motion and enable --execute. "
            "This is for Gazebo visualization only; keep the normal conservative defaults for first checks."
        ),
    )
    parser.add_argument("--duration", type=float, default=12.0, help="Total experiment duration in seconds.")
    parser.add_argument("--rate", type=float, default=20.0, help="Admittance integration rate in Hz.")
    parser.add_argument("--move-period", type=float, default=0.8, help="Seconds between MoveIt execution attempts.")
    parser.add_argument(
        "--planner-mode",
        choices=["joint-proxy", "position", "cartesian", "pose", "auto"],
        default="auto",
        help=(
            "Execution planner. joint-proxy sends a direct Gazebo joint trajectory for visualization; "
            "position ignores wrist orientation and is more robust "
            "for the first translational demo; cartesian follows a short straight TCP path; "
            "pose uses normal MoveIt planning; auto tries pose then Cartesian fallback."
        ),
    )
    parser.add_argument("--prepare", action="store_true", help="Move to a small bent joint posture before taking the equilibrium pose.")
    parser.add_argument(
        "--prepare-joints",
        type=float,
        nargs=7,
        default=list(DEFAULT_PREPARE_JOINTS),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="Joint target in radians used by --prepare.",
    )
    parser.add_argument("--cartesian-eef-step", type=float, default=0.01, help="Cartesian interpolation step in meters.")
    parser.add_argument("--cartesian-min-fraction", type=float, default=0.80, help="Minimum accepted Cartesian path fraction.")
    parser.add_argument("--mass", type=float, nargs=3, default=[2.0, 2.0, 2.0], metavar=("MX", "MY", "MZ"))
    parser.add_argument("--damping", type=float, nargs=3, default=[40.0, 40.0, 40.0], metavar=("DX", "DY", "DZ"))
    parser.add_argument("--stiffness", type=float, nargs=3, default=[80.0, 80.0, 80.0], metavar=("KX", "KY", "KZ"))
    parser.add_argument(
        "--max-offset",
        type=float,
        nargs=3,
        default=[0.05, 0.05, 0.04],
        metavar=("X", "Y", "Z"),
        help="Per-axis Cartesian offset limits in meters.",
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        nargs=3,
        default=[0.03, 0.03, 0.025],
        metavar=("VX", "VY", "VZ"),
        help="Per-axis Cartesian virtual velocity limits in m/s.",
    )
    parser.add_argument("--wrench-timeout", type=float, default=0.5, help="Seconds before topic wrench is treated as zero.")
    parser.add_argument("--demo-force-x", type=float, default=5.0, help="Built-in virtual force X in Newton.")
    parser.add_argument("--demo-force-y", type=float, default=0.0, help="Built-in virtual force Y in Newton.")
    parser.add_argument("--demo-force-z", type=float, default=0.0, help="Built-in virtual force Z in Newton.")
    parser.add_argument(
        "--demo-force-duration",
        type=float,
        default=4.0,
        help="Seconds to apply the built-in force before releasing it.",
    )
    parser.add_argument(
        "--use-topic-wrench",
        action="store_true",
        help="Use /rm75_admittance/target_wrench instead of the built-in demo force.",
    )
    parser.add_argument("--group", default=GROUP_NAME)
    parser.add_argument("--eef-link", default=EEF_LINK)
    parser.add_argument("--reference-frame", default=REFERENCE_FRAME)
    parser.add_argument("--velocity-scaling", type=float, default=0.08)
    parser.add_argument("--acceleration-scaling", type=float, default=0.08)
    return parser.parse_args(argv)


def apply_visible_demo_preset(args):
    """Make the first Gazebo execution visually obvious without changing safe defaults.

    The normal defaults are deliberately small because they are good for checking
    equations and topics. In Gazebo, however, 5 cm of wrist motion can be hard to
    see. This preset keeps the experiment one-dimensional in base_link +X, raises
    only the x-axis motion limit, and slows MoveIt execution enough that the user
    can see the compliant "push then return" behavior in RViz/Gazebo.
    """
    if not args.visible_demo:
        return

    args.execute = True
    args.duration = 16.0
    args.demo_force_x = 8.0
    args.demo_force_y = 0.0
    args.demo_force_z = 0.0
    args.demo_force_duration = 6.0
    args.stiffness = [45.0, 80.0, 80.0]
    args.damping = [35.0, 40.0, 40.0]
    args.max_offset = [0.10, 0.05, 0.04]
    args.max_velocity = [0.06, 0.03, 0.025]
    args.move_period = 0.4
    args.planner_mode = "joint-proxy"
    args.cartesian_eef_step = 0.01
    args.cartesian_min_fraction = 0.05
    args.prepare = True
    args.velocity_scaling = 0.10
    args.acceleration_scaling = 0.10


def main():
    args = parse_args(sys.argv[1:])
    apply_visible_demo_preset(args)

    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt first, for example:")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_05_admittance_virtual_force", anonymous=True)

    group = MoveGroupCommander(args.group)
    group.set_end_effector_link(args.eef_link)
    group.set_pose_reference_frame(args.reference_frame)
    group.set_planning_time(3.0)
    group.set_num_planning_attempts(5)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)

    rospy.sleep(1.0)

    controller_client = None
    if args.execute:
        controller_client = run_execute_preflight()
        if args.prepare:
            move_to_prepare_joints(group, args.prepare_joints)

    # The equilibrium pose is the "spring center" x0. With zero external force,
    # the virtual spring pulls the target pose back to this initial pose.
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

    # ROS outputs for learning/debugging:
    #   target_pose lets the user inspect the pose generated by admittance.
    #   state is a text summary of force, offset, and velocity. It is deliberately
    #   not a physical WrenchStamped message because the admittance state is a
    #   virtual controller state, not a sensor reading.
    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)

    print("RM75-6F admittance demo")
    print("Planning group:    ", args.group)
    print("End-effector link: ", args.eef_link)
    print("Reference frame:   ", args.reference_frame)
    print("Equilibrium xyz:   ", fmt3([
        equilibrium_pose.pose.position.x,
        equilibrium_pose.pose.position.y,
        equilibrium_pose.pose.position.z,
    ]))
    print("M/D/K:             ", args.mass, args.damping, args.stiffness)
    print("Max offset [m]:    ", args.max_offset)
    print("Max velocity [m/s]:", args.max_velocity)
    print("Execute mode:      ", args.execute)

    start_time = rospy.Time.now()
    last_time = start_time
    next_execute_time = start_time
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
        target_pub.publish(target_pose)

        state_pub.publish(String(data="force_N={} offset_m={} velocity_mps={}".format(
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
        )))

        rospy.loginfo_throttle(
            1.0,
            "force[N]=%s offset[m]=%s velocity[m/s]=%s acceleration[m/s^2]=%s",
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt3(acceleration),
        )

        if args.execute and now >= next_execute_time:
            if args.planner_mode == "joint-proxy":
                execute_joint_proxy(controller_client, equilibrium_joints, admittance.offset, args.move_period)
            else:
                plan_and_execute_pose(
                    group,
                    target_pose,
                    args.eef_link,
                    args.planner_mode,
                    args.cartesian_eef_step,
                    args.cartesian_min_fraction,
                )
            next_execute_time = rospy.Time.now() + rospy.Duration(args.move_period)

        rate.sleep()

    print("Experiment finished.")
    print("Final offset [m]:  ", fmt3(admittance.offset))
    print("Final velocity [m/s]:", fmt3(admittance.velocity))

    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
