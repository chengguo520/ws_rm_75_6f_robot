#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F Gazebo 仿真柔顺控制 sim_07：IK 位置映射版。

sim_06 使用 joint-proxy 经验映射：
    offset -> target_joints

sim_07 的目标是往“真正末端柔顺”迈一步：
    offset -> target_pose -> /compute_ik -> target_joints -> /arm/arm_joint_controller/command

注意：
    1. 这里调用的是 MoveIt /compute_ik 服务，不做完整规划。
    2. IK 位置映射仍不等于高频笛卡尔伺服，默认频率较低。
    3. 这个脚本适合验证“导纳输出的末端目标位姿是否可解、是否能执行”。
    4. 如果当前 MoveIt/KDL IK 对 RM75 模型求解失败，脚本默认启用 joint-proxy fallback，
       让你仍能看到导纳运动，同时在 state 里明确标出 backend=joint_proxy_fallback。
"""

from __future__ import print_function

import argparse
import sys
import threading

import moveit_commander
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


GROUP_NAME = "arm"
EEF_LINK = "link7"
REFERENCE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_PREPARE_JOINTS = [0.0, 0.25, 0.0, 0.35, 0.0, 0.20, 0.0]

COMMAND_TOPIC = "/arm/arm_joint_controller/command"
JOINT_STATES_TOPIC = "/joint_states"
WRENCH_TOPIC = "/rm75_admittance_ik/target_wrench"
TARGET_POSE_TOPIC = "/rm75_admittance_ik/target_pose"
STATE_TOPIC = "/rm75_admittance_ik/state"
COMPUTE_IK_SERVICE = "/compute_ik"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt3(values):
    return "[{:.4f}, {:.4f}, {:.4f}]".format(values[0], values[1], values[2])


def max_abs(values):
    if not values:
        return 0.0
    return max(abs(v) for v in values)


def error_name(error_code):
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


def copy_pose_stamped(source):
    copied = PoseStamped()
    copied.header.frame_id = source.header.frame_id
    copied.header.stamp = source.header.stamp
    copied.pose.position.x = source.pose.position.x
    copied.pose.position.y = source.pose.position.y
    copied.pose.position.z = source.pose.position.z
    copied.pose.orientation = source.pose.orientation
    return copied


class JointStateCache(object):
    """缓存 /joint_states。

    sim_07 使用它做两件事：
        1. 给 IK 服务提供 seed state；
        2. 诊断 target_joints 和 measured_joints 的误差。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._positions_by_name = {}
        self.sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._callback, queue_size=1)

    def _callback(self, msg):
        with self._lock:
            self._positions_by_name = dict(zip(msg.name, msg.position))

    def current_ordered(self):
        with self._lock:
            if not self._positions_by_name:
                return None
            missing = [name for name in JOINT_NAMES if name not in self._positions_by_name]
            if missing:
                return None
            return [self._positions_by_name[name] for name in JOINT_NAMES]


class WrenchInput(object):
    """可选外部虚拟力输入。

    Topic:
        /rm75_admittance_ik/target_wrench

    Type:
        geometry_msgs/WrenchStamped

    约定：
        force.x/y/z 单位 N，默认在 base_link 下表达。
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
                "Received wrench in frame '%s', but sim_07 assumes '%s'. No TF transform is applied.",
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
    """三轴平移导纳：M*x_ddot + D*x_dot + K*x = F_ext。"""

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
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(JOINT_NAMES, prepare_joints)))
    success, plan, error_code = normalize_plan_result(group.plan())
    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo("Prepare plan success=%s, points=%d", success, point_count)
    if not success or point_count == 0:
        label = error_name(error_code) if error_code is not None else "unknown"
        raise RuntimeError("Prepare joint plan failed: {}".format(label))
    ok = group.execute(plan, wait=True)
    group.stop()
    if not ok:
        raise RuntimeError("Prepare joint execution failed")


def wait_for_joint_state(timeout):
    msg = rospy.wait_for_message(JOINT_STATES_TOPIC, JointState, timeout=timeout)
    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("/joint_states missing joints: {}".format(", ".join(missing)))
    return msg


def wait_for_command_connection(pub, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if pub.get_num_connections() > 0:
            rospy.loginfo("Command topic connected: %s subscribers=%d", COMMAND_TOPIC, pub.get_num_connections())
            return True
        rospy.sleep(0.05)
    rospy.logwarn("No subscriber connected to %s within %.1fs", COMMAND_TOPIC, timeout)
    return False


def smooth_joint_target(previous_target, raw_target, alpha):
    if previous_target is None:
        return list(raw_target)
    alpha = clamp(alpha, 0.0, 1.0)
    return [
        previous_target[i] + alpha * (raw_target[i] - previous_target[i])
        for i in range(len(raw_target))
    ]


def limit_joint_step(previous_target, target, max_step):
    if previous_target is None or max_step <= 0.0:
        return list(target)
    limited = []
    for i in range(len(target)):
        delta = target[i] - previous_target[i]
        limited.append(previous_target[i] + clamp(delta, -max_step, max_step))
    return limited


def publish_joint_command(pub, target_joints, horizon):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = target_joints
    point.velocities = [0.0] * len(JOINT_NAMES)
    point.time_from_start = rospy.Duration(horizon)
    msg.points = [point]
    pub.publish(msg)


def ordered_solution_joints(solution):
    positions_by_name = dict(zip(solution.joint_state.name, solution.joint_state.position))
    missing = [name for name in JOINT_NAMES if name not in positions_by_name]
    if missing:
        raise RuntimeError("IK solution missing joints: {}".format(", ".join(missing)))
    return [positions_by_name[name] for name in JOINT_NAMES]


def solve_ik(ik_client, args, target_pose, seed_joints):
    """调用 MoveIt /compute_ik，把 target_pose 转换成关节目标。

    seed_joints 会影响 7 自由度机械臂选哪个冗余解。这里优先使用当前实测关节角，
    这样 IK 解通常更接近当前姿态，控制命令也更连续。
    """
    request = GetPositionIKRequest()
    request.ik_request.group_name = args.group
    request.ik_request.ik_link_name = args.eef_link
    request.ik_request.pose_stamped = target_pose
    request.ik_request.timeout = rospy.Duration(args.ik_timeout)
    request.ik_request.avoid_collisions = args.avoid_collisions

    robot_state = RobotState()
    robot_state.is_diff = True
    robot_state.joint_state.header.stamp = rospy.Time.now()
    robot_state.joint_state.name = list(JOINT_NAMES)
    robot_state.joint_state.position = list(seed_joints)
    request.ik_request.robot_state = robot_state

    response = ik_client(request)
    success = response.error_code.val == MoveItErrorCodes.SUCCESS
    if not success:
        return False, None, error_name(response.error_code)
    return True, ordered_solution_joints(response.solution), error_name(response.error_code)


def map_offset_to_joint_target(equilibrium_joints, offset):
    """IK 失败时的可视化 fallback。

    这个映射和 sim_06 的 joint-proxy 思路一致：
        offset -> 几个关节的小范围变化。

    重要：
        这不是 IK，也不保证 link7 精确到 target_pose。
        它只用于在 /compute_ik 返回 NO_IK_SOLUTION 时，继续观察导纳模型的运动趋势。
    """
    target = list(equilibrium_joints)
    x = clamp(offset[0], -0.06, 0.06)
    y = clamp(offset[1], -0.03, 0.03)
    z = clamp(offset[2], -0.03, 0.03)
    target[0] += 1.0 * y
    target[1] += 1.0 * x - 0.6 * z
    target[3] += -2.2 * x + 1.2 * z
    target[5] += 1.2 * x - 0.6 * z
    return target


def parse_args(argv):
    parser = argparse.ArgumentParser(description="RM75-6F sim_07: admittance with MoveIt IK position mapping.")
    parser.add_argument("--duration", type=float, default=16.0)
    parser.add_argument("--rate", type=float, default=10.0, help="IK loop rate. Keep low; /compute_ik is not a servo controller.")
    parser.add_argument("--command-horizon", type=float, default=0.20)
    parser.add_argument("--prepare", action="store_true", default=True)
    parser.add_argument("--no-prepare", dest="prepare", action="store_false")
    parser.add_argument("--prepare-joints", type=float, nargs=7, default=list(DEFAULT_PREPARE_JOINTS))
    parser.add_argument("--mass", type=float, nargs=3, default=[2.0, 2.0, 2.0])
    parser.add_argument("--damping", type=float, nargs=3, default=[35.0, 40.0, 40.0])
    parser.add_argument("--stiffness", type=float, nargs=3, default=[45.0, 80.0, 80.0])
    parser.add_argument("--max-offset", type=float, nargs=3, default=[0.06, 0.03, 0.03])
    parser.add_argument("--max-velocity", type=float, nargs=3, default=[0.025, 0.015, 0.015])
    parser.add_argument("--joint-smoothing-alpha", type=float, default=0.35)
    parser.add_argument("--max-joint-step", type=float, default=0.015)
    parser.add_argument("--settle-duration", type=float, default=2.0)
    parser.add_argument("--ik-timeout", type=float, default=0.10)
    parser.add_argument("--avoid-collisions", action="store_true", help="Ask /compute_ik to collision-check IK solutions.")
    parser.add_argument("--allow-joint-proxy-fallback", action="store_true", default=True)
    parser.add_argument("--no-joint-proxy-fallback", dest="allow_joint_proxy_fallback", action="store_false")
    parser.add_argument("--wrench-timeout", type=float, default=0.5)
    parser.add_argument("--demo-force-x", type=float, default=5.0)
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


def run_control_phase(args, ik_client, command_pub, target_pub, state_pub, joint_cache,
                      admittance, equilibrium_pose, equilibrium_joints, filtered_target_joints, start_time,
                      last_time, duration, use_demo_force):
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()
        if elapsed > duration:
            break

        dt = (now - last_time).to_sec()
        last_time = now

        force = demo_force(args, elapsed) if use_demo_force else [0.0, 0.0, 0.0]
        admittance.step(force, dt)
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)

        measured_joints = joint_cache.current_ordered()
        seed_joints = measured_joints if measured_joints is not None else filtered_target_joints
        ik_success, raw_target_joints, ik_status = solve_ik(ik_client, args, target_pose, seed_joints)

        if ik_success:
            backend = "ik"
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        elif args.allow_joint_proxy_fallback:
            backend = "joint_proxy_fallback"
            raw_target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        else:
            backend = "hold"
            raw_target_joints = list(filtered_target_joints)
            target_joints = list(filtered_target_joints)

        joint_error = None
        if measured_joints is not None:
            joint_error = [target_joints[i] - measured_joints[i] for i in range(len(target_joints))]

        target_pub.publish(target_pose)
        state_pub.publish(String(data=(
            "backend={} ik_success={} ik_status={} force_N={} offset_m={} target_xyz={} "
            "target_joints={} joint_error_max={:.4f}"
        ).format(
            backend,
            ik_success,
            ik_status,
            fmt3(force),
            fmt3(admittance.offset),
            fmt3([
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ]),
            fmt(target_joints),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )))
        rospy.loginfo_throttle(
            0.5,
            "backend=%s IK %s offset=%s joint_error_max=%.4f",
            backend,
            ik_status,
            fmt3(admittance.offset),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )
        rate.sleep()

    return filtered_target_joints, last_time


def main():
    args = parse_args(sys.argv[1:])
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running. Start Gazebo + MoveIt first.")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_07_admittance_ik_position", anonymous=True)

    wait_for_joint_state(timeout=3.0)
    joint_cache = JointStateCache()

    command_pub = rospy.Publisher(COMMAND_TOPIC, JointTrajectory, queue_size=1)
    wait_for_command_connection(command_pub, timeout=3.0)

    rospy.loginfo("Waiting for %s service", COMPUTE_IK_SERVICE)
    rospy.wait_for_service(COMPUTE_IK_SERVICE, timeout=5.0)
    ik_client = rospy.ServiceProxy(COMPUTE_IK_SERVICE, GetPositionIK)

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

    # 启动时先做一次 offset=0 的 IK 自检。
    # 如果这里都失败，说明当前 MoveIt/KDL 对这个模型的 IK 求解链路不可靠，
    # 后续看到 NO_IK_SOLUTION 就不是“位移太小”，而是 IK 没解出来。
    initial_seed = joint_cache.current_ordered() or equilibrium_joints
    eq_ik_success, _, eq_ik_status = solve_ik(ik_client, args, equilibrium_pose, initial_seed)
    if eq_ik_success:
        rospy.loginfo("Initial equilibrium IK check passed.")
    else:
        rospy.logwarn(
            "Initial equilibrium IK check failed: %s. "
            "sim_07 will use joint_proxy_fallback=%s when IK fails.",
            eq_ik_status,
            args.allow_joint_proxy_fallback,
        )

    wrench_input = WrenchInput(args.reference_frame, args.wrench_timeout)
    admittance = TranslationalAdmittance(args.mass, args.damping, args.stiffness, args.max_offset, args.max_velocity)
    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)

    print("RM75-6F sim_07 admittance IK position demo")
    print("IK service:       ", COMPUTE_IK_SERVICE)
    print("Command topic:    ", COMMAND_TOPIC)
    print("Rate/horizon:     ", args.rate, args.command_horizon)
    print("Max offset [m]:   ", args.max_offset)
    print("IK timeout [s]:   ", args.ik_timeout)
    print("IK fallback:      ", args.allow_joint_proxy_fallback)

    filtered_target_joints = list(equilibrium_joints)
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
        force = wrench_input.current_force() if args.use_topic_wrench else demo_force(args, elapsed)
        admittance.step(force, dt)
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)

        measured_joints = joint_cache.current_ordered()
        seed_joints = measured_joints if measured_joints is not None else filtered_target_joints
        ik_success, raw_target_joints, ik_status = solve_ik(ik_client, args, target_pose, seed_joints)

        if ik_success:
            backend = "ik"
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        elif args.allow_joint_proxy_fallback:
            backend = "joint_proxy_fallback"
            raw_target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        else:
            backend = "hold"
            raw_target_joints = list(filtered_target_joints)
            target_joints = list(filtered_target_joints)

        joint_error = None
        if measured_joints is not None:
            joint_error = [target_joints[i] - measured_joints[i] for i in range(len(target_joints))]

        target_pub.publish(target_pose)
        state_pub.publish(String(data=(
            "backend={} ik_success={} ik_status={} force_N={} offset_m={} target_xyz={} "
            "raw_target_joints={} target_joints={} joint_error_max={:.4f}"
        ).format(
            backend,
            ik_success,
            ik_status,
            fmt3(force),
            fmt3(admittance.offset),
            fmt3([target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z]),
            fmt(raw_target_joints),
            fmt(target_joints),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )))

        rospy.loginfo_throttle(
            0.5,
            "backend=%s IK %s force=%s offset=%s joint_error_max=%.4f",
            backend,
            ik_status,
            fmt3(force),
            fmt3(admittance.offset),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )
        rate.sleep()

    settle_until = rospy.Time.now() + rospy.Duration(max(0.0, args.settle_duration))
    while not rospy.is_shutdown() and rospy.Time.now() < settle_until:
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now
        admittance.step([0.0, 0.0, 0.0], dt)
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)
        measured_joints = joint_cache.current_ordered()
        seed_joints = measured_joints if measured_joints is not None else filtered_target_joints
        ik_success, raw_target_joints, _ = solve_ik(ik_client, args, target_pose, seed_joints)
        if ik_success:
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        elif args.allow_joint_proxy_fallback:
            raw_target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)
            smoothed = smooth_joint_target(filtered_target_joints, raw_target_joints, args.joint_smoothing_alpha)
            target_joints = limit_joint_step(filtered_target_joints, smoothed, args.max_joint_step)
            filtered_target_joints = list(target_joints)
            publish_joint_command(command_pub, target_joints, args.command_horizon)
        rate.sleep()

    print("Experiment finished.")
    print("Final offset [m]:   ", fmt3(admittance.offset))
    print("Final velocity [m/s]:", fmt3(admittance.velocity))
    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
