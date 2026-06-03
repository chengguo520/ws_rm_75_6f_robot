#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F Gazebo 仿真柔顺控制 sim_08：Jacobian 伪逆速度映射版。

sim_08 是当前柔顺控制学习主线里更重要的一步：

    外力 F
      -> 导纳模型 M/D/K
      -> 笛卡尔速度 v_cartesian
      -> Jacobian 阻尼伪逆 J^#
      -> 关节速度 q_dot
      -> 积分成关节目标 q_des
      -> /arm/arm_joint_controller/command

和 sim_07 相比：
    sim_07：target_pose -> IK -> target_joints，偏位置映射验证；
    sim_08：cartesian velocity -> Jacobian -> q_dot，更接近连续伺服控制。

注意：
    当前 Gazebo 控制器仍然是 position JointTrajectoryController，所以本脚本最终仍
    发布短 horizon 的 JointTrajectory 位置点，不是直接发速度控制器。
"""

from __future__ import print_function

import argparse
import math
import sys
import threading

import moveit_commander
import numpy as np
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

COMMAND_TOPIC = "/arm/arm_joint_controller/command"
JOINT_STATES_TOPIC = "/joint_states"
WRENCH_TOPIC = "/rm75_admittance_jacobian/target_wrench"
TARGET_POSE_TOPIC = "/rm75_admittance_jacobian/target_pose"
STATE_TOPIC = "/rm75_admittance_jacobian/state"


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
    """缓存最新 /joint_states。

    sim_08 用 measured_joints 作为 Jacobian 计算点 q_current。
    这比一直围绕 equilibrium_joints 积分更像反馈控制。
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
    """可选外部 wrench 输入，单位 N，默认在 base_link 下表达。"""

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
                "Received wrench in frame '%s', but sim_08 assumes '%s'. No TF transform is applied.",
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
    """三轴平移导纳。

    输出里最关键的是 velocity：
        velocity[m/s] -> Jacobian 伪逆 -> q_dot[rad/s]
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
    group.set_start_state_to_current_state()
    group.set_joint_value_target(dict(zip(JOINT_NAMES, prepare_joints)))
    success, plan, error_code = normalize_plan_result(group.plan())
    point_count = len(plan.joint_trajectory.points)
    rospy.loginfo("Prepare plan success=%s, points=%d, code=%s", success, point_count, error_name(error_code))
    if not success or point_count == 0:
        raise RuntimeError("Prepare joint plan failed: {}".format(error_name(error_code)))
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


def limit_vector(values, max_abs_value):
    if max_abs_value <= 0.0:
        return list(values)
    return [clamp(v, -max_abs_value, max_abs_value) for v in values]


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


def damped_pseudo_inverse_qdot(group, current_joints, cartesian_velocity, damping, max_joint_velocity):
    """用阻尼最小二乘 DLS 计算 q_dot。

    MoveIt 返回 6x7 Jacobian：
        [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]

    当前实验只做平移导纳，但为了姿态不要乱漂，把期望角速度设为 0，
    使用完整 6D 速度 [vx, vy, vz, 0, 0, 0]。

    DLS 公式：
        q_dot = J^T * (J*J^T + lambda^2 I)^-1 * v
    """
    jacobian = np.array(group.get_jacobian_matrix(current_joints), dtype=float)
    if jacobian.shape[0] < 6:
        raise RuntimeError("Unexpected Jacobian shape: {}".format(jacobian.shape))

    v6 = np.array([
        cartesian_velocity[0],
        cartesian_velocity[1],
        cartesian_velocity[2],
        0.0,
        0.0,
        0.0,
    ], dtype=float)

    lambda2 = damping * damping
    identity = np.eye(jacobian.shape[0])
    qdot = jacobian.T.dot(np.linalg.solve(jacobian.dot(jacobian.T) + lambda2 * identity, v6))
    qdot = limit_vector(qdot.tolist(), max_joint_velocity)

    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    min_singular = float(np.min(singular_values)) if singular_values.size else 0.0
    return qdot, min_singular


def parse_args(argv):
    parser = argparse.ArgumentParser(description="RM75-6F sim_08: admittance with Jacobian velocity mapping.")
    parser.add_argument("--duration", type=float, default=16.0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--command-horizon", type=float, default=0.12)
    parser.add_argument("--prepare", action="store_true", default=True)
    parser.add_argument("--no-prepare", dest="prepare", action="store_false")
    parser.add_argument("--prepare-joints", type=float, nargs=7, default=list(DEFAULT_PREPARE_JOINTS))
    parser.add_argument("--mass", type=float, nargs=3, default=[2.0, 2.0, 2.0])
    parser.add_argument("--damping", type=float, nargs=3, default=[35.0, 40.0, 40.0])
    parser.add_argument("--stiffness", type=float, nargs=3, default=[45.0, 80.0, 80.0])
    parser.add_argument("--max-offset", type=float, nargs=3, default=[0.06, 0.03, 0.03])
    parser.add_argument("--max-velocity", type=float, nargs=3, default=[0.025, 0.015, 0.015])
    parser.add_argument("--jacobian-damping", type=float, default=0.05, help="DLS damping lambda. Larger is safer but slower.")
    parser.add_argument("--max-joint-velocity", type=float, default=0.25, help="q_dot clamp in rad/s.")
    parser.add_argument("--max-joint-step", type=float, default=0.015, help="q_des step clamp in rad/control-cycle.")
    parser.add_argument("--settle-duration", type=float, default=2.0)
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


def main():
    args = parse_args(sys.argv[1:])
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running. Start Gazebo + MoveIt first.")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_08_admittance_jacobian_velocity", anonymous=True)

    wait_for_joint_state(timeout=3.0)
    joint_cache = JointStateCache()

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

    wrench_input = WrenchInput(args.reference_frame, args.wrench_timeout)
    admittance = TranslationalAdmittance(args.mass, args.damping, args.stiffness, args.max_offset, args.max_velocity)
    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)

    measured = joint_cache.current_ordered()
    last_commanded_joints = measured if measured is not None else group.get_current_joint_values()

    print("RM75-6F sim_08 admittance Jacobian velocity demo")
    print("Command topic:       ", COMMAND_TOPIC)
    print("Rate/horizon:        ", args.rate, args.command_horizon)
    print("Jacobian damping:    ", args.jacobian_damping)
    print("Max joint velocity:  ", args.max_joint_velocity)
    print("Max joint step:      ", args.max_joint_step)

    start_time = rospy.Time.now()
    last_time = start_time
    rate = rospy.Rate(args.rate)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()
        if elapsed > args.duration:
            break

        dt = clamp((now - last_time).to_sec(), 0.001, 0.2)
        last_time = now
        force = wrench_input.current_force() if args.use_topic_wrench else demo_force(args, elapsed)

        acceleration = admittance.step(force, dt)
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)
        measured_joints = joint_cache.current_ordered()
        current_joints = measured_joints if measured_joints is not None else last_commanded_joints

        qdot, min_singular = damped_pseudo_inverse_qdot(
            group,
            current_joints,
            admittance.velocity,
            args.jacobian_damping,
            args.max_joint_velocity,
        )

        raw_target_joints = [
            current_joints[i] + qdot[i] * dt
            for i in range(len(current_joints))
        ]
        target_joints = limit_joint_step(last_commanded_joints, raw_target_joints, args.max_joint_step)
        last_commanded_joints = list(target_joints)
        publish_joint_command(command_pub, target_joints, args.command_horizon)

        joint_error = None
        if measured_joints is not None:
            joint_error = [target_joints[i] - measured_joints[i] for i in range(len(target_joints))]

        target_pub.publish(target_pose)
        state_pub.publish(String(data=(
            "force_N={} offset_m={} velocity_mps={} acceleration={} qdot={} "
            "min_singular={:.5f} target_joints={} joint_error_max={:.4f}"
        ).format(
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt3(acceleration),
            fmt(qdot),
            min_singular,
            fmt(target_joints),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )))

        rospy.loginfo_throttle(
            0.5,
            "force=%s offset=%s qdot_max=%.4f min_singular=%.5f joint_error_max=%.4f",
            fmt3(force),
            fmt3(admittance.offset),
            max_abs(qdot),
            min_singular,
            max_abs(joint_error) if joint_error is not None else -1.0,
        )
        if min_singular < 0.02:
            rospy.logwarn_throttle(1.0, "Jacobian is near singular: min_singular=%.5f", min_singular)

        rate.sleep()

    settle_until = rospy.Time.now() + rospy.Duration(max(0.0, args.settle_duration))
    while not rospy.is_shutdown() and rospy.Time.now() < settle_until:
        now = rospy.Time.now()
        dt = clamp((now - last_time).to_sec(), 0.001, 0.2)
        last_time = now
        admittance.step([0.0, 0.0, 0.0], dt)
        measured_joints = joint_cache.current_ordered()
        current_joints = measured_joints if measured_joints is not None else last_commanded_joints
        qdot, _ = damped_pseudo_inverse_qdot(
            group,
            current_joints,
            admittance.velocity,
            args.jacobian_damping,
            args.max_joint_velocity,
        )
        raw_target_joints = [current_joints[i] + qdot[i] * dt for i in range(len(current_joints))]
        target_joints = limit_joint_step(last_commanded_joints, raw_target_joints, args.max_joint_step)
        last_commanded_joints = list(target_joints)
        publish_joint_command(command_pub, target_joints, args.command_horizon)
        rate.sleep()

    print("Experiment finished.")
    print("Final offset [m]:   ", fmt3(admittance.offset))
    print("Final velocity [m/s]:", fmt3(admittance.velocity))
    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

