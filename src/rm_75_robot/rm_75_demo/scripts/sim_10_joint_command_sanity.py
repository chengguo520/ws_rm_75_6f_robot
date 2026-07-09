#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F Gazebo sim_10：JointTrajectoryController 命令链路最小验证。

这个脚本是柔顺控制继续学习前的“地基检查”：

    /arm/arm_joint_controller/command
      -> Gazebo position JointTrajectoryController
      -> /arm/joint_states 或 /joint_states 反馈

它不计算导纳、不做 IK、不做 Jacobian，只给一个关节发送很小的正弦或阶跃命令。
如果这个最小实验都不能让 /joint_states 变化，后面的 sim_08/sim_09 再复杂也会
被控制器链路问题卡住。
"""

from __future__ import print_function

import argparse
import math
import sys
import threading

import rosgraph
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_COMMAND_TOPIC = "/arm/arm_joint_controller/command"
DEFAULT_JOINT_STATES_TOPIC = "/arm/joint_states"
DEFAULT_STATE_TOPIC = "/rm75_joint_command_sanity/state"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def normalize_angle(angle):
    """把角度折算到 [-pi, pi]。

    只用于诊断或用户显式要求的命令基准归一化。
    注意：归一化后的角度和原始大角度在数学上等价，但 position controller
    是否按等价角处理，取决于关节限制和控制器实现，所以默认不自动归一化。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_angle_delta(start, current):
    """计算 current 相对 start 的最短角度差，用来观察反馈是否真的动了。"""
    return normalize_angle(current - start)


class JointStateCache(object):
    """缓存指定 joint_states topic 的最新关节角。

    默认 topic 是 /arm/joint_states，因为这是 Gazebo arm controller namespace
    下的真实反馈；如果你想和 MoveIt 聚合后的 /joint_states 对比，可以用参数切换。
    """

    def __init__(self, topic):
        self.topic = topic
        self._lock = threading.Lock()
        self._positions_by_name = {}
        self.sub = rospy.Subscriber(topic, JointState, self._callback, queue_size=1)

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


def wait_for_joint_states(cache, timeout):
    """等待反馈 topic 出现完整 7 关节数据。"""
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        joints = cache.current_ordered()
        if joints is not None:
            rospy.loginfo("Initial measured joints from %s: %s", cache.topic, fmt(joints))
            return joints
        rate.sleep()
    raise RuntimeError("No complete joint state received from {}".format(cache.topic))


def wait_for_command_connection(pub, topic, timeout):
    """等待 JointTrajectoryController 订阅 /command。"""
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if pub.get_num_connections() > 0:
            rospy.loginfo("Command topic connected: %s", topic)
            return
        rate.sleep()
    raise RuntimeError("No subscriber connected to {}".format(topic))


def make_trajectory(joints, command_topic_joints, horizon, stamp_mode, lead_time):
    """构造单点 JointTrajectory。

    当前实验持续刷新短 horizon 目标点，和 sim_06/sim_08 的命令形式保持一致。
    stamp_mode 用来复现和排查“时间戳过期/未来时间戳/零时间戳”的差异。
    """
    msg = JointTrajectory()
    if stamp_mode == "zero":
        msg.header.stamp = rospy.Time(0)
    elif stamp_mode == "future":
        msg.header.stamp = rospy.Time.now() + rospy.Duration(lead_time)
    else:
        msg.header.stamp = rospy.Time.now()

    msg.joint_names = command_topic_joints

    point = JointTrajectoryPoint()
    point.positions = joints
    point.velocities = [0.0] * len(command_topic_joints)
    point.time_from_start = rospy.Duration(horizon)
    msg.points = [point]
    return msg


def command_delta(mode, amplitude, elapsed, duration):
    """生成一个很小的关节扰动。

    sine：完整一圈正弦，结束自然回到 0。
    step：前半段 +amplitude，后半段 -amplitude，更容易看阶跃跟随。
    """
    if mode == "sine":
        return amplitude * math.sin(2.0 * math.pi * elapsed / max(duration, 0.001))
    if elapsed < duration * 0.5:
        return amplitude
    return -amplitude


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Minimal RM75-6F Gazebo /command -> joint_states sanity test."
    )
    parser.add_argument("--joint", default="joint4", choices=JOINT_NAMES, help="要测试的小幅运动关节")
    parser.add_argument("--mode", default="sine", choices=["sine", "step"], help="命令形状")
    parser.add_argument("--amplitude", type=float, default=0.08, help="关节扰动幅度，单位 rad")
    parser.add_argument("--duration", type=float, default=6.0, help="主测试时长，单位 s")
    parser.add_argument("--hold-final", type=float, default=1.5, help="结束后回到基准位并保持的时长，单位 s")
    parser.add_argument("--rate", type=float, default=20.0, help="命令发布频率，单位 Hz")
    parser.add_argument("--horizon", type=float, default=0.25, help="单点轨迹到达时间，单位 s")
    parser.add_argument("--lead-time", type=float, default=0.03, help="future stamp 模式下的未来时间偏置，单位 s")
    parser.add_argument("--stamp-mode", default="future", choices=["future", "now", "zero"], help="轨迹 header 时间戳模式")
    parser.add_argument("--min-motion", type=float, default=0.02, help="判定反馈真实运动的最小角度，单位 rad")
    parser.add_argument("--timeout", type=float, default=8.0, help="等待 ROS topic 连接的超时，单位 s")
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC, help="JointTrajectoryController command topic")
    parser.add_argument("--joint-states-topic", default=DEFAULT_JOINT_STATES_TOPIC, help="用于判断真实运动的 joint_states topic")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help="发布简短诊断字符串的 topic")
    parser.add_argument(
        "--normalize-command-base",
        action="store_true",
        help="把初始关节角归一化到 [-pi, pi] 后再作为命令基准；只在排查大角度包装时使用",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if not rosgraph.is_master_online():
        raise RuntimeError("ROS master is not running. Start Gazebo/MoveIt launch first.")

    rospy.init_node("rm75_joint_command_sanity", anonymous=True)

    joint_index = JOINT_NAMES.index(args.joint)
    cache = JointStateCache(args.joint_states_topic)
    state_pub = rospy.Publisher(args.state_topic, String, queue_size=10)
    command_pub = rospy.Publisher(args.command_topic, JointTrajectory, queue_size=10)

    measured_start = wait_for_joint_states(cache, args.timeout)
    wait_for_command_connection(command_pub, args.command_topic, args.timeout)

    if args.normalize_command_base:
        command_base = [normalize_angle(v) for v in measured_start]
        rospy.logwarn("Using normalized command base: %s", fmt(command_base))
    else:
        command_base = list(measured_start)

    wrapped = [name for name, value in zip(JOINT_NAMES, measured_start) if abs(value) > math.pi]
    if wrapped and not args.normalize_command_base:
        rospy.logwarn(
            "Large raw joint angles detected on %s: %s. "
            "First test raw mode; if it does not move, rerun with --normalize-command-base.",
            args.joint_states_topic,
            ", ".join(wrapped),
        )

    rospy.loginfo(
        "Testing %s with mode=%s amplitude=%.4f rad duration=%.2fs horizon=%.2fs stamp=%s",
        args.joint,
        args.mode,
        args.amplitude,
        args.duration,
        args.horizon,
        args.stamp_mode,
    )

    rate = rospy.Rate(args.rate)
    start_time = rospy.Time.now()
    measured_deltas = []

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > args.duration:
            break

        target = list(command_base)
        delta = command_delta(args.mode, args.amplitude, elapsed, args.duration)
        target[joint_index] = command_base[joint_index] + delta

        command_pub.publish(
            make_trajectory(target, JOINT_NAMES, args.horizon, args.stamp_mode, args.lead_time)
        )

        measured = cache.current_ordered()
        measured_value = float("nan")
        measured_delta = 0.0
        if measured is not None:
            measured_value = measured[joint_index]
            measured_delta = shortest_angle_delta(measured_start[joint_index], measured_value)
            measured_deltas.append(measured_delta)

        text = (
            "t={:.2f}s joint={} command={:.4f} measured={:.4f} "
            "measured_delta={:.4f}"
        ).format(elapsed, args.joint, target[joint_index], measured_value, measured_delta)
        state_pub.publish(text)
        rospy.loginfo_throttle(0.5, text)
        rate.sleep()

    hold_start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - hold_start).to_sec() < args.hold_final:
        command_pub.publish(
            make_trajectory(command_base, JOINT_NAMES, args.horizon, args.stamp_mode, args.lead_time)
        )
        rate.sleep()

    observed_peak = max([abs(v) for v in measured_deltas]) if measured_deltas else 0.0
    observed_span = (max(measured_deltas) - min(measured_deltas)) if measured_deltas else 0.0
    summary = (
        "RESULT joint={} observed_peak={:.4f}rad observed_span={:.4f}rad min_motion={:.4f}rad"
    ).format(args.joint, observed_peak, observed_span, args.min_motion)

    if observed_peak >= args.min_motion or observed_span >= args.min_motion:
        rospy.loginfo("PASS: %s", summary)
    else:
        rospy.logwarn("WARN: %s. Feedback did not show enough motion.", summary)
    state_pub.publish(summary)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("sim_10_joint_command_sanity failed: {}".format(exc), file=sys.stderr)
        sys.exit(1)
