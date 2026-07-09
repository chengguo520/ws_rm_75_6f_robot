#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F Gazebo sim_09：慢速桌面擦拭 + z 单轴导纳验证。

这版先做“可看懂、可回退”的擦桌子闭环：

    /arm/joint_states
      -> TF(base_link -> link7)
      -> 虚拟 TCP/contact_z
      -> 虚拟桌面法向力
      -> z 单轴导纳得到 vz
      -> Jacobian DLS 得到 q_dot
      -> q_last_commanded + q_dot * dt
      -> /arm/arm_joint_controller/command

为什么先做这个实验：
    sim_08 已经证明 Jacobian 速度导纳能让机械臂真实运动。
    sim_09 默认先进入最小 x_line 慢速桌面擦拭：固定一条 x 方向直线，
    y 固定，z 方向根据虚拟桌面高度做单轴导纳反馈。x_bump 单凸起和
    x_wave 平滑起伏曲面模式用于观察更明显的柔顺退让。

Gazebo 中可以用 arm_75_bumpy_wiping_moveit.launch 载入可见桌子和凸起；
当前反馈仍来自数学虚拟力场，而不是真实 Gazebo 接触力。后续接触传感器
验证通过后，可以把 virtual_contact_force() 换成 Gazebo contact/force sensor
或真实 RM75-6F 六维力传感器读数。
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
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


GROUP_NAME = "arm"
EEF_LINK = "link7"
REFERENCE_FRAME = "base_link"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
# 来自示教器保存点 cazhuozi1：作为当前擦桌子短直线的准备起点。
DEFAULT_PREPARE_JOINTS = [-0.000107, 0.185648, 0.109322, 1.316919, -0.076408, 0.927566, 0.599898]
DEFAULT_TABLE_CENTER_X = 0.410
DEFAULT_TABLE_CENTER_Y = 0.026
DEFAULT_TABLE_SURFACE_Z = 0.336
DEFAULT_VISUAL_TABLE_CENTER_X = 0.640
DEFAULT_VISUAL_TABLE_CENTER_Y = 0.026
DEFAULT_VISUAL_TABLE_SIZE_X = 0.760
DEFAULT_VISUAL_TABLE_SIZE_Y = 0.440

COMMAND_TOPIC = "/arm/arm_joint_controller/command"
DEFAULT_JOINT_STATES_TOPIC = "/arm/joint_states"
TARGET_POSE_TOPIC = "/rm75_z_admittance_probe/target_pose"
STATE_TOPIC = "/rm75_z_admittance_probe/state"
MARKER_TOPIC = "/rm75_z_admittance_probe/markers"


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def fmt(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt3(values):
    return "[{:.4f}, {:.4f}, {:.4f}]".format(values[0], values[1], values[2])


def quat_to_matrix(quat):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def link_z_axis_in_base(pose):
    """返回 link7 局部 Z 轴在 base_link 下的方向。

    当前用 link7/法兰作为虚拟擦拭端。如果这个向量的 z 分量为正，
    说明法兰面更像朝天；z 分量越接近 -1，越像朝向桌面。
    """
    return quat_to_matrix(pose.pose.orientation).dot(np.array([0.0, 0.0, 1.0])).tolist()


def max_abs(values):
    if not values:
        return 0.0
    return max(abs(v) for v in values)


def bounds_margin(inner_min, inner_max, outer_min, outer_max):
    return min(inner_min - outer_min, outer_max - inner_max)


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


class JointStateCache(object):
    """缓存 Gazebo arm controller 的真实关节反馈。

    sim_10 已验证 /arm/joint_states 会跟随 /arm/arm_joint_controller/command。
    sim_09 继续使用这个 topic 作为 Jacobian 计算点和 joint_error 诊断来源。
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


class TfPoseReader(object):
    """从 TF 读取 base_link -> link7。

    闭环位姿不用 MoveIt get_current_pose()，因为之前 sim_09 失败时它可能滞后。
    这里强制以 /arm/joint_states 经 robot_state_publisher 发布的 TF 为准。
    """

    def __init__(self):
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

    def current_pose(self, reference_frame, eef_link, timeout=0.05):
        transform = self.buffer.lookup_transform(
            reference_frame,
            eef_link,
            rospy.Time(0),
            rospy.Duration(timeout),
        )
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose


class ZAxisAdmittance(object):
    """只控制 z 方向的质量-阻尼导纳。

    符号约定：
      - contact_force 是虚拟桌面对 TCP 的向上法向力，单位 N。
      - desired_force 是希望保持的向上法向力，单位 N。
      - contact_force > desired_force 时，说明压得太重，vz 应为正，末端上抬。
      - contact_force < desired_force 时，说明压得太轻，vz 应为负，末端下压。
    """

    def __init__(self, mass, damping, max_velocity):
        self.mass = max(0.01, mass)
        self.damping = max(0.0, damping)
        self.max_velocity = max(0.001, max_velocity)
        self.velocity = 0.0
        self.acceleration = 0.0

    def step(self, contact_force, desired_force, dt):
        dt = clamp(dt, 0.001, 0.2)
        force_error = contact_force - desired_force
        self.acceleration = (force_error - self.damping * self.velocity) / self.mass
        self.velocity += self.acceleration * dt
        self.velocity = clamp(self.velocity, -self.max_velocity, self.max_velocity)
        return self.velocity, self.acceleration, force_error


def wait_for_joint_state(topic, timeout):
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("{} missing joints: {}".format(topic, ", ".join(missing)))
    return msg


def wait_for_cached_joint_state(cache, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        joints = cache.current_ordered()
        if joints is not None:
            return joints
        rospy.sleep(0.02)
    raise RuntimeError("No complete cached joint state from {}".format(cache.topic))


def wait_for_tf_pose(tf_reader, reference_frame, eef_link, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    last_error = None
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            return tf_reader.current_pose(reference_frame, eef_link, timeout=0.05)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            last_error = exc
            rospy.sleep(0.05)
    raise RuntimeError("TF lookup {} -> {} failed: {}".format(reference_frame, eef_link, last_error))


def wait_for_command_connection(pub, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if pub.get_num_connections() > 0:
            rospy.loginfo("Command topic connected: %s subscribers=%d", COMMAND_TOPIC, pub.get_num_connections())
            return True
        rospy.sleep(0.05)
    rospy.logwarn("No subscriber connected to %s within %.1fs", COMMAND_TOPIC, timeout)
    return False


def move_to_prepare_joints(group, prepare_joints):
    """用 MoveIt 到一个轻微弯曲姿态，避开零位奇异附近。"""
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


def publish_joint_command(pub, target_joints, horizon, stamp_mode, lead_time):
    """发布短 horizon 关节位置命令。

    延续 sim_08/sim_10 已验证的 future stamp，用少量未来时间偏置减少
    JointTrajectoryController 把目标点判定为过期的概率。
    """
    msg = JointTrajectory()
    if stamp_mode == "zero":
        msg.header.stamp = rospy.Time(0)
    elif stamp_mode == "future":
        msg.header.stamp = rospy.Time.now() + rospy.Duration(lead_time)
    else:
        msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = target_joints
    point.velocities = [0.0] * len(JOINT_NAMES)
    point.time_from_start = rospy.Duration(horizon)
    msg.points = [point]
    pub.publish(msg)


def smooth_pulse(elapsed, start, duration, height, ramp):
    """生成一个时间上的虚拟桌面小台阶。

    它模拟“TCP 原地不动，但桌面局部凸起被推到 TCP 下方”的情况。
    使用半余弦上升/下降，避免力突然跳变太猛。
    """
    if height <= 0.0 or duration <= 0.0 or elapsed < start or elapsed > start + duration:
        return 0.0

    local_t = elapsed - start
    ramp = clamp(ramp, 0.001, duration * 0.5)
    if local_t < ramp:
        return height * 0.5 * (1.0 - math.cos(math.pi * local_t / ramp))
    if local_t > duration - ramp:
        down_t = duration - local_t
        return height * 0.5 * (1.0 - math.cos(math.pi * down_t / ramp))
    return height


class StraightLinePath(object):
    """x 方向慢速直线往复轨迹。

    这是从 z_probe 进入擦桌子的第一步：只让 TCP 沿 x 做短距离往返，
    y 仍然固定，z 仍然由刚刚验证通过的单轴导纳负责。
    """

    def __init__(self, center_x, center_y, line_length, line_speed):
        self.center_x = center_x
        self.center_y = center_y
        self.half_length = max(0.005, 0.5 * abs(line_length))
        self.line_speed = max(0.001, abs(line_speed))
        self.period = 4.0 * self.half_length / self.line_speed

    def sample(self, elapsed):
        phase = (elapsed % self.period) / self.period
        if phase < 0.5:
            ratio = -1.0 + 4.0 * phase
            vx = self.line_speed
        else:
            ratio = 3.0 - 4.0 * phase
            vx = -self.line_speed
        x = self.center_x + self.half_length * ratio
        return [x, self.center_y], [vx, 0.0]


def spatial_gaussian_bump(x, y, center_x, center_y, height, sigma_x, sigma_y):
    """空间高度场：按当前 TCP/link x/y 位置计算凸起高度。

    x_bump 模式用它替代时间 smooth_pulse：
      - 机械臂没擦到凸起位置时，surface_step 接近 0；
      - TCP 经过凸起中心附近时，surface_step 接近 bump_height。

    这一步仍是虚拟高度场，不要求 Gazebo 视觉桌面真的有凸起。
    """
    sigma_x = max(0.001, abs(sigma_x))
    sigma_y = max(0.001, abs(sigma_y))
    dx = (x - center_x) / sigma_x
    dy = (y - center_y) / sigma_y
    return max(0.0, height) * math.exp(-0.5 * (dx * dx + dy * dy))


def spatial_wave_surface(x, y, center_x, center_y, height, cycles, line_length, sigma_y):
    """平滑正弦平方曲面高度场。

    x_wave 模式使用 sin^2 曲面，而不是离散方块或硬台阶：
      - 整条线从基准桌面平滑升起、落下、再升起；
      - surface_step 永远非负，视觉上像桌面上铺了一条连续波浪垫；
      - Gazebo 里的 DAE 网格只是这条数学曲面的视觉版本，不参与接触。

    这里仍是虚拟力场，不是真实 Gazebo 接触力。
    """
    height = max(0.0, height)
    line_length = max(0.001, abs(line_length))
    sigma_y = max(0.001, abs(sigma_y))
    cycles = max(0.25, abs(cycles))
    s = clamp((x - center_x) / line_length + 0.5, 0.0, 1.0)
    y_gain = math.exp(-0.5 * (((y - center_y) / sigma_y) ** 2))
    return height * (math.sin(math.pi * cycles * s) ** 2) * y_gain


def virtual_contact_force(surface_z, tcp_z, tcp_vz, stiffness, damping, max_force):
    """由虚拟桌面高度和 TCP 压入量计算法向力。

    penetration > 0 表示 TCP 在虚拟桌面以下，产生向上的接触力。
    damping 项用 TCP 竖直速度做简单阻尼，降低 z 方向抖动。
    """
    penetration = surface_z - tcp_z
    raw_force = stiffness * penetration - damping * tcp_vz
    contact_force = clamp(raw_force, 0.0, max_force)
    return contact_force, penetration


def clamp_vz_by_tcp_limits(vz, tcp_z, min_tcp_z, max_tcp_z):
    """虚拟 TCP 高度安全限幅。

    这是擦桌子单轴导纳的最后一道保护：
      - TCP 已低于 min_tcp_z 时，禁止继续向下；
      - TCP 已高于 max_tcp_z 时，禁止继续向上。
    """
    if tcp_z <= min_tcp_z and vz < 0.0:
        return 0.0, "min_tcp_z"
    if tcp_z >= max_tcp_z and vz > 0.0:
        return 0.0, "max_tcp_z"
    return vz, "free"


def damped_pseudo_inverse_qdot(group, current_joints, cartesian_velocity, damping, max_joint_velocity, translation_only):
    """用 Jacobian DLS 把末端速度转换为关节速度。"""
    jacobian = np.array(group.get_jacobian_matrix(current_joints), dtype=float)
    if jacobian.shape[0] < 6:
        raise RuntimeError("Unexpected Jacobian shape: {}".format(jacobian.shape))

    if translation_only:
        task_jacobian = jacobian[0:3, :]
        task_velocity = np.array(cartesian_velocity, dtype=float)
    else:
        task_jacobian = jacobian
        task_velocity = np.array([
            cartesian_velocity[0],
            cartesian_velocity[1],
            cartesian_velocity[2],
            0.0,
            0.0,
            0.0,
        ], dtype=float)

    lambda2 = damping * damping
    identity = np.eye(task_jacobian.shape[0])
    qdot = task_jacobian.T.dot(
        np.linalg.solve(task_jacobian.dot(task_jacobian.T) + lambda2 * identity, task_velocity)
    )
    qdot = limit_vector(qdot.tolist(), max_joint_velocity)

    singular_values = np.linalg.svd(task_jacobian, compute_uv=False)
    min_singular = float(np.min(singular_values)) if singular_values.size else 0.0
    return qdot, min_singular


def make_target_pose(current_pose, link_target_xyz, reference_frame):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = reference_frame
    msg.pose.position.x = link_target_xyz[0]
    msg.pose.position.y = link_target_xyz[1]
    msg.pose.position.z = link_target_xyz[2]
    msg.pose.orientation = current_pose.pose.orientation
    return msg


def rgba(marker, r, g, b, a):
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a


def make_marker(marker_id, marker_type, frame_id, namespace, xyz, scale, color, lifetime=2.0):
    """构造 RViz Marker。

    这些 marker 只用于理解当前控制点和虚拟桌面，不参与 Gazebo 物理接触。
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = xyz[0]
    marker.pose.position.y = xyz[1]
    marker.pose.position.z = xyz[2]
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    rgba(marker, color[0], color[1], color[2], color[3])
    marker.lifetime = rospy.Duration(lifetime)
    return marker


def make_line_marker(marker_id, frame_id, namespace, points_xyz, width, color, lifetime=2.0):
    marker = make_marker(
        marker_id,
        Marker.LINE_STRIP,
        frame_id,
        namespace,
        [0.0, 0.0, 0.0],
        [width, 0.0, 0.0],
        color,
        lifetime=lifetime,
    )
    marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points_xyz]
    return marker


def make_text_marker(marker_id, frame_id, namespace, xyz, text, height=0.035, lifetime=2.0):
    marker = make_marker(
        marker_id,
        Marker.TEXT_VIEW_FACING,
        frame_id,
        namespace,
        xyz,
        [0.0, 0.0, height],
        [1.0, 1.0, 1.0, 0.9],
        lifetime=lifetime,
    )
    marker.text = text
    return marker


def publish_visual_markers(
    pub,
    frame_id,
    mode,
    link_xyz,
    tcp_z,
    hold_xy,
    xy_ref,
    base_surface_z,
    surface_z,
    bump_center,
    args,
):
    """发布 TCP、擦拭块、桌面、凸起和擦拭线的 RViz MarkerArray。

    这一步解决“看不懂机械臂到底在擦哪里”的问题：
      - 绿色小球：虚拟 TCP/contact point；
      - 黄色半透明块：虚拟擦拭 pad；
      - 蓝色半透明平面：虚拟桌面高度；
      - 红色半透明凸起：x_bump 模式中的空间凸起；
      - 橙色曲线：x_wave 模式中的平滑起伏曲面轮廓；
      - 白色线：x 方向擦拭路径；
      - 紫色线：link7 到 TCP 的虚拟工具偏移。
    """
    tcp_xyz = [link_xyz[0], link_xyz[1], tcp_z]
    table_size_x = max(args.line_length + 0.08, 0.14)
    table_size_y = 0.10
    table_thickness = 0.003
    line_z = base_surface_z + 0.006
    lifetime = max(0.05, args.marker_lifetime)

    markers = MarkerArray()
    markers.markers.append(make_marker(1, Marker.SPHERE, frame_id, "tcp", tcp_xyz, [0.018, 0.018, 0.018], [0.0, 1.0, 0.0, 0.95], lifetime=lifetime))
    markers.markers.append(make_text_marker(11, frame_id, "labels", [tcp_xyz[0], tcp_xyz[1], tcp_xyz[2] + 0.045], "TCP", lifetime=lifetime))
    markers.markers.append(make_marker(2, Marker.CUBE, frame_id, "wiping_pad", tcp_xyz, [0.055, 0.030, 0.008], [1.0, 0.85, 0.05, 0.55], lifetime=lifetime))
    markers.markers.append(make_marker(
        3,
        Marker.CUBE,
        frame_id,
        "virtual_table",
        [hold_xy[0], hold_xy[1], base_surface_z - table_thickness * 0.5],
        [table_size_x, table_size_y, table_thickness],
        [0.15, 0.45, 1.0, 0.28],
        lifetime=lifetime,
    ))
    markers.markers.append(make_text_marker(12, frame_id, "labels", [hold_xy[0], hold_xy[1], base_surface_z + 0.035], "virtual table", lifetime=lifetime))
    markers.markers.append(make_line_marker(
        4,
        frame_id,
        "wiping_line",
        [
            [hold_xy[0] - args.line_length * 0.5, hold_xy[1] + args.line_y_offset, line_z],
            [hold_xy[0] + args.line_length * 0.5, hold_xy[1] + args.line_y_offset, line_z],
        ],
        0.008,
        [1.0, 1.0, 1.0, 0.9],
        lifetime=lifetime,
    ))
    markers.markers.append(make_line_marker(
        5,
        frame_id,
        "tool_offset",
        [[link_xyz[0], link_xyz[1], link_xyz[2]], tcp_xyz],
        0.004,
        [0.8, 0.2, 1.0, 0.9],
        lifetime=lifetime,
    ))
    markers.markers.append(make_marker(
        6,
        Marker.SPHERE,
        frame_id,
        "xy_ref",
        [xy_ref[0], xy_ref[1], line_z + 0.015],
        [0.014, 0.014, 0.014],
        [1.0, 1.0, 1.0, 0.9],
        lifetime=lifetime,
    ))

    if mode == "x_bump":
        markers.markers.append(make_marker(
            7,
            Marker.CYLINDER,
            frame_id,
            "spatial_bump",
            [bump_center[0], bump_center[1], base_surface_z + args.bump_height * 0.5],
            [args.bump_sigma_x * 4.0, args.bump_sigma_y * 2.0, max(args.bump_height, 0.002)],
            [1.0, 0.12, 0.08, 0.55],
            lifetime=lifetime,
        ))
        markers.markers.append(make_text_marker(
            13,
            frame_id,
            "labels",
            [bump_center[0], bump_center[1], base_surface_z + args.bump_height + 0.035],
            "spatial bump",
            lifetime=lifetime,
        ))

    if mode == "x_wave":
        profile_points = []
        sample_count = 36
        start_x = hold_xy[0] - args.line_length * 0.5
        end_x = hold_xy[0] + args.line_length * 0.5
        for i in range(sample_count + 1):
            ratio = float(i) / float(sample_count)
            x = start_x + (end_x - start_x) * ratio
            step = spatial_wave_surface(
                x,
                hold_xy[1] + args.line_y_offset,
                hold_xy[0],
                hold_xy[1] + args.line_y_offset,
                args.wave_height,
                args.wave_cycles,
                args.line_length,
                args.wave_sigma_y,
            )
            profile_points.append([x, hold_xy[1] + args.line_y_offset, base_surface_z + step + 0.012])
        markers.markers.append(make_line_marker(
            9,
            frame_id,
            "wave_profile",
            profile_points,
            0.010,
            [1.0, 0.45, 0.05, 0.95],
            lifetime=lifetime,
        ))
        markers.markers.append(make_marker(
            10,
            Marker.SPHERE,
            frame_id,
            "wave_first_peak",
            [hold_xy[0] - args.line_length * 0.25, hold_xy[1] + args.line_y_offset, base_surface_z + args.wave_height],
            [0.020, 0.020, 0.020],
            [1.0, 0.25, 0.05, 0.8],
            lifetime=lifetime,
        ))
        markers.markers.append(make_marker(
            14,
            Marker.SPHERE,
            frame_id,
            "wave_middle_valley",
            [hold_xy[0], hold_xy[1] + args.line_y_offset, base_surface_z],
            [0.018, 0.018, 0.018],
            [0.05, 0.75, 1.0, 0.8],
            lifetime=lifetime,
        ))
        markers.markers.append(make_marker(
            15,
            Marker.SPHERE,
            frame_id,
            "wave_second_peak",
            [hold_xy[0] + args.line_length * 0.25, hold_xy[1] + args.line_y_offset, base_surface_z + args.wave_height],
            [0.020, 0.020, 0.020],
            [1.0, 0.35, 0.05, 0.8],
            lifetime=lifetime,
        ))
        markers.markers.append(make_text_marker(
            16,
            frame_id,
            "labels",
            [hold_xy[0], hold_xy[1] + args.line_y_offset, base_surface_z + args.wave_height + 0.055],
            "smooth wave",
            lifetime=lifetime,
        ))

    markers.markers.append(make_marker(
        8,
        Marker.CUBE,
        frame_id,
        "current_surface_height",
        [link_xyz[0], link_xyz[1], surface_z],
        [0.020, 0.020, 0.004],
        [1.0, 0.25, 0.25, 0.7],
        lifetime=lifetime,
    ))
    pub.publish(markers)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="RM75-6F sim_09: slow table wiping with z-axis admittance.")
    parser.add_argument("--mode", default="x_line", choices=["z_probe", "x_line", "x_bump", "x_wave"])
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--command-horizon", type=float, default=0.16)
    parser.add_argument("--stamp-mode", default="future", choices=["future", "now", "zero"])
    parser.add_argument("--lead-time", type=float, default=0.03)
    parser.add_argument("--startup-timeout", type=float, default=45.0, help="Seconds to wait for joint states, controller, and TF.")
    parser.add_argument("--settle-time", type=float, default=4.0, help="Initial still time before the virtual wiping/bump motion starts.")
    parser.add_argument("--log-period", type=float, default=1.0, help="Throttled console log period in seconds.")

    parser.add_argument("--prepare", action="store_true", default=False)
    parser.add_argument("--no-prepare", dest="prepare", action="store_false")
    parser.add_argument("--prepare-joints", type=float, nargs=7, default=list(DEFAULT_PREPARE_JOINTS))

    parser.add_argument("--tool-offset-z", type=float, default=0.040, help="Virtual TCP below link7 along base z, m.")
    parser.add_argument("--surface-z", type=float, default=DEFAULT_TABLE_SURFACE_Z, help="Base virtual tabletop height, m.")
    parser.add_argument("--auto-surface-z", action="store_true", help="Estimate surface_z from the initial TCP height and desired force.")
    parser.add_argument("--desired-normal-force", type=float, default=8.0)
    parser.add_argument("--surface-stiffness", type=float, default=600.0)
    parser.add_argument("--surface-damping", type=float, default=6.0)
    parser.add_argument("--max-contact-force", type=float, default=45.0)

    parser.add_argument("--step-start", type=float, default=6.0)
    parser.add_argument("--step-duration", type=float, default=12.0)
    parser.add_argument("--step-height", type=float, default=0.0)
    parser.add_argument("--step-ramp", type=float, default=2.0)

    parser.add_argument("--line-length", type=float, default=0.180, help="x_line/x_wave wiping length in x direction, m.")
    parser.add_argument("--line-speed", type=float, default=0.004, help="x_line mode feed-forward x speed, m/s.")
    parser.add_argument("--line-center-x", type=float, default=DEFAULT_TABLE_CENTER_X, help="Wiping line center x in base_link, m.")
    parser.add_argument("--line-center-y", type=float, default=DEFAULT_TABLE_CENTER_Y, help="Wiping line center y in base_link, m.")
    parser.add_argument("--line-y-offset", type=float, default=0.0, help="Y offset from the fixed wiping line center, m.")
    parser.add_argument("--min-x-motion", type=float, default=0.025, help="x_line pass threshold for link x span, m.")
    parser.add_argument("--max-initial-xy-error", type=float, default=0.35, help="Warn if initial link7 xy is farther than this from the fixed wiping line center, m.")
    parser.add_argument("--max-initial-z-gap", type=float, default=0.12, help="Warn if initial virtual TCP is farther than this from surface_z, m.")
    parser.add_argument("--visual-table-center-x", type=float, default=DEFAULT_VISUAL_TABLE_CENTER_X)
    parser.add_argument("--visual-table-center-y", type=float, default=DEFAULT_VISUAL_TABLE_CENTER_Y)
    parser.add_argument("--visual-table-size-x", type=float, default=DEFAULT_VISUAL_TABLE_SIZE_X)
    parser.add_argument("--visual-table-size-y", type=float, default=DEFAULT_VISUAL_TABLE_SIZE_Y)
    parser.add_argument("--min-table-edge-margin", type=float, default=0.030, help="Warn if the commanded line is closer than this to the visual table edge, m.")

    parser.add_argument("--bump-height", type=float, default=0.010, help="x_bump spatial bump height, m.")
    parser.add_argument("--bump-x-offset", type=float, default=-0.018, help="x_bump center offset from initial x, m.")
    parser.add_argument("--bump-y-offset", type=float, default=0.0, help="x_bump center offset from line center y, m.")
    parser.add_argument("--bump-sigma-x", type=float, default=0.012, help="x_bump Gaussian sigma in x, m.")
    parser.add_argument("--bump-sigma-y", type=float, default=0.060, help="x_bump Gaussian sigma in y, m.")

    parser.add_argument("--wave-height", type=float, default=0.014, help="x_wave peak height above the base virtual surface, m.")
    parser.add_argument("--wave-cycles", type=float, default=2.0, help="x_wave sin^2 cycles along one line_length.")
    parser.add_argument("--wave-dip-ratio", type=float, default=0.65, help="Deprecated compatibility option; x_wave now uses non-negative sin^2 valleys.")
    parser.add_argument("--wave-spacing", type=float, default=0.055, help="x_wave distance from center dip to each peak, m.")
    parser.add_argument("--wave-sigma-x", type=float, default=0.020, help="x_wave Gaussian sigma along x, m.")
    parser.add_argument("--wave-sigma-y", type=float, default=0.060, help="x_wave Gaussian sigma along y, m.")

    parser.add_argument("--normal-mass", type=float, default=1.2)
    parser.add_argument("--normal-damping", type=float, default=75.0)
    parser.add_argument("--max-z-velocity", type=float, default=0.018)
    parser.add_argument("--max-penetration", type=float, default=0.030)
    parser.add_argument("--max-lift", type=float, default=0.050)
    parser.add_argument("--min-tcp-motion", type=float, default=0.004)

    parser.add_argument("--xy-hold-gain", type=float, default=1.8)
    parser.add_argument("--max-xy-velocity", type=float, default=0.012)

    parser.add_argument("--jacobian-damping", type=float, default=0.12)
    parser.add_argument("--max-joint-velocity", type=float, default=0.10)
    parser.add_argument("--max-joint-step", type=float, default=0.005)
    parser.add_argument("--min-safe-singular", type=float, default=0.02, help="Warn when the translational Jacobian minimum singular value drops below this.")
    parser.add_argument("--translation-only-jacobian", action="store_true")
    parser.add_argument("--publish-markers", action="store_true", default=True)
    parser.add_argument("--no-markers", dest="publish_markers", action="store_false")
    parser.add_argument("--marker-lifetime", type=float, default=2.0, help="RViz marker lifetime in seconds.")

    parser.add_argument("--group", default=GROUP_NAME)
    parser.add_argument("--eef-link", default=EEF_LINK)
    parser.add_argument("--reference-frame", default=REFERENCE_FRAME)
    parser.add_argument("--joint-states-topic", default=DEFAULT_JOINT_STATES_TOPIC)
    parser.add_argument("--velocity-scaling", type=float, default=0.10)
    parser.add_argument("--acceleration-scaling", type=float, default=0.10)
    args = parser.parse_args(argv)
    if args.auto_surface_z:
        args.surface_z = None
    return args


def main():
    # roslaunch 会追加 __name:=...、__log:=... 等 ROS remap 参数；
    # 先用 rospy.myargv() 过滤掉，再交给 argparse 解析脚本自己的参数。
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running. Start Gazebo + MoveIt first.")
        return 2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_09_admittance_table_wiping", anonymous=True)

    rospy.loginfo(
        "sim_09 starting: mode=%s prepare=%s waiting for %s",
        args.mode,
        args.prepare,
        args.joint_states_topic,
    )
    wait_for_joint_state(args.joint_states_topic, timeout=args.startup_timeout)
    joint_cache = JointStateCache(args.joint_states_topic)

    command_pub = rospy.Publisher(COMMAND_TOPIC, JointTrajectory, queue_size=1)
    rospy.loginfo("sim_09 waiting for command topic subscriber: %s", COMMAND_TOPIC)
    wait_for_command_connection(command_pub, timeout=args.startup_timeout)

    rospy.loginfo("sim_09 connecting to MoveIt group '%s'", args.group)
    group = MoveGroupCommander(args.group)
    group.set_end_effector_link(args.eef_link)
    group.set_pose_reference_frame(args.reference_frame)
    group.set_planning_time(3.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)

    if args.prepare:
        rospy.loginfo("sim_09 moving to prepare joints before wiping")
        move_to_prepare_joints(group, args.prepare_joints)

    tf_pose_reader = TfPoseReader()
    rospy.sleep(0.5)
    rospy.loginfo("sim_09 waiting for TF %s -> %s", args.reference_frame, args.eef_link)
    current_pose = wait_for_tf_pose(tf_pose_reader, args.reference_frame, args.eef_link, timeout=args.startup_timeout)
    initial_link_xyz = [
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z,
    ]
    initial_link_z_axis = link_z_axis_in_base(current_pose)
    initial_tool_down_score = -initial_link_z_axis[2]
    # Gazebo 桌面是固定模型，擦拭线也应该固定在桌面上，而不是跟着
    # 当前 wrist/link7 的高度和位置到处漂。默认中心和 world 文件一致：
    # x/y/surface_z 来自 cazhuozi1 -> cazhuozi2 短直线标定。
    hold_xy = [args.line_center_x, args.line_center_y]
    line_path = StraightLinePath(
        center_x=hold_xy[0],
        center_y=hold_xy[1] + args.line_y_offset,
        line_length=args.line_length,
        line_speed=args.line_speed,
    )
    bump_center = [
        hold_xy[0] + args.bump_x_offset,
        line_path.center_y + args.bump_y_offset,
    ]
    initial_tcp_z = initial_link_xyz[2] - args.tool_offset_z

    desired_penetration = args.desired_normal_force / max(1.0, args.surface_stiffness)
    base_surface_z = args.surface_z if args.surface_z is not None else initial_tcp_z + desired_penetration
    min_tcp_z = base_surface_z - abs(args.max_penetration)
    max_tcp_z = base_surface_z + abs(args.max_lift)

    admittance_z = ZAxisAdmittance(args.normal_mass, args.normal_damping, args.max_z_velocity)
    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)
    marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)

    # 预备动作之后重新取真实反馈，作为 q_last_commanded 的起点。
    measured = wait_for_cached_joint_state(joint_cache, timeout=args.startup_timeout)
    last_commanded_joints = list(measured)
    last_tcp_z = initial_tcp_z
    _, initial_min_singular = damped_pseudo_inverse_qdot(
        group,
        measured,
        [0.0, 0.0, 0.0],
        args.jacobian_damping,
        args.max_joint_velocity,
        args.translation_only_jacobian,
    )

    tcp_z_samples = []
    link_x_samples = []
    xy_error_samples = []
    surface_step_samples = []
    fz_samples = []
    joint_error_samples = []
    singular_samples = []

    initial_xy_error = math.hypot(hold_xy[0] - initial_link_xyz[0], hold_xy[1] - initial_link_xyz[1])
    initial_z_gap = initial_tcp_z - base_surface_z
    line_start = [hold_xy[0] - args.line_length * 0.5, hold_xy[1] + args.line_y_offset]
    line_end = [hold_xy[0] + args.line_length * 0.5, hold_xy[1] + args.line_y_offset]
    table_min_x = args.visual_table_center_x - args.visual_table_size_x * 0.5
    table_max_x = args.visual_table_center_x + args.visual_table_size_x * 0.5
    table_min_y = args.visual_table_center_y - args.visual_table_size_y * 0.5
    table_max_y = args.visual_table_center_y + args.visual_table_size_y * 0.5
    line_min_x = min(line_start[0], line_end[0])
    line_max_x = max(line_start[0], line_end[0])
    line_min_y = line_path.center_y - 0.5 * 0.09
    line_max_y = line_path.center_y + 0.5 * 0.09
    table_margin_x = bounds_margin(line_min_x, line_max_x, table_min_x, table_max_x)
    table_margin_y = bounds_margin(line_min_y, line_max_y, table_min_y, table_max_y)

    print("RM75-6F sim_09 slow table wiping admittance")
    print("Mode:                 ", args.mode)
    print("Command topic:        ", COMMAND_TOPIC)
    print("Joint state topic:    ", args.joint_states_topic)
    print("State topic:          ", STATE_TOPIC)
    print("Marker topic:         ", MARKER_TOPIC)
    print("Recommended Gazebo:   ", "roslaunch rm_75_gazebo arm_75_bumpy_wiping_moveit.launch")
    print("EEF link:             ", args.eef_link)
    print("Initial link7 xyz:    ", fmt3(initial_link_xyz))
    print("Initial link7 z axis: ", fmt3(initial_link_z_axis))
    print("Initial down score:   ", "{:.4f}".format(initial_tool_down_score))
    print("Initial TCP z:        ", "{:.4f}".format(initial_tcp_z))
    print("Hold xy:              ", fmt(hold_xy))
    print("Line start/end xy:    ", "{} -> {}".format(fmt(line_start), fmt(line_end)))
    print("Visual table x/y:     ", "[{:.3f}, {:.3f}] / [{:.3f}, {:.3f}]".format(
        table_min_x, table_max_x, table_min_y, table_max_y
    ))
    print("Line edge margin x/y: ", "{:.3f}m / {:.3f}m".format(table_margin_x, table_margin_y))
    print("Initial line xy err:  ", "{:.4f}m".format(initial_xy_error))
    print("Initial TCP-surface:  ", "{:.4f}m".format(initial_z_gap))
    print("Initial min singular: ", "{:.5f}".format(initial_min_singular))
    print("Line length/speed:    ", "{:.4f}m / {:.4f}mps".format(args.line_length, args.line_speed))
    print("Bump center xy:       ", fmt(bump_center))
    print("Bump height/sigma:    ", "{:.4f}m / [{:.4f}, {:.4f}]m".format(
        args.bump_height, args.bump_sigma_x, args.bump_sigma_y
    ))
    print("Wave peak x:          ", fmt([hold_xy[0] - args.line_length * 0.25, hold_xy[0] + args.line_length * 0.25]))
    print("Wave height/cycles:   ", "{:.4f}m / {:.2f}".format(args.wave_height, args.wave_cycles))
    print("Wave sigma y:         ", "{:.4f}m".format(args.wave_sigma_y))
    print("Base surface z:       ", "{:.4f}".format(base_surface_z))
    print("TCP z limits:         ", "[{:.4f}, {:.4f}]".format(min_tcp_z, max_tcp_z))
    print("Desired normal force: ", args.desired_normal_force)
    print("Desired penetration:  ", "{:.4f}m".format(desired_penetration))
    print("Temporal step:        ", "start={:.2f}s duration={:.2f}s height={:.4f}m".format(
        args.step_start, args.step_duration, args.step_height
    ))
    print("Settle/log/marker:    ", "{:.2f}s / {:.2f}s / {:.2f}s".format(
        args.settle_time, args.log_period, args.marker_lifetime
    ))
    print("Jacobian damping:     ", args.jacobian_damping)
    if initial_xy_error > args.max_initial_xy_error:
        rospy.logwarn(
            "Initial link7 xy is %.3fm from the fixed wiping line center; verify the line center or use --line-center-x/--line-center-y.",
            initial_xy_error,
        )
    if abs(initial_z_gap) > args.max_initial_z_gap:
        rospy.logwarn(
            "Initial virtual TCP is %.3fm away from surface_z; z admittance may need more time or a better table height.",
            initial_z_gap,
        )
    if initial_min_singular < args.min_safe_singular:
        rospy.logwarn(
            "Initial Jacobian is near singular: min_singular=%.5f. Use this run only as a visual check.",
            initial_min_singular,
        )
    if initial_tool_down_score < 0.5:
        rospy.logwarn(
            "Initial link7 local Z axis is not facing down enough: down_score=%.3f. Use --prepare or adjust prepare joints.",
            initial_tool_down_score,
        )
    if table_margin_x < args.min_table_edge_margin or table_margin_y < args.min_table_edge_margin:
        rospy.logwarn(
            "Commanded wiping line is close to or outside the visual table: margin_x=%.3fm margin_y=%.3fm. Reduce --line-length or adjust the line/table center.",
            table_margin_x,
            table_margin_y,
        )

    start_time = rospy.Time.now()
    last_time = start_time
    rate = rospy.Rate(args.rate)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()
        if elapsed > args.duration:
            break
        motion_elapsed = max(0.0, elapsed - max(0.0, args.settle_time))

        dt = clamp((now - last_time).to_sec(), 0.001, 0.2)
        last_time = now

        try:
            current_pose = tf_pose_reader.current_pose(args.reference_frame, args.eef_link, timeout=0.03)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, "Skipping cycle because TF is unavailable: %s", exc)
            rate.sleep()
            continue

        link_xyz = [
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
        ]
        link_z_axis = link_z_axis_in_base(current_pose)
        tool_down_score = -link_z_axis[2]
        tcp_z = link_xyz[2] - args.tool_offset_z
        tcp_vz = (tcp_z - last_tcp_z) / dt
        last_tcp_z = tcp_z

        if elapsed < args.settle_time:
            surface_source = "settle"
            surface_step = 0.0
        elif args.mode == "x_bump":
            surface_source = "spatial_bump"
            surface_step = spatial_gaussian_bump(
                link_xyz[0],
                link_xyz[1],
                bump_center[0],
                bump_center[1],
                args.bump_height,
                args.bump_sigma_x,
                args.bump_sigma_y,
            )
        elif args.mode == "x_wave":
            surface_source = "spatial_wave"
            surface_step = spatial_wave_surface(
                link_xyz[0],
                link_xyz[1],
                hold_xy[0],
                line_path.center_y,
                args.wave_height,
                args.wave_cycles,
                args.line_length,
                args.wave_sigma_y,
            )
        elif args.step_height <= 0.0:
            # 默认 x_line 只做固定桌面上的直线擦拭；不额外叠加时间台阶，
            # 这样先把“沿桌面一个方向运动”这件事看清楚。
            surface_source = "fixed_surface"
            surface_step = 0.0
        else:
            surface_source = "temporal_step"
            surface_step = smooth_pulse(
                motion_elapsed,
                args.step_start,
                args.step_duration,
                args.step_height,
                args.step_ramp,
            )
        surface_z = base_surface_z + surface_step
        equilibrium_tcp_z = surface_z - desired_penetration
        tcp_equilibrium_error = tcp_z - equilibrium_tcp_z
        contact_force, penetration = virtual_contact_force(
            surface_z,
            tcp_z,
            tcp_vz,
            args.surface_stiffness,
            args.surface_damping,
            args.max_contact_force,
        )

        raw_vz, z_acceleration, force_error = admittance_z.step(
            contact_force,
            args.desired_normal_force,
            dt,
        )
        vz, tcp_limit_state = clamp_vz_by_tcp_limits(raw_vz, tcp_z, min_tcp_z, max_tcp_z)

        if elapsed < args.settle_time:
            xy_ref = list(hold_xy)
            xy_ref_velocity = [0.0, 0.0]
        elif args.mode in ["x_line", "x_bump", "x_wave"]:
            xy_ref, xy_ref_velocity = line_path.sample(motion_elapsed)
        else:
            xy_ref = list(hold_xy)
            xy_ref_velocity = [0.0, 0.0]

        # z_probe 模式下 x/y 只是保持；x_line/x_bump/x_wave 模式下 x 加入
        # 慢速往复参考，y 仍然固定。这样每次只新增一个自由度，便于定位问题。
        vx = xy_ref_velocity[0] + args.xy_hold_gain * (xy_ref[0] - link_xyz[0])
        vy = xy_ref_velocity[1] + args.xy_hold_gain * (xy_ref[1] - link_xyz[1])
        vx = clamp(vx, -args.max_xy_velocity, args.max_xy_velocity)
        vy = clamp(vy, -args.max_xy_velocity, args.max_xy_velocity)
        cartesian_velocity = [vx, vy, vz]

        measured_joints = joint_cache.current_ordered()
        current_joints = measured_joints if measured_joints is not None else last_commanded_joints
        qdot, min_singular = damped_pseudo_inverse_qdot(
            group,
            current_joints,
            cartesian_velocity,
            args.jacobian_damping,
            args.max_joint_velocity,
            args.translation_only_jacobian,
        )

        # Jacobian 用当前真实反馈；命令积分从上一帧命令继续，沿用 sim_08 已验证逻辑。
        raw_target_joints = [
            last_commanded_joints[i] + qdot[i] * dt
            for i in range(len(last_commanded_joints))
        ]
        target_joints = limit_joint_step(last_commanded_joints, raw_target_joints, args.max_joint_step)
        last_commanded_joints = list(target_joints)
        publish_joint_command(command_pub, target_joints, args.command_horizon, args.stamp_mode, args.lead_time)

        joint_error = None
        if measured_joints is not None:
            joint_error = [target_joints[i] - measured_joints[i] for i in range(len(target_joints))]

        target_link_xyz = [xy_ref[0], xy_ref[1], link_xyz[2] + vz * dt]
        target_pub.publish(make_target_pose(current_pose, target_link_xyz, args.reference_frame))
        if args.publish_markers:
            publish_visual_markers(
                marker_pub,
                args.reference_frame,
                args.mode,
                link_xyz,
                tcp_z,
                hold_xy,
                xy_ref,
                base_surface_z,
                surface_z,
                bump_center,
                args,
            )

        tcp_z_samples.append(tcp_z)
        link_x_samples.append(link_xyz[0])
        xy_error_samples.append(math.hypot(xy_ref[0] - link_xyz[0], xy_ref[1] - link_xyz[1]))
        surface_step_samples.append(surface_step)
        fz_samples.append(contact_force)
        singular_samples.append(min_singular)
        if joint_error is not None:
            joint_error_samples.append(max_abs(joint_error))

        state_text = (
            "elapsed={:.2f} mode={} xy_ref={} link_xyz={} link7_z_axis={} tool_down_score={:.4f} "
            "tcp_z={:.4f} base_surface_z={:.4f} "
            "surface_source={} surface_step={:.4f} surface_z={:.4f} eq_tcp_z={:.4f} tcp_eq_err={:.4f} "
            "bump_center={} penetration={:.4f} "
            "fz_virtual_N={:.3f} desired_fz_N={:.3f} force_error_N={:.3f} "
            "vz_raw={:.4f} vz_admittance={:.4f} tcp_limit={} cartesian_velocity={} "
            "xy_error={:.4f} qdot_max={:.4f} min_singular={:.5f} joint_error_max={:.4f}"
        ).format(
            elapsed,
            args.mode,
            fmt([xy_ref[0], xy_ref[1]]),
            fmt3(link_xyz),
            fmt3(link_z_axis),
            tool_down_score,
            tcp_z,
            base_surface_z,
            surface_source,
            surface_step,
            surface_z,
            equilibrium_tcp_z,
            tcp_equilibrium_error,
            fmt(bump_center),
            penetration,
            contact_force,
            args.desired_normal_force,
            force_error,
            raw_vz,
            vz,
            tcp_limit_state,
            fmt3(cartesian_velocity),
            xy_error_samples[-1],
            max_abs(qdot),
            min_singular,
            max_abs(joint_error) if joint_error is not None else -1.0,
        )
        state_pub.publish(String(data=state_text))

        rospy.loginfo_throttle(
            max(0.1, args.log_period),
            "mode=%s x=%.4f x_ref=%.4f tcp_z=%.4f eq_err=%.4f down=%.3f step=%.4f src=%s fz=%.2fN vz=%.4f xy_err=%.4f min_singular=%.5f joint_err=%.4f",
            args.mode,
            link_xyz[0],
            xy_ref[0],
            tcp_z,
            tcp_equilibrium_error,
            tool_down_score,
            surface_step,
            surface_source,
            contact_force,
            vz,
            xy_error_samples[-1],
            min_singular,
            max_abs(joint_error) if joint_error is not None else -1.0,
        )
        if min_singular < args.min_safe_singular:
            rospy.logwarn_throttle(1.0, "Jacobian is near singular: min_singular=%.5f", min_singular)

        rate.sleep()

    print("Experiment finished.")
    if tcp_z_samples:
        tcp_span = max(tcp_z_samples) - min(tcp_z_samples)
        x_span = max(link_x_samples) - min(link_x_samples) if link_x_samples else 0.0
        fz_span = max(fz_samples) - min(fz_samples) if fz_samples else 0.0
        surface_step_span = max(surface_step_samples) - min(surface_step_samples) if surface_step_samples else 0.0
        max_xy_error = max(xy_error_samples) if xy_error_samples else -1.0
        max_joint_error = max(joint_error_samples) if joint_error_samples else -1.0
        min_singular_observed = min(singular_samples) if singular_samples else -1.0
        print(
            "RESULT mode={} tcp_z_span={:.4f}m x_span={:.4f}m surface_step_span={:.4f}m fz_span={:.3f}N "
            "max_xy_error={:.4f}m max_joint_error={:.4f}rad min_singular={:.5f} "
            "min_tcp_motion={:.4f}m min_x_motion={:.4f}m".format(
                args.mode,
                tcp_span,
                x_span,
                surface_step_span,
                fz_span,
                max_xy_error,
                max_joint_error,
                min_singular_observed,
                args.min_tcp_motion,
                args.min_x_motion,
            )
        )
        if args.mode == "x_line" and args.step_height <= 0.0:
            if x_span >= args.min_x_motion:
                rospy.loginfo("PASS: fixed x-line wiping produced visible TCP x motion.")
            else:
                rospy.logwarn("WARN: fixed x-line wiping did not reach expected x motion; check line center, speed, and singularity.")
        elif args.mode in ["x_line", "x_bump", "x_wave"]:
            if tcp_span >= args.min_tcp_motion and x_span >= args.min_x_motion:
                rospy.loginfo("PASS: x wiping and z-axis admittance both produced visible TCP motion.")
            else:
                rospy.logwarn("WARN: x mode did not reach expected x/z motion; check line speed, damping, and singularity.")
        elif tcp_span >= args.min_tcp_motion:
            rospy.loginfo("PASS: z-axis admittance produced visible TCP motion.")
        else:
            rospy.logwarn("WARN: TCP z motion is small; check surface step, force, damping, and singularity.")

    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
