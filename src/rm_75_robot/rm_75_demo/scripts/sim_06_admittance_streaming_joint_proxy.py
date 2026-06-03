#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_06_admittance_streaming_joint_proxy.py

RM75-6F Gazebo 仿真柔顺控制实验：连续发布 /command 的导纳演示版。

这个文件是 sim_06 的“分层 + 中文详细注释 + 关节命令整形”版本。
它保留 joint-proxy 学习后端，同时增加低通滤波、每周期关节步长限幅和 /joint_states
反馈诊断，让 Gazebo 中的连续 /command 柔顺演示更容易调参和观察。

========================
一、整体分层
========================

Layer 0：全局配置层
    - 定义 MoveIt group、末端 link、参考坐标系、关节名、topic 名称等。
    - 这些常量决定本节点连接哪一个机器人、哪一个控制器、发布哪些观察话题。

Layer 1：通用工具层
    - clamp、fmt、MoveIt plan 结果兼容处理、PoseStamped 拷贝等。
    - 不涉及具体控制算法，只负责减少重复代码。

Layer 2：外部力输入层
    - WrenchInput 类。
    - 可选订阅 /rm75_admittance_stream/target_wrench。
    - 当前只使用 force.x/y/z，不使用 torque，因为本实验只做平移导纳。

Layer 3：导纳控制模型层
    - TranslationalAdmittance 类。
    - 用虚拟质量 M、阻尼 D、刚度 K，把外力 F 转换成笛卡尔 offset。
    - 公式：M * x_ddot + D * x_dot + K * x = F_ext。

Layer 4：机器人准备与状态检查层
    - wait_for_joint_state：检查 /joint_states 是否正常。
    - JointStateCache：持续缓存 /joint_states，用于诊断关节跟随误差。
    - wait_for_command_connection：检查 JointTrajectoryController 是否订阅 /command。
    - move_to_prepare_joints：用 MoveIt 先移动到一个小弯曲姿态，避开零位奇异附近。

Layer 5：控制输出层
    - map_offset_to_joint_target：把导纳 offset 映射为一组关节目标。
      注意：这是 joint-proxy 学习映射，不是真正 IK/Jacobian。
    - smooth_joint_target：对关节目标做一阶低通滤波。
    - limit_joint_step：限制每周期最大关节目标变化量。
    - publish_joint_command：连续发布 trajectory_msgs/JointTrajectory 到 /arm/arm_joint_controller/command。

Layer 6：参数与主流程层
    - parse_args：命令行参数。
    - main：初始化 ROS/MoveIt，构造对象，进入控制循环。

========================
二、当前实验的控制链路
========================

默认内置虚拟力 Fx=8N，持续 6s：

    虚拟外力 F
      -> 导纳模型计算 offset / velocity / acceleration
      -> joint-proxy 把 offset 转成 target_joints
      -> 按 30Hz 发布 /arm/arm_joint_controller/command
      -> Gazebo 中 JointTrajectoryController 跟随目标关节角
      -> 撤力后 offset 回零，机械臂回到预备姿态附近

========================
三、重要限制
========================

1. 该脚本是 Gazebo 学习演示，不是实物控制程序。
2. joint-proxy 不是严格的逆运动学，只是为了让“柔顺偏移”在 Gazebo 中可见。
3. 后续真正做擦桌子，应逐步替换为：
    - IK 位置映射；
    - Jacobian 速度映射；
    - 或 controller 层更直接的位置 / 速度 / 力矩控制。
"""

from __future__ import print_function

# ==============================
# Layer 0.1：标准库导入
# ==============================
import argparse
import sys
import threading

# ==============================
# Layer 0.2：ROS / MoveIt 消息和接口导入
# ==============================
import moveit_commander
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# =============================================================================
# Layer 0：全局配置层
# =============================================================================
# MoveIt 规划组名称。RM75-6F 的 MoveIt 配置里，7 个关节组成的机械臂规划组一般叫 arm。
GROUP_NAME = "arm"

# 当前实验使用 link7 作为末端执行器 link。
# 注意：如果后续定义了夹爪 TCP 或擦桌子工具 TCP，这里可能要换成实际 TCP link。
EEF_LINK = "link7"

# 导纳 offset 的参考坐标系。
# 当前脚本假设虚拟力和 offset 都在 base_link 下表达，不做 TF 坐标变换。
REFERENCE_FRAME = "base_link"

# 7 自由度 RM75 的关节名，需要和 /joint_states、JointTrajectoryController 的关节名一致。
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

# 预备姿态：启动后先让机械臂从零位移动到一个轻微弯曲的姿态。
# 目的：
#   1. 避免全零竖直姿态附近的 IK / 奇异问题；
#   2. 让后续 Gazebo 中的柔顺偏移更容易观察。
DEFAULT_PREPARE_JOINTS = [0.0, 0.25, 0.0, 0.35, 0.0, 0.20, 0.0]

# 可选外部虚拟力输入 topic。
# 使用 --use-topic-wrench 时，节点会从这里读取 geometry_msgs/WrenchStamped。
WRENCH_TOPIC = "/rm75_admittance_stream/target_wrench"

# 发布导纳 offset 对应的“理论末端目标位姿”。
# 注意：当前 joint-proxy 后端不保证 link7 精确到该 pose；这个 topic 主要用于观察导纳输出。
TARGET_POSE_TOPIC = "/rm75_admittance_stream/target_pose"

# 发布当前导纳状态，便于 rostopic echo 观察。
STATE_TOPIC = "/rm75_admittance_stream/state"

# 关键控制输出 topic。
# /arm/arm_joint_controller 是 Gazebo 里的 position_controllers/JointTrajectoryController。
# 它订阅 /command，消息类型是 trajectory_msgs/JointTrajectory。
COMMAND_TOPIC = "/arm/arm_joint_controller/command"

# Gazebo / robot_state_publisher 发布的关节状态。
JOINT_STATES_TOPIC = "/joint_states"


# =============================================================================
# Layer 1：通用工具层
# =============================================================================
def clamp(value, lower, upper):
    """把数值限制在 [lower, upper] 范围内。

    在控制程序中，限幅非常重要：
    - 防止导纳 offset 过大；
    - 防止虚拟速度过大；
    - 防止关节目标瞬间跳变太大。
    """
    return max(lower, min(upper, value))


def fmt(values):
    """把任意长度的数值列表格式化成 [0.0000, ...] 字符串，用于日志输出。"""
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt3(values):
    """把 3 维向量格式化成固定格式字符串，主要用于 force / offset / velocity。"""
    return "[{:.4f}, {:.4f}, {:.4f}]".format(values[0], values[1], values[2])


def fmt_optional(values):
    """格式化可能不存在的关节反馈。

    /joint_states 偶发还没更新时，用 unavailable 表示，不让诊断字符串报错。
    """
    if values is None:
        return "unavailable"
    return fmt(values)


def max_abs(values):
    """返回向量中最大的绝对值，用于快速观察最大关节跟踪误差。"""
    if not values:
        return 0.0
    return max(abs(v) for v in values)


def normalize_plan_result(plan_result):
    """兼容不同 MoveIt Python 版本的 group.plan() 返回值。

    不同 ROS / MoveIt 版本里，MoveGroupCommander.plan() 可能返回：
    1. RobotTrajectory 对象；
    2. tuple，例如 (success, plan, planning_time, error_code)。

    为了让脚本在你的 Noetic 环境和其他环境中都更稳，这里统一转成：
        success, plan, error_code
    """
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code

    # 如果不是 tuple，就认为它是 RobotTrajectory。
    # 只要轨迹点数不为 0，就认为规划成功。
    points = plan_result.joint_trajectory.points
    return bool(points), plan_result, None


def error_name(error_code):
    """把 MoveItErrorCodes 转成可读字符串。

    例如：
        SUCCESS
        TIMED_OUT
        PLANNING_FAILED

    这样日志里不会只出现数字错误码。
    """
    if error_code is None:
        return "unknown"

    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


def copy_pose_stamped(source):
    """手动深拷贝 PoseStamped。

    这里只拷贝 header、position、orientation。
    目的：
        根据 equilibrium_pose 生成 target_pose 时，不直接修改原始平衡位姿。
    """
    copied = PoseStamped()
    copied.header.frame_id = source.header.frame_id
    copied.header.stamp = source.header.stamp
    copied.pose.position.x = source.pose.position.x
    copied.pose.position.y = source.pose.position.y
    copied.pose.position.z = source.pose.position.z
    copied.pose.orientation = source.pose.orientation
    return copied


# =============================================================================
# Layer 2：外部力输入层
# =============================================================================
class WrenchInput(object):
    """订阅并缓存最新的外部虚拟力。

    Topic:
        /rm75_admittance_stream/target_wrench

    Type:
        geometry_msgs/WrenchStamped

    当前实验约定：
        - force.x / force.y / force.z 的单位是 N；
        - 坐标系默认是 base_link；
        - torque.x / torque.y / torque.z 暂时不用；
        - 不做 TF 变换。

    为什么需要 timeout：
        如果外部发布器停止发布，而控制节点还一直使用旧的力值，机械臂会持续偏移。
        所以超过 wrench_timeout 后，自动把外力视为 0。
    """

    def __init__(self, expected_frame, timeout):
        self.expected_frame = expected_frame
        self.timeout = timeout

        # 当前缓存的三维力，单位 N。
        self.force = [0.0, 0.0, 0.0]

        # 上一次收到 wrench 的时间。Time(0) 表示还没收到过。
        self.last_stamp = rospy.Time(0)

        # 只警告一次坐标系不一致，避免日志刷屏。
        self.warned_frame = False

        # queue_size=1 表示只保留最新 wrench。
        # 对控制输入来说，旧力值没有意义。
        self.sub = rospy.Subscriber(WRENCH_TOPIC, WrenchStamped, self._callback, queue_size=1)

    def _callback(self, msg):
        """收到外部 WrenchStamped 后，更新缓存。"""
        # 如果消息自带 frame_id，且不是期望的 base_link，则提示风险。
        # 当前脚本不做 TF，所以不同 frame 的力不能直接混用。
        if msg.header.frame_id and msg.header.frame_id != self.expected_frame and not self.warned_frame:
            rospy.logwarn(
                "Received wrench in frame '%s', but this demo assumes '%s'. No TF transform is applied.",
                msg.header.frame_id,
                self.expected_frame,
            )
            self.warned_frame = True

        # 只取平移力；力矩暂时忽略。
        self.force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.last_stamp = rospy.Time.now()

    def current_force(self):
        """返回当前有效外力。

        逻辑：
            - 从未收到过 wrench：返回 0；
            - 超过 timeout 没有更新：返回 0；
            - 否则返回最新缓存 force。
        """
        if self.last_stamp == rospy.Time(0):
            return [0.0, 0.0, 0.0]
        if (rospy.Time.now() - self.last_stamp).to_sec() > self.timeout:
            return [0.0, 0.0, 0.0]
        return list(self.force)


# =============================================================================
# Layer 3：导纳控制模型层
# =============================================================================
class TranslationalAdmittance(object):
    """三轴平移导纳模型。

    经典导纳方程：
        M * x_ddot + D * x_dot + K * x = F_ext

    各变量含义：
        M：虚拟质量，越大响应越慢；
        D：虚拟阻尼，越大越稳但越迟钝；
        K：虚拟刚度，越大越硬，offset 越小；
        F_ext：外部力；
        x：当前导纳输出的虚拟位移 offset；
        x_dot：虚拟速度；
        x_ddot：虚拟加速度。

    注意：
        x 不是机器人绝对位置，而是相对 equilibrium_pose 的小偏移。
        当前脚本后端再把 offset 映射到 joint target。
    """

    def __init__(self, mass, damping, stiffness, max_offset, max_velocity):
        # 每个方向一组 M/D/K。
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness

        # 安全限幅：防止仿真里目标偏移过大。
        self.max_offset = max_offset
        self.max_velocity = max_velocity

        # 导纳状态量：位移和速度。
        # 初始时没有外力，offset=0，velocity=0。
        self.offset = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

    def step(self, force, dt):
        """执行一次导纳离散积分。

        输入：
            force: [Fx, Fy, Fz]，单位 N；
            dt: 控制周期，单位 s。

        离散更新：
            a = (F - D*v - K*x) / M
            v = v + a*dt
            x = x + v*dt

        返回：
            acceleration: 当前三轴虚拟加速度，仅用于日志观察。
        """
        # 避免 dt 异常。
        # 如果仿真暂停或系统卡顿，dt 可能突然很大；直接积分会导致跳变。
        dt = clamp(dt, 0.001, 0.2)

        acceleration = [0.0, 0.0, 0.0]

        for i in range(3):
            # 导纳方程移项：
            #   M*a = F - D*v - K*x
            #   a = (F - D*v - K*x) / M
            acceleration[i] = (
                force[i]
                - self.damping[i] * self.velocity[i]
                - self.stiffness[i] * self.offset[i]
            ) / self.mass[i]

            # 速度积分。
            self.velocity[i] += acceleration[i] * dt

            # 速度限幅，防止 target_joints 变化太快。
            self.velocity[i] = clamp(self.velocity[i], -self.max_velocity[i], self.max_velocity[i])

            # 位移积分。
            self.offset[i] += self.velocity[i] * dt

            # 位移限幅。当前 x 方向默认最大 0.10m，仅用于 Gazebo 可视化。
            self.offset[i] = clamp(self.offset[i], -self.max_offset[i], self.max_offset[i])

        return acceleration


# =============================================================================
# Layer 3.5：导纳输出可视化辅助
# =============================================================================
def make_target_pose(equilibrium_pose, offset, reference_frame):
    """根据平衡位姿和 offset 生成理论 target_pose。

    这个 PoseStamped 发布到 /rm75_admittance_stream/target_pose。
    作用：
        方便你在 RViz / rostopic 中看到导纳模型希望末端到哪里。

    注意：
        当前执行端使用 joint-proxy，不保证 link7 精确到这个 target_pose。
        所以 target_pose 是“导纳理论输出”，不是严格执行反馈。
    """
    target = copy_pose_stamped(equilibrium_pose)
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = reference_frame

    # 平移位置 = 初始平衡位置 + 导纳 offset。
    target.pose.position.x = equilibrium_pose.pose.position.x + offset[0]
    target.pose.position.y = equilibrium_pose.pose.position.y + offset[1]
    target.pose.position.z = equilibrium_pose.pose.position.z + offset[2]

    # 姿态保持不变。当前实验只做平移导纳。
    return target


def demo_force(args, elapsed):
    """生成默认内置虚拟力。

    如果不使用 --use-topic-wrench，就用这个函数产生力输入：
        前 demo_force_duration 秒：返回 [Fx, Fy, Fz]
        之后：返回 [0, 0, 0]

    默认：
        Fx=8N，持续 6s。
    """
    if elapsed < args.demo_force_duration:
        return [args.demo_force_x, args.demo_force_y, args.demo_force_z]
    return [0.0, 0.0, 0.0]


# =============================================================================
# Layer 4：机器人准备与状态检查层
# =============================================================================
def move_to_prepare_joints(group, prepare_joints):
    """用 MoveIt 把机械臂移动到预备姿态。

    这一步只在实验开始时做一次，不属于连续柔顺控制循环。

    为什么需要预备姿态：
        1. 全零姿态附近，末端竖直，IK/Cartesian path 容易失败；
        2. 机械臂弯曲后，Gazebo 中关节运动更明显；
        3. 后续 joint-proxy 映射从该姿态附近做小范围偏移，更稳定。
    """
    # 告诉 MoveIt：规划从当前真实状态开始。
    group.set_start_state_to_current_state()

    # 给 MoveIt 一个关节目标，而不是末端 pose 目标。
    # 关节目标规划通常比末端 IK 更稳。
    group.set_joint_value_target(dict(zip(JOINT_NAMES, prepare_joints)))

    # 规划到预备姿态。
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

    # 执行规划轨迹，并等待执行完成。
    ok = group.execute(plan, wait=True)
    group.stop()
    rospy.loginfo("Prepare joint execute returned: %s", ok)
    if not ok:
        raise RuntimeError("Prepare joint execution failed")


def wait_for_command_connection(pub, timeout):
    """等待 /arm/arm_joint_controller/command 有订阅者。

    这里的订阅者应当是 JointTrajectoryController。
    如果没有订阅者，说明控制器没有启动，发布 command 也不会让机械臂运动。
    """
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if pub.get_num_connections() > 0:
            rospy.loginfo("Command topic connected: %s subscribers=%d", COMMAND_TOPIC, pub.get_num_connections())
            return True
        rospy.sleep(0.05)

    rospy.logwarn("No subscriber connected to %s within %.1fs", COMMAND_TOPIC, timeout)
    return False


def wait_for_joint_state(timeout):
    """等待 /joint_states，并检查 7 个关节是否都存在。

    如果 /joint_states 没有数据：
        - Gazebo 可能没启动；
        - ros_control 可能没加载；
        - robot_state_publisher / joint_state_publisher 可能异常。

    如果缺少某些 joint：
        - 关节名和控制器配置可能不一致。
    """
    msg = rospy.wait_for_message(JOINT_STATES_TOPIC, JointState, timeout=timeout)
    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("/joint_states missing joints: {}".format(", ".join(missing)))
    rospy.loginfo("Preflight OK: received %d joints from %s", len(msg.name), JOINT_STATES_TOPIC)
    return msg


class JointStateCache(object):
    """缓存最新 /joint_states，给控制循环做反馈诊断。

    Topic:
        /joint_states

    Type:
        sensor_msgs/JointState

    当前 sim_06 仍然不是闭环伺服控制器，关节反馈主要用于观察：
        - 控制器是否真的在跟随 target_joints；
        - target_joints 和 measured_joints 的误差有多大；
        - rate / command_horizon / smoothing 调参是否合理。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._positions_by_name = {}
        self.sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self._callback, queue_size=1)

    def _callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        with self._lock:
            self._positions_by_name = positions

    def current_ordered(self):
        """按 JOINT_NAMES 顺序返回最新关节角。

        如果某个关节还没有反馈，返回 None，避免主循环使用不完整数据。
        """
        with self._lock:
            if not self._positions_by_name:
                return None
            missing = [name for name in JOINT_NAMES if name not in self._positions_by_name]
            if missing:
                return None
            return [self._positions_by_name[name] for name in JOINT_NAMES]


# =============================================================================
# Layer 5：控制输出层
# =============================================================================
def map_offset_to_joint_target(equilibrium_joints, offset):
    """把导纳 offset 映射成关节目标。

    重要说明：
        这是 joint-proxy 学习映射，不是真正的 IK 或 Jacobian 控制。

    为什么先这样做：
        之前用 MoveIt pose / Cartesian path 做每周期末端跟随时，容易出现 IK 失败、
        fraction 很低、规划超时等问题。为了第一阶段能稳定看到柔顺现象，
        这里先用经验映射把 offset 转成几个关节的小幅变化。

    当前映射规律：
        - offset_x 主要影响 joint2、joint4、joint6；
        - offset_y 影响 joint1；
        - offset_z 也混合到 joint2、joint4、joint6。

    后续替换方向：
        offset -> IK 求 q_des
        或
        Cartesian velocity -> Jacobian pseudo-inverse -> q_dot_des
    """
    # 从平衡关节角开始，每个周期只在平衡姿态附近做小范围偏移。
    target = list(equilibrium_joints)

    # 再做一次映射前限幅，防止外部参数改得过大。
    x = clamp(offset[0], -0.10, 0.10)
    y = clamp(offset[1], -0.05, 0.05)
    z = clamp(offset[2], -0.04, 0.04)

    # joint1 控制底座旋转，用来表现 y 方向小偏移。
    target[0] += 1.0 * y

    # joint2 / joint4 / joint6 组合出一个比较明显的“前后摆动”效果。
    # 这些系数是经验可视化参数，不代表机器人真实运动学逆解。
    target[1] += 1.2 * x - 0.8 * z
    target[3] += -2.8 * x + 1.6 * z
    target[5] += 1.6 * x - 0.8 * z

    return target


def smooth_joint_target(previous_target, raw_target, alpha):
    """对关节目标做一阶低通滤波。

    物理直觉：
        raw_target 是 joint-proxy 直接算出来的目标；
        previous_target 是上一周期真正发给控制器的目标；
        alpha 越小，目标越平滑但响应越慢；
        alpha 越接近 1，越接近原始目标但可能更生硬。

    公式：
        q_cmd = q_prev + alpha * (q_raw - q_prev)
    """
    if previous_target is None:
        return list(raw_target)

    alpha = clamp(alpha, 0.0, 1.0)
    return [
        previous_target[i] + alpha * (raw_target[i] - previous_target[i])
        for i in range(len(raw_target))
    ]


def limit_joint_step(previous_target, target, max_step):
    """限制每个控制周期的关节目标变化量。

    这是发 /command 前的最后一道保护：
        - 防止参数调得太激进时，目标关节角瞬间跳变；
        - 降低 Gazebo 中 position controller 的突兀响应；
        - 为后续 IK/Jacobian 版本复用安全限幅逻辑。

    max_step 单位是 rad/control-cycle。
    如果 max_step <= 0，则不启用这个限幅。
    """
    if previous_target is None or max_step <= 0.0:
        return list(target)

    limited = []
    for i in range(len(target)):
        delta = target[i] - previous_target[i]
        limited.append(previous_target[i] + clamp(delta, -max_step, max_step))
    return limited


def publish_joint_command(pub, target_joints, horizon):
    """向 JointTrajectoryController 连续发布短 horizon 关节目标。

    发布 topic：
        /arm/arm_joint_controller/command

    消息类型：
        trajectory_msgs/JointTrajectory

    控制器行为：
        每次收到一条 JointTrajectory，就尝试在 time_from_start 指定的时间内到达目标点。

    为什么只有一个 trajectory point：
        当前外层导纳循环已经在持续刷新目标；
        每个周期只给控制器一个短时目标即可。

    horizon 参数的影响：
        - horizon 太大：响应慢，看起来滞后；
        - horizon 太小：控制器来不及跟随，可能抖动；
        - 默认 0.12s，配合 30Hz 发布较稳。
    """
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = target_joints

    # 这里显式给 0 速度，表示目标点速度为 0。
    # 对 position JointTrajectoryController 来说，核心仍是位置点。
    point.velocities = [0.0] * len(JOINT_NAMES)

    # 控制器应在 horizon 秒后到达这个点。
    point.time_from_start = rospy.Duration(horizon)
    msg.points = [point]

    pub.publish(msg)


# =============================================================================
# Layer 6.1：命令行参数层
# =============================================================================
def parse_args(argv):
    """解析命令行参数。

    常用调参：
        --rate
        --command-horizon
        --mass
        --damping
        --stiffness
        --max-offset
        --demo-force-x
        --use-topic-wrench
    """
    parser = argparse.ArgumentParser(
        description="RM75-6F simulation experiment 06: streaming admittance joint proxy."
    )

    # 实验总时长。默认 16s：前 6s 有虚拟力，后 10s 撤力回位。
    parser.add_argument("--duration", type=float, default=16.0, help="Total experiment duration in seconds.")

    # 控制命令发布频率。30Hz 对 Gazebo + JointTrajectoryController 比较稳。
    parser.add_argument("--rate", type=float, default=30.0, help="Streaming command rate in Hz.")

    # 每个 JointTrajectoryPoint 的 time_from_start。
    parser.add_argument("--command-horizon", type=float, default=0.12, help="JointTrajectory point time_from_start in seconds.")

    # 是否先移动到预备姿态。默认 True。
    parser.add_argument("--prepare", action="store_true", default=True, help="Move to a small bent joint posture first.")
    parser.add_argument("--no-prepare", dest="prepare", action="store_false", help="Skip the initial MoveIt prepare motion.")

    # 自定义预备姿态，必须给 7 个关节角。
    parser.add_argument(
        "--prepare-joints",
        type=float,
        nargs=7,
        default=list(DEFAULT_PREPARE_JOINTS),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
    )

    # 导纳模型 M/D/K。
    parser.add_argument("--mass", type=float, nargs=3, default=[2.0, 2.0, 2.0], metavar=("MX", "MY", "MZ"))
    parser.add_argument("--damping", type=float, nargs=3, default=[35.0, 40.0, 40.0], metavar=("DX", "DY", "DZ"))
    parser.add_argument("--stiffness", type=float, nargs=3, default=[45.0, 80.0, 80.0], metavar=("KX", "KY", "KZ"))

    # 导纳 offset / velocity 限幅。
    parser.add_argument("--max-offset", type=float, nargs=3, default=[0.10, 0.05, 0.04], metavar=("X", "Y", "Z"))
    parser.add_argument("--max-velocity", type=float, nargs=3, default=[0.06, 0.03, 0.025], metavar=("VX", "VY", "VZ"))

    # 关节命令整形参数。
    # alpha 越小越顺但越慢；max_joint_step 是每个控制周期允许的最大关节目标变化。
    parser.add_argument("--joint-smoothing-alpha", type=float, default=0.45, help="Low-pass alpha for target joints. 1.0 disables smoothing.")
    parser.add_argument("--max-joint-step", type=float, default=0.02, help="Max joint target change per control cycle in rad. <=0 disables this limit.")

    # 主实验结束后继续用零外力积分，让导纳 offset 和关节命令自然回到平衡点。
    parser.add_argument("--settle-duration", type=float, default=2.0, help="Extra zero-force settling time after the main duration.")

    # 外部 wrench 超时。如果 0.5s 没收到新 wrench，就认为外力为 0。
    parser.add_argument("--wrench-timeout", type=float, default=0.5)

    # 默认内置虚拟力。当前是 x 方向 8N。
    parser.add_argument("--demo-force-x", type=float, default=8.0)
    parser.add_argument("--demo-force-y", type=float, default=0.0)
    parser.add_argument("--demo-force-z", type=float, default=0.0)
    parser.add_argument("--demo-force-duration", type=float, default=6.0)

    # 使用外部 topic wrench，而不是脚本内置 demo force。
    parser.add_argument("--use-topic-wrench", action="store_true")

    # MoveIt 配置。
    parser.add_argument("--group", default=GROUP_NAME)
    parser.add_argument("--eef-link", default=EEF_LINK)
    parser.add_argument("--reference-frame", default=REFERENCE_FRAME)

    # 预备姿态规划速度 / 加速度比例。
    parser.add_argument("--velocity-scaling", type=float, default=0.10)
    parser.add_argument("--acceleration-scaling", type=float, default=0.10)

    return parser.parse_args(argv)


# =============================================================================
# Layer 6.2：主流程层
# =============================================================================
def main():
    """节点主流程。

    执行顺序：
        1. 解析参数；
        2. 检查 ROS master；
        3. 初始化 MoveIt 和 rospy 节点；
        4. 检查 /joint_states；
        5. 创建 /command publisher，并等待控制器连接；
        6. MoveIt 移动到预备姿态；
        7. 记录 equilibrium_pose 和 equilibrium_joints；
        8. 创建 wrench 输入、导纳模型、观察 topic；
        9. 进入 30Hz 控制循环；
        10. 撤力后继续短暂发布，帮助控制器收敛；
        11. 关闭 MoveIt。
    """
    args = parse_args(sys.argv[1:])

    # 没有 roscore / roslaunch 时，直接提示用户先启动仿真。
    if not rosgraph.is_master_online():
        print("ERROR: ROS master is not running.")
        print("Start Gazebo + MoveIt first:")
        print("  cd /home/ros/ws_rm_75_6f_robot")
        print("  source devel/setup.bash")
        print("  roslaunch rm_75_gazebo arm_75_bringup_moveit.launch")
        return 2

    # MoveIt C++ 后端初始化。
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化 ROS 节点。
    # anonymous=True 防止重复运行时节点名冲突。
    rospy.init_node("sim_06_admittance_streaming_joint_proxy", anonymous=True)

    # 预检查 1：确认仿真已经在发布 7 个关节状态。
    wait_for_joint_state(timeout=3.0)

    # 持续订阅 /joint_states，后续用于诊断 target_joints 和 measured_joints 的误差。
    joint_state_cache = JointStateCache()

    # 创建控制输出 publisher。
    # queue_size=1 表示只保留最新目标，避免控制命令堆积。
    command_pub = rospy.Publisher(COMMAND_TOPIC, JointTrajectory, queue_size=1)

    # 预检查 2：确认 JointTrajectoryController 已经订阅 /command。
    wait_for_command_connection(command_pub, timeout=3.0)

    # 创建 MoveIt group，只用于预备姿态和读取当前位姿/关节角。
    group = MoveGroupCommander(args.group)
    group.set_end_effector_link(args.eef_link)
    group.set_pose_reference_frame(args.reference_frame)
    group.set_planning_time(3.0)
    group.set_num_planning_attempts(5)
    group.set_max_velocity_scaling_factor(args.velocity_scaling)
    group.set_max_acceleration_scaling_factor(args.acceleration_scaling)

    # 等待 publisher / MoveIt 状态稳定。
    rospy.sleep(0.5)

    # 移动到预备姿态。
    if args.prepare:
        move_to_prepare_joints(group, args.prepare_joints)

    # 记录导纳控制的平衡点。
    # equilibrium_pose：用于生成理论 target_pose；
    # equilibrium_joints：用于 joint-proxy 映射的零偏置基准。
    equilibrium_pose = group.get_current_pose(args.eef_link)
    equilibrium_pose.header.frame_id = args.reference_frame
    equilibrium_joints = group.get_current_joint_values()

    # 外部 wrench 输入对象。即使默认不用 topic wrench，也可以创建订阅者，方便后续切换。
    wrench_input = WrenchInput(args.reference_frame, args.wrench_timeout)

    # 构造导纳模型。
    admittance = TranslationalAdmittance(
        mass=args.mass,
        damping=args.damping,
        stiffness=args.stiffness,
        max_offset=args.max_offset,
        max_velocity=args.max_velocity,
    )

    # 观察输出 topic。
    target_pub = rospy.Publisher(TARGET_POSE_TOPIC, PoseStamped, queue_size=5)
    state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=5)

    # 打印实验配置，方便你确认参数是否生效。
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
    print("Joint smoothing:   ", args.joint_smoothing_alpha)
    print("Max joint step:    ", args.max_joint_step)
    print("Settle duration:   ", args.settle_duration)

    # 控制循环计时。
    start_time = rospy.Time.now()
    last_time = start_time
    rate = rospy.Rate(args.rate)
    filtered_target_joints = list(equilibrium_joints)

    # ==============================
    # 主控制循环
    # ==============================
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()

        # 超过实验总时长后退出。
        if elapsed > args.duration:
            break

        # 当前控制周期 dt。
        dt = (now - last_time).to_sec()
        last_time = now

        # 获取外力：
        #   --use-topic-wrench：使用外部发布的 WrenchStamped；
        #   默认：使用内置 demo force。
        if args.use_topic_wrench:
            force = wrench_input.current_force()
        else:
            force = demo_force(args, elapsed)

        # 1. 导纳模型：F -> offset / velocity / acceleration。
        acceleration = admittance.step(force, dt)

        # 2. 生成理论末端目标位姿，仅用于观察。
        target_pose = make_target_pose(equilibrium_pose, admittance.offset, args.reference_frame)

        # 3. joint-proxy：offset -> raw_target_joints。
        raw_target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)

        # 4. 关节命令整形：低通滤波 + 每周期步长限幅。
        # 这是 sim_06 本轮升级的重点，用来减少“卡卡的”目标跳变感。
        smoothed_target_joints = smooth_joint_target(
            filtered_target_joints,
            raw_target_joints,
            args.joint_smoothing_alpha,
        )
        target_joints = limit_joint_step(
            filtered_target_joints,
            smoothed_target_joints,
            args.max_joint_step,
        )
        filtered_target_joints = list(target_joints)

        # 5. 控制输出：持续发布 /arm/arm_joint_controller/command。
        publish_joint_command(command_pub, target_joints, args.command_horizon)

        # 6. 反馈诊断：读取 Gazebo 当前关节角，计算 target-measured 误差。
        measured_joints = joint_state_cache.current_ordered()
        joint_error = None
        if measured_joints is not None:
            joint_error = [
                target_joints[i] - measured_joints[i]
                for i in range(len(target_joints))
            ]

        # 7. 观察输出：发布理论 target_pose 和状态字符串。
        target_pub.publish(target_pose)
        state_pub.publish(String(data=(
            "dt={:.4f} force_N={} offset_m={} velocity_mps={} "
            "raw_target_joints={} target_joints={} measured_joints={} joint_error_max={:.4f}"
        ).format(
            dt,
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt(raw_target_joints),
            fmt(target_joints),
            fmt_optional(measured_joints),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )))

        # 节流日志：每 0.5s 打印一次，避免刷屏。
        rospy.loginfo_throttle(
            0.5,
            "force[N]=%s offset[m]=%s velocity[m/s]=%s acceleration[m/s^2]=%s joint_error_max=%.4f",
            fmt3(force),
            fmt3(admittance.offset),
            fmt3(admittance.velocity),
            fmt3(acceleration),
            max_abs(joint_error) if joint_error is not None else -1.0,
        )

        # 按设定频率休眠。
        rate.sleep()

    # ==============================
    # 收尾阶段
    # ==============================
    # 实验结束时，导纳 offset 可能还没有完全回到 0。
    # 这里继续使用零外力积分一段时间，让 offset / velocity 自然回零，
    # 同时继续经过平滑和步长限幅发布关节命令。
    settle_until = rospy.Time.now() + rospy.Duration(max(0.0, args.settle_duration))
    while not rospy.is_shutdown() and rospy.Time.now() < settle_until:
        now = rospy.Time.now()
        dt = (now - last_time).to_sec()
        last_time = now

        force = [0.0, 0.0, 0.0]
        admittance.step(force, dt)

        raw_target_joints = map_offset_to_joint_target(equilibrium_joints, admittance.offset)
        smoothed_target_joints = smooth_joint_target(
            filtered_target_joints,
            raw_target_joints,
            args.joint_smoothing_alpha,
        )
        target_joints = limit_joint_step(
            filtered_target_joints,
            smoothed_target_joints,
            args.max_joint_step,
        )
        filtered_target_joints = list(target_joints)

        publish_joint_command(command_pub, target_joints, args.command_horizon)
        rate.sleep()

    print("Experiment finished.")
    print("Final offset [m]:  ", fmt3(admittance.offset))
    print("Final velocity [m/s]:", fmt3(admittance.velocity))

    # 关闭 MoveIt commander。
    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
