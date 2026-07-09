#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import datetime
import math
import os
import sys
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import messagebox, ttk

import moveit_commander
import rospy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_slerp


# =============================================================================
# 1. 工程配置层
# =============================================================================
#
# 简化示教器：
# - 左侧/上方：多关节 jog 面板，调目标关节角
# - 右侧/下方：点位库，保存当前姿态、加载点位、规划、执行
# - 所有执行都走 MoveIt：Plan -> Execute planned
# - 不做滑条实时下发，避免手滑导致机械臂立即动作
GROUP_NAME = "arm"
DEFAULT_EEF_LINK = "link7"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_STORE = "/home/ros/ws_rm_75_6f_robot/study/rm75_6f_waypoints.yaml"
MAX_AXES = 7
DEFAULT_AXES = 7
VERIFY_TOLERANCE_DEG = 1.0
VERIFY_POSITION_TOLERANCE_M = 0.005
VERIFY_ROTATION_TOLERANCE_DEG = 2.0
CARTESIAN_EEF_STEP_M = 0.005
CARTESIAN_JUMP_THRESHOLD = 0.0
MIN_CARTESIAN_FRACTION = 0.95

REAL_SPEED_PRESETS = {
    "Teach 1%": (0.01, 0.01),
    "Slow 3%": (0.03, 0.03),
    "Normal 5%": (0.05, 0.05),
    "Fast 10%": (0.10, 0.08),
    "Auto 20%": (0.20, 0.12),
}

SIM_SPEED_PRESETS = dict(REAL_SPEED_PRESETS)
SIM_SPEED_PRESETS.update(
    {
        "Sim 30%": (0.30, 0.25),
        "Sim 50%": (0.50, 0.40),
        "Sim 70%": (0.70, 0.55),
    }
)

PROFILE_DEFAULTS = {
    "real": {
        "range": 20.0,
        "step_degrees": 1.0,
        "pose_position_range": 0.05,
        "pose_rotation_range": 20.0,
        "position_step_m": 0.01,
        "rotation_step_degrees": 5.0,
        "velocity": 0.05,
        "acceleration": 0.05,
        "max_range": 20.0,
        "max_scaling": 0.20,
        "speed_presets": REAL_SPEED_PRESETS,
    },
    "sim": {
        "range": 90.0,
        "step_degrees": 5.0,
        "pose_position_range": 0.20,
        "pose_rotation_range": 60.0,
        "position_step_m": 0.02,
        "rotation_step_degrees": 10.0,
        "velocity": 0.30,
        "acceleration": 0.25,
        "max_range": 120.0,
        "max_scaling": 0.80,
        "speed_presets": SIM_SPEED_PRESETS,
    },
}


# =============================================================================
# 2. 小工具层
# =============================================================================

def deg(rad_value):
    return math.degrees(rad_value)


def rad(deg_value):
    return math.radians(deg_value)


def fmt_rad(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt_deg(value):
    return "{:.2f}".format(deg(value))


def fmt_m(value):
    return "{:.4f}".format(value)


def now_iso():
    return datetime.datetime.now().replace(microsecond=0).isoformat()


def normalize_plan_result(plan_result):
    if isinstance(plan_result, tuple):
        success = bool(plan_result[0])
        plan = plan_result[1]
        error_code = plan_result[3] if len(plan_result) > 3 else None
        return success, plan, error_code

    return bool(plan_result.joint_trajectory.points), plan_result, None


def error_name(error_code):
    if error_code is None:
        return "unknown"

    values = {
        value: name
        for name, value in MoveItErrorCodes.__dict__.items()
        if name.isupper() and isinstance(value, int)
    }
    return values.get(error_code.val, str(error_code.val))


# =============================================================================
# 3. 点位库层
# =============================================================================

def empty_store():
    return {
        "version": 1,
        "joint_names": list(JOINT_NAMES),
        "waypoints": {},
    }


def load_store(path):
    if not os.path.exists(path):
        return empty_store()

    with open(path, "r") as f:
        data = yaml.safe_load(f) or empty_store()

    data.setdefault("version", 1)
    data.setdefault("joint_names", list(JOINT_NAMES))
    data.setdefault("waypoints", {})
    return data


def save_store(path, data):
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)

    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)


def positions_to_dict(values):
    return {name: float(values[index]) for index, name in enumerate(JOINT_NAMES)}


def positions_from_waypoint(waypoint):
    positions = waypoint.get("positions", {})
    missing = [name for name in JOINT_NAMES if name not in positions]
    if missing:
        raise ValueError("Waypoint missing joints: {}".format(", ".join(missing)))
    return [float(positions[name]) for name in JOINT_NAMES]


def pose_to_dict(pose, eef_link, frame_id):
    xyz_rpy = pose_to_xyz_rpy(pose)
    return {
        "eef_link": eef_link,
        "frame_id": frame_id,
        "xyz_rpy": [float(v) for v in xyz_rpy],
    }


def pose_from_waypoint(waypoint, expected_eef_link):
    pose_data = waypoint.get("tcp_pose")
    if not pose_data:
        raise ValueError("Waypoint has no tcp_pose. Save this point again with the updated teach pendant.")

    eef_link = pose_data.get("eef_link", "")
    if eef_link and eef_link != expected_eef_link:
        raise ValueError(
            "Waypoint eef_link is '{}', current eef_link is '{}'.".format(eef_link, expected_eef_link)
        )

    xyz_rpy = pose_data.get("xyz_rpy")
    if not isinstance(xyz_rpy, list) or len(xyz_rpy) != 6:
        raise ValueError("Waypoint tcp_pose.xyz_rpy must contain 6 values.")
    return xyz_rpy_to_pose([float(v) for v in xyz_rpy])


def pose_to_xyz_rpy(pose):
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    roll, pitch, yaw = euler_from_quaternion(quat)
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll,
        pitch,
        yaw,
    ]


def xyz_rpy_to_pose(values):
    pose = Pose()
    pose.position.x = values[0]
    pose.position.y = values[1]
    pose.position.z = values[2]
    quat = quaternion_from_euler(values[3], values[4], values[5])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
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


def final_joints_from_plan(plan, fallback):
    trajectory = plan.joint_trajectory
    if not trajectory.points:
        return list(fallback)

    result = list(fallback)
    final_positions = trajectory.points[-1].positions
    for name, value in zip(trajectory.joint_names, final_positions):
        if name in JOINT_NAMES:
            result[JOINT_NAMES.index(name)] = value
    return result


def quaternion_dot_abs(a, b):
    return abs(
        a.orientation.x * b.orientation.x
        + a.orientation.y * b.orientation.y
        + a.orientation.z * b.orientation.z
        + a.orientation.w * b.orientation.w
    )


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a, scale):
    return [a[0] * scale, a[1] * scale, a[2] * scale]


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vec_norm(a):
    return math.sqrt(vec_dot(a, a))


def vec_normalize(a):
    norm = vec_norm(a)
    if norm < 1e-9:
        raise ValueError("Cannot normalize a zero-length vector.")
    return vec_scale(a, 1.0 / norm)


def pose_position(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


def set_pose_position(pose, value):
    pose.position.x = value[0]
    pose.position.y = value[1]
    pose.position.z = value[2]


def pose_quaternion(pose):
    return [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]


def set_pose_quaternion(pose, quat):
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]


def make_arc_poses(start_pose, via_pose, end_pose, steps):
    p0 = pose_position(start_pose)
    p1 = pose_position(via_pose)
    p2 = pose_position(end_pose)

    chord = vec_sub(p1, p0)
    chord_len = vec_norm(chord)
    if chord_len < 1e-6:
        raise ValueError("MoveC via point is too close to start point.")

    e1 = vec_normalize(chord)
    normal = vec_cross(vec_sub(p1, p0), vec_sub(p2, p0))
    if vec_norm(normal) < 1e-6:
        raise ValueError("MoveC start/via/end are almost collinear. Use MoveL for a straight line.")
    normal = vec_normalize(normal)
    e2 = vec_cross(normal, e1)

    x2 = vec_dot(vec_sub(p2, p0), e1)
    y2 = vec_dot(vec_sub(p2, p0), e2)
    if abs(y2) < 1e-6:
        raise ValueError("MoveC points are too close to a straight line.")

    cx = chord_len / 2.0
    cy = (x2 * x2 + y2 * y2 - chord_len * x2) / (2.0 * y2)
    center = vec_add(p0, vec_add(vec_scale(e1, cx), vec_scale(e2, cy)))
    radius = math.sqrt(cx * cx + cy * cy)

    def angle_for(point):
        rel = vec_sub(point, center)
        return math.atan2(vec_dot(rel, e2), vec_dot(rel, e1))

    a0 = angle_for(p0)
    a1 = angle_for(p1)
    a2 = angle_for(p2)

    def positive_delta(start, end):
        delta = end - start
        while delta < 0.0:
            delta += 2.0 * math.pi
        while delta >= 2.0 * math.pi:
            delta -= 2.0 * math.pi
        return delta

    ccw_total = positive_delta(a0, a2)
    ccw_via = positive_delta(a0, a1)
    if ccw_via <= ccw_total:
        total = ccw_total
    else:
        total = ccw_total - 2.0 * math.pi

    start_q = pose_quaternion(start_pose)
    end_q = pose_quaternion(end_pose)
    poses = []
    for index in range(1, steps + 1):
        t = float(index) / float(steps)
        angle = a0 + total * t
        point = vec_add(center, vec_add(vec_scale(e1, radius * math.cos(angle)), vec_scale(e2, radius * math.sin(angle))))
        pose = copy_pose(start_pose)
        set_pose_position(pose, point)
        set_pose_quaternion(pose, quaternion_slerp(start_q, end_q, t))
        poses.append(pose)
    return poses


# =============================================================================
# 4. ROS / MoveIt 前置检查层
# =============================================================================

def wait_for_joint_states():
    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
    except rospy.ROSException:
        raise RuntimeError("No /joint_states received. Start rm_driver first.")

    missing = [name for name in JOINT_NAMES if name not in msg.name]
    if missing:
        raise RuntimeError("/joint_states missing joints: {}".format(", ".join(missing)))


def load_joint_limits():
    limits = {}
    robot_description = rospy.get_param("/robot_description", "")
    if not robot_description:
        return limits

    root = ET.fromstring(robot_description)
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if name in JOINT_NAMES and limit is not None:
            lower = limit.attrib.get("lower")
            upper = limit.attrib.get("upper")
            if lower is not None and upper is not None:
                limits[name] = (float(lower), float(upper))
    return limits


# =============================================================================
# 5. 示教器 GUI
# =============================================================================

class TeachPendant(object):
    def __init__(self, root, group, args):
        self.root = root
        self.group = group
        self.args = args
        self.eef_link = args.eef_link or group.get_end_effector_link() or DEFAULT_EEF_LINK
        if self.eef_link:
            self.group.set_end_effector_link(self.eef_link)
        self.visible_joints = JOINT_NAMES[:args.axes]
        self.joint_limits = load_joint_limits()

        self.current = [0.0] * MAX_AXES
        self.synced = [0.0] * MAX_AXES
        self.target = [0.0] * MAX_AXES
        self.pose_target = [0.0] * 6
        self.pose_synced = [0.0] * 6
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None

        self.step_degrees = tk.DoubleVar(value=args.step_degrees)
        self.position_step_m = tk.DoubleVar(value=args.position_step_m)
        self.rotation_step_degrees = tk.DoubleVar(value=args.rotation_step_degrees)
        self.velocity = tk.DoubleVar(value=args.velocity)
        self.acceleration = tk.DoubleVar(value=args.acceleration)
        self.speed_preset = tk.StringVar(value=self.initial_speed_preset(args.velocity, args.acceleration))
        self.status = tk.StringVar(value="Starting...")
        self.name_var = tk.StringVar()
        self.note_var = tk.StringVar()
        self.arc_via_var = tk.StringVar()
        self.arc_end_var = tk.StringVar()

        self.joint_vars = {}
        self.current_labels = {}
        self.target_labels = {}
        self.limit_labels = {}
        self.sliders = {}
        self.pose_vars = {}
        self.pose_current_labels = {}
        self.pose_target_labels = {}
        self.velocity_value_label = None
        self.acceleration_value_label = None
        self.waypoint_list = None

        self.build_ui()
        self.refresh_waypoints()
        self.sync_from_robot()

    # -------------------------------------------------------------------------
    # 5.1 UI 构建
    # -------------------------------------------------------------------------

    def build_ui(self):
        self.root.title("RM75-6F Teach Pendant ({})".format(self.args.profile))
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        top = ttk.Frame(outer)
        top.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        ttk.Label(top, text="Step deg").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            top,
            width=6,
            textvariable=self.step_degrees,
            values=[0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0],
            state="readonly",
        ).grid(row=0, column=1, padx=(4, 12))

        ttk.Label(top, text="Speed preset").grid(row=0, column=2, sticky="w")
        speed_combo = ttk.Combobox(
            top,
            width=12,
            textvariable=self.speed_preset,
            values=list(self.args.speed_presets.keys()) + ["Custom"],
            state="readonly",
        )
        speed_combo.grid(row=0, column=3, padx=(4, 12))
        speed_combo.bind("<<ComboboxSelected>>", self.speed_preset_selected)

        ttk.Label(top, text="Velocity").grid(row=0, column=4, sticky="w")
        ttk.Scale(
            top,
            from_=0.01,
            to=self.args.max_scaling,
            variable=self.velocity,
            length=120,
            command=self.speed_slider_changed,
        ).grid(row=0, column=5, padx=(4, 4))
        self.velocity_value_label = ttk.Label(top, text="0.05", width=5)
        self.velocity_value_label.grid(row=0, column=6, padx=(0, 10))

        ttk.Label(top, text="Accel").grid(row=0, column=7, sticky="w")
        ttk.Scale(
            top,
            from_=0.01,
            to=self.args.max_scaling,
            variable=self.acceleration,
            length=120,
            command=self.speed_slider_changed,
        ).grid(row=0, column=8, padx=(4, 4))
        self.acceleration_value_label = ttk.Label(top, text="0.05", width=5)
        self.acceleration_value_label.grid(row=0, column=9, padx=(0, 10))

        ttk.Button(top, text="Sync current", command=self.sync_from_robot).grid(row=0, column=10, padx=(0, 6))
        ttk.Button(top, text="Stop MoveIt", command=self.stop_motion).grid(row=0, column=11)
        self.update_speed_labels()

        main = ttk.Frame(outer)
        main.grid(row=1, column=0, sticky="nsew")
        outer.rowconfigure(1, weight=1)
        outer.columnconfigure(0, weight=1)

        joints_frame = ttk.LabelFrame(main, text="Joint Jog")
        joints_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        pose_frame = ttk.LabelFrame(main, text="TCP Pose IK")
        pose_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 10), pady=(8, 0))

        waypoint_frame = ttk.LabelFrame(main, text="Waypoints")
        waypoint_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=3)
        main.rowconfigure(1, weight=2)

        self.build_joint_panel(joints_frame)
        self.build_pose_panel(pose_frame)
        self.build_waypoint_panel(waypoint_frame)

        bottom = ttk.Frame(outer)
        bottom.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(bottom, text="Plan MoveJ", command=self.plan_target).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(bottom, text="Plan IK", command=self.plan_pose_target).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(bottom, text="Plan MoveL", command=self.plan_cartesian_line_target).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(bottom, text="Execute planned", command=self.execute_planned).grid(row=0, column=3, padx=(0, 6))
        ttk.Button(bottom, text="Reset target", command=self.reset_target).grid(row=0, column=4, padx=(0, 6))
        ttk.Button(bottom, text="Check feedback", command=self.check_feedback_to_target).grid(row=0, column=5)

        ttk.Label(outer, textvariable=self.status, foreground="#1f4b7a").grid(
            row=3, column=0, sticky="w", pady=(8, 0)
        )

    def build_joint_panel(self, parent):
        ttk.Label(parent, text="Joint", width=8).grid(row=0, column=0)
        ttk.Label(parent, text="Current", width=10).grid(row=0, column=1)
        ttk.Label(parent, text="-").grid(row=0, column=2)
        ttk.Label(parent, text="Target offset", width=32).grid(row=0, column=3)
        ttk.Label(parent, text="+").grid(row=0, column=4)
        ttk.Label(parent, text="Target", width=10).grid(row=0, column=5)
        ttk.Label(parent, text="Limit", width=16).grid(row=0, column=6)

        for row, joint_name in enumerate(self.visible_joints, start=1):
            self.build_joint_row(parent, row, joint_name)

        parent.columnconfigure(3, weight=1)

    def build_pose_panel(self, parent):
        ttk.Label(parent, text="EEF").grid(row=0, column=0, sticky="w")
        ttk.Label(parent, text=self.eef_link, width=10).grid(row=0, column=1, sticky="w")
        ttk.Label(parent, text="XYZ step m").grid(row=0, column=2, sticky="e")
        ttk.Combobox(
            parent,
            width=8,
            textvariable=self.position_step_m,
            values=[0.002, 0.005, 0.01, 0.02, 0.05, 0.10],
            state="readonly",
        ).grid(row=0, column=3, padx=(4, 10))
        ttk.Label(parent, text="RPY step deg").grid(row=0, column=4, sticky="e")
        ttk.Combobox(
            parent,
            width=8,
            textvariable=self.rotation_step_degrees,
            values=[1.0, 2.0, 5.0, 10.0, 20.0, 30.0],
            state="readonly",
        ).grid(row=0, column=5, padx=(4, 0))

        ttk.Label(parent, text="Axis", width=8).grid(row=1, column=0)
        ttk.Label(parent, text="Current", width=10).grid(row=1, column=1)
        ttk.Label(parent, text="-").grid(row=1, column=2)
        ttk.Label(parent, text="Offset", width=24).grid(row=1, column=3)
        ttk.Label(parent, text="+").grid(row=1, column=4)
        ttk.Label(parent, text="Target", width=10).grid(row=1, column=5)

        pose_rows = [
            ("x", "m", self.args.pose_position_range),
            ("y", "m", self.args.pose_position_range),
            ("z", "m", self.args.pose_position_range),
            ("roll", "deg", self.args.pose_rotation_range),
            ("pitch", "deg", self.args.pose_rotation_range),
            ("yaw", "deg", self.args.pose_rotation_range),
        ]
        for row, (axis, unit, slider_range) in enumerate(pose_rows, start=2):
            self.build_pose_row(parent, row, axis, unit, slider_range)

        parent.columnconfigure(3, weight=1)

    def build_pose_row(self, parent, row, axis, unit, slider_range):
        ttk.Label(parent, text=axis, width=8).grid(row=row, column=0)

        current_label = ttk.Label(parent, text="0.0000", width=10)
        current_label.grid(row=row, column=1)
        self.pose_current_labels[axis] = current_label

        ttk.Button(parent, text="-", width=3, command=lambda name=axis: self.step_pose(name, -1)).grid(
            row=row, column=2, padx=(2, 4)
        )

        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(
            parent,
            from_=-slider_range,
            to=slider_range,
            variable=var,
            orient="horizontal",
            length=260,
            command=lambda _value, name=axis: self.pose_slider_changed(name),
        )
        slider.grid(row=row, column=3, sticky="ew")
        self.pose_vars[axis] = var

        ttk.Button(parent, text="+", width=3, command=lambda name=axis: self.step_pose(name, +1)).grid(
            row=row, column=4, padx=(4, 2)
        )

        target_label = ttk.Label(parent, text="0.0000", width=10)
        target_label.grid(row=row, column=5)
        self.pose_target_labels[axis] = target_label

        ttk.Label(parent, text=unit, width=4).grid(row=row, column=6, sticky="w")

    def build_joint_row(self, parent, row, joint_name):
        joint_index = JOINT_NAMES.index(joint_name)
        ttk.Label(parent, text=joint_name, width=8).grid(row=row, column=0)

        current_label = ttk.Label(parent, text="0.00", width=10)
        current_label.grid(row=row, column=1)
        self.current_labels[joint_name] = current_label

        ttk.Button(parent, text="-", width=3, command=lambda idx=joint_index: self.step_joint(idx, -1)).grid(
            row=row, column=2, padx=(2, 4)
        )

        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(
            parent,
            from_=-self.args.range,
            to=self.args.range,
            variable=var,
            orient="horizontal",
            length=320,
            command=lambda _value, idx=joint_index: self.slider_changed(idx),
        )
        slider.grid(row=row, column=3, sticky="ew")
        self.joint_vars[joint_name] = var
        self.sliders[joint_name] = slider

        ttk.Button(parent, text="+", width=3, command=lambda idx=joint_index: self.step_joint(idx, +1)).grid(
            row=row, column=4, padx=(4, 2)
        )

        target_label = ttk.Label(parent, text="0.00", width=10)
        target_label.grid(row=row, column=5)
        self.target_labels[joint_name] = target_label

        limit_label = ttk.Label(parent, text="unknown", width=16)
        limit_label.grid(row=row, column=6)
        self.limit_labels[joint_name] = limit_label

    # -------------------------------------------------------------------------
    # 5.1.1 速度档位
    # -------------------------------------------------------------------------

    def initial_speed_preset(self, velocity, acceleration):
        for name, values in self.args.speed_presets.items():
            preset_velocity, preset_acceleration = values
            if abs(velocity - preset_velocity) < 1e-6 and abs(acceleration - preset_acceleration) < 1e-6:
                return name
        return "Custom"

    def speed_preset_selected(self, _event=None):
        name = self.speed_preset.get()
        if name not in self.args.speed_presets:
            self.update_speed_labels()
            return
        velocity, acceleration = self.args.speed_presets[name]
        self.velocity.set(velocity)
        self.acceleration.set(acceleration)
        self.update_speed_labels()
        self.status.set("Speed preset set to {}: velocity {:.2f}, accel {:.2f}.".format(name, velocity, acceleration))

    def speed_slider_changed(self, _value=None):
        self.speed_preset.set(self.initial_speed_preset(self.velocity.get(), self.acceleration.get()))
        self.update_speed_labels()

    def update_speed_labels(self):
        if self.velocity_value_label is not None:
            self.velocity_value_label.configure(text="{:.2f}".format(self.velocity.get()))
        if self.acceleration_value_label is not None:
            self.acceleration_value_label.configure(text="{:.2f}".format(self.acceleration.get()))

    def build_waypoint_panel(self, parent):
        ttk.Label(parent, text="Name").grid(row=0, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.name_var, width=20).grid(row=0, column=1, sticky="ew", pady=(0, 4))

        ttk.Label(parent, text="Note").grid(row=1, column=0, sticky="w")
        ttk.Entry(parent, textvariable=self.note_var, width=20).grid(row=1, column=1, sticky="ew", pady=(0, 6))

        self.waypoint_list = tk.Listbox(parent, height=10, exportselection=False)
        self.waypoint_list.grid(row=2, column=0, columnspan=2, sticky="nsew")
        self.waypoint_list.bind("<<ListboxSelect>>", self.on_waypoint_select)

        buttons = ttk.Frame(parent)
        buttons.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(buttons, text="Refresh", command=self.refresh_waypoints).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(buttons, text="Save current", command=self.save_current_waypoint).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(buttons, text="Save target", command=self.save_target_waypoint).grid(row=0, column=2)

        buttons2 = ttk.Frame(parent)
        buttons2.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(buttons2, text="Load target", command=self.load_selected_waypoint).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(buttons2, text="Plan MoveJ", command=self.plan_selected_waypoint).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(buttons2, text="Go MoveJ", command=self.go_selected_waypoint).grid(row=0, column=2)

        buttons3 = ttk.Frame(parent)
        buttons3.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(buttons3, text="Plan MoveL", command=self.plan_move_l_selected).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(buttons3, text="Go MoveL", command=self.go_move_l_selected).grid(row=0, column=1)

        arc_frame = ttk.LabelFrame(parent, text="MoveC Arc")
        arc_frame.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Label(arc_frame, text="Via").grid(row=0, column=0, sticky="w")
        ttk.Entry(arc_frame, textvariable=self.arc_via_var, width=16).grid(row=0, column=1, sticky="ew", padx=(4, 4))
        ttk.Button(arc_frame, text="Use selected", command=self.use_selected_as_arc_via).grid(row=0, column=2)
        ttk.Label(arc_frame, text="End").grid(row=1, column=0, sticky="w")
        ttk.Entry(arc_frame, textvariable=self.arc_end_var, width=16).grid(row=1, column=1, sticky="ew", padx=(4, 4), pady=(4, 0))
        ttk.Button(arc_frame, text="Use selected", command=self.use_selected_as_arc_end).grid(row=1, column=2, pady=(4, 0))
        ttk.Button(arc_frame, text="Plan MoveC", command=self.plan_move_c_arc).grid(row=2, column=0, pady=(6, 0))
        ttk.Button(arc_frame, text="Go MoveC", command=self.go_move_c_arc).grid(row=2, column=1, pady=(6, 0), sticky="w")
        arc_frame.columnconfigure(1, weight=1)

        buttons4 = ttk.Frame(parent)
        buttons4.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(buttons4, text="Delete selected", command=self.delete_selected_waypoint).grid(row=0, column=0)

        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(2, weight=1)

    # -------------------------------------------------------------------------
    # 5.2 状态同步与目标编辑
    # -------------------------------------------------------------------------

    def sync_from_robot(self, status_text=None):
        self.current = self.group.get_current_joint_values()
        self.synced = list(self.current)
        self.target = list(self.current)
        self.pose_synced = pose_to_xyz_rpy(self.group.get_current_pose(self.eef_link).pose)
        self.pose_target = list(self.pose_synced)
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None

        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            min_offset, max_offset = self.offset_bounds_for_joint(joint_name, idx)
            self.sliders[joint_name].configure(from_=min_offset, to=max_offset)
            self.joint_vars[joint_name].set(0.0)
            self.current_labels[joint_name].configure(text=fmt_deg(self.current[idx]))
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))
            self.limit_labels[joint_name].configure(text=self.limit_text(joint_name))

        self.refresh_pose_labels(reset_offsets=True)

        if status_text is None:
            status_text = "Synced from robot. Adjust joints, TCP pose, or load a waypoint."
        self.status.set(status_text)

    def reset_target(self):
        self.target = list(self.synced)
        self.pose_target = list(self.pose_synced)
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None
        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            self.joint_vars[joint_name].set(0.0)
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))
        self.refresh_pose_labels(reset_offsets=True)
        self.status.set("Target reset to synced pose.")

    def slider_changed(self, joint_index):
        joint_name = JOINT_NAMES[joint_index]
        min_offset, max_offset = self.offset_bounds_for_joint(joint_name, joint_index)
        offset_degrees = self.clamp(self.joint_vars[joint_name].get(), min_offset, max_offset)
        if abs(offset_degrees - self.joint_vars[joint_name].get()) > 1e-6:
            self.joint_vars[joint_name].set(offset_degrees)

        self.target[joint_index] = self.synced[joint_index] + rad(offset_degrees)
        self.target_labels[joint_name].configure(text=fmt_deg(self.target[joint_index]))
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None

    def step_joint(self, joint_index, direction):
        joint_name = JOINT_NAMES[joint_index]
        var = self.joint_vars[joint_name]
        next_value = var.get() + direction * self.step_degrees.get()
        min_offset, max_offset = self.offset_bounds_for_joint(joint_name, joint_index)
        var.set(self.clamp(next_value, min_offset, max_offset))
        self.slider_changed(joint_index)

    def set_target(self, values, status_text):
        self.target = list(values)
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None
        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            offset = deg(self.target[idx] - self.synced[idx])
            min_offset, max_offset = self.offset_bounds_for_joint(joint_name, idx)
            self.joint_vars[joint_name].set(self.clamp(offset, min_offset, max_offset))
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))
        self.status.set(status_text)

    def pose_axis_index(self, axis):
        return ["x", "y", "z", "roll", "pitch", "yaw"].index(axis)

    def pose_slider_changed(self, axis):
        idx = self.pose_axis_index(axis)
        offset = self.pose_vars[axis].get()
        if idx < 3:
            offset = self.clamp(offset, -self.args.pose_position_range, self.args.pose_position_range)
            self.pose_target[idx] = self.pose_synced[idx] + offset
        else:
            offset = self.clamp(offset, -self.args.pose_rotation_range, self.args.pose_rotation_range)
            self.pose_target[idx] = self.pose_synced[idx] + rad(offset)
        if abs(offset - self.pose_vars[axis].get()) > 1e-9:
            self.pose_vars[axis].set(offset)
        self.refresh_pose_labels(reset_offsets=False)
        self.plan = None
        self.planned_target = None
        self.planned_pose = None
        self.plan_kind = None

    def step_pose(self, axis, direction):
        idx = self.pose_axis_index(axis)
        var = self.pose_vars[axis]
        if idx < 3:
            step = self.position_step_m.get()
            limit = self.args.pose_position_range
        else:
            step = self.rotation_step_degrees.get()
            limit = self.args.pose_rotation_range
        var.set(self.clamp(var.get() + direction * step, -limit, limit))
        self.pose_slider_changed(axis)

    def refresh_pose_labels(self, reset_offsets):
        axis_names = ["x", "y", "z", "roll", "pitch", "yaw"]
        for idx, axis in enumerate(axis_names):
            if reset_offsets:
                self.pose_vars[axis].set(0.0)
            if idx < 3:
                current_text = fmt_m(self.pose_synced[idx])
                target_text = fmt_m(self.pose_target[idx])
            else:
                current_text = fmt_deg(self.pose_synced[idx])
                target_text = fmt_deg(self.pose_target[idx])
            self.pose_current_labels[axis].configure(text=current_text)
            self.pose_target_labels[axis].configure(text=target_text)

    # -------------------------------------------------------------------------
    # 5.3 点位库操作
    # -------------------------------------------------------------------------

    def store_data(self):
        return load_store(self.args.store)

    def save_store_data(self, data):
        save_store(self.args.store, data)

    def refresh_waypoints(self):
        data = self.store_data()
        names = sorted(data.get("waypoints", {}).keys())
        self.waypoint_list.delete(0, tk.END)
        for name in names:
            note = data["waypoints"][name].get("note", "")
            label = "{}  {}".format(name, note)
            self.waypoint_list.insert(tk.END, label)
        self.status.set("Loaded {} waypoint(s).".format(len(names)))

    def selected_waypoint_name(self):
        selection = self.waypoint_list.curselection()
        if not selection:
            messagebox.showwarning("No waypoint", "Please select a waypoint.")
            return None
        label = self.waypoint_list.get(selection[0])
        return label.split()[0]

    def on_waypoint_select(self, _event):
        name = self.selected_waypoint_name()
        if name:
            self.name_var.set(name)

    def save_waypoint(self, values, source_name, tcp_pose=None):
        name = self.name_var.get().strip()
        if not name:
            messagebox.showwarning("Missing name", "Please input a waypoint name.")
            return

        data = self.store_data()
        if name in data["waypoints"]:
            ok = messagebox.askyesno("Overwrite waypoint", "Overwrite waypoint '{}'?".format(name))
            if not ok:
                return

        data["joint_names"] = list(JOINT_NAMES)
        waypoint = {
            "created_at": now_iso(),
            "positions": positions_to_dict(values),
            "note": self.note_var.get().strip(),
        }
        if tcp_pose is not None:
            waypoint["tcp_pose"] = pose_to_dict(tcp_pose, self.eef_link, self.group.get_planning_frame())
        data["waypoints"][name] = waypoint
        self.save_store_data(data)
        self.refresh_waypoints()
        self.status.set("Saved {} waypoint '{}'.".format(source_name, name))

    def save_current_waypoint(self):
        self.current = self.group.get_current_joint_values()
        tcp_pose = self.group.get_current_pose(self.eef_link).pose
        self.save_waypoint(self.current, "current", tcp_pose=tcp_pose)

    def save_target_waypoint(self):
        if self.plan_kind == "ik" and self.planned_target is not None:
            self.save_waypoint(self.planned_target, "planned IK", tcp_pose=self.planned_pose)
            return
        if self.plan_kind in ("movel", "movec") and self.planned_target is not None:
            self.save_waypoint(self.planned_target, self.plan_kind, tcp_pose=self.planned_pose)
            return

        bounds_error = self.bounds_error()
        if bounds_error:
            messagebox.showwarning("Target out of bounds", bounds_error)
            return
        self.save_waypoint(self.target, "target")

    def load_selected_waypoint(self):
        name = self.selected_waypoint_name()
        if not name:
            return
        target = self.target_from_waypoint(name)
        if target is None:
            return
        self.set_target(target, "Loaded waypoint '{}' as target.".format(name))

    def delete_selected_waypoint(self):
        name = self.selected_waypoint_name()
        if not name:
            return
        ok = messagebox.askyesno("Delete waypoint", "Delete waypoint '{}'?".format(name))
        if not ok:
            return
        data = self.store_data()
        data.get("waypoints", {}).pop(name, None)
        self.save_store_data(data)
        self.refresh_waypoints()
        self.status.set("Deleted waypoint '{}'.".format(name))

    def target_from_waypoint(self, name):
        data = self.store_data()
        waypoint = data.get("waypoints", {}).get(name)
        if waypoint is None:
            messagebox.showwarning("Missing waypoint", "Waypoint '{}' not found.".format(name))
            return None
        try:
            return positions_from_waypoint(waypoint)
        except Exception as exc:
            messagebox.showerror("Waypoint error", str(exc))
            return None

    def pose_from_waypoint_name(self, name):
        data = self.store_data()
        waypoint = data.get("waypoints", {}).get(name)
        if waypoint is None:
            messagebox.showwarning("Missing waypoint", "Waypoint '{}' not found.".format(name))
            return None
        try:
            return pose_from_waypoint(waypoint, self.eef_link)
        except Exception as exc:
            messagebox.showerror("TCP waypoint error", str(exc))
            return None

    # -------------------------------------------------------------------------
    # 5.4 MoveIt 规划与执行
    # -------------------------------------------------------------------------

    def apply_scaling(self):
        velocity = self.velocity.get()
        acceleration = self.acceleration.get()
        if not 0.0 < velocity <= self.args.max_scaling:
            raise ValueError("Velocity must be in (0, {:.2f}].".format(self.args.max_scaling))
        if not 0.0 < acceleration <= self.args.max_scaling:
            raise ValueError("Acceleration must be in (0, {:.2f}].".format(self.args.max_scaling))
        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_max_acceleration_scaling_factor(acceleration)
        self.update_speed_labels()

    def plan_selected_waypoint(self):
        name = self.selected_waypoint_name()
        if not name:
            return
        target = self.target_from_waypoint(name)
        if target is None:
            return
        self.set_target(target, "Loaded waypoint '{}' as target.".format(name))
        self.plan_target()

    def go_selected_waypoint(self):
        name = self.selected_waypoint_name()
        if not name:
            return
        target = self.target_from_waypoint(name)
        if target is None:
            return
        self.set_target(target, "Loaded waypoint '{}' as target.".format(name))
        self.plan_target()
        if self.plan is not None:
            self.execute_planned()

    def use_selected_as_arc_via(self):
        name = self.selected_waypoint_name()
        if name:
            self.arc_via_var.set(name)
            self.status.set("MoveC via waypoint set to '{}'.".format(name))

    def use_selected_as_arc_end(self):
        name = self.selected_waypoint_name()
        if name:
            self.arc_end_var.set(name)
            self.status.set("MoveC end waypoint set to '{}'.".format(name))

    def plan_move_l_selected(self):
        name = self.selected_waypoint_name()
        if not name:
            return
        pose = self.pose_from_waypoint_name(name)
        if pose is None:
            return
        self.plan_cartesian_path([pose], "movel", pose, "MoveL waypoint '{}'".format(name))

    def go_move_l_selected(self):
        self.plan_move_l_selected()
        if self.plan is not None:
            self.execute_planned()

    def plan_move_c_arc(self):
        via_name = self.arc_via_var.get().strip()
        end_name = self.arc_end_var.get().strip()
        if not via_name or not end_name:
            messagebox.showwarning("Missing MoveC point", "Please set both MoveC via and end waypoints.")
            return
        via_pose = self.pose_from_waypoint_name(via_name)
        end_pose = self.pose_from_waypoint_name(end_name)
        if via_pose is None or end_pose is None:
            return

        try:
            start_pose = self.group.get_current_pose(self.eef_link).pose
            arc_poses = make_arc_poses(start_pose, via_pose, end_pose, steps=24)
        except Exception as exc:
            self.status.set("MoveC arc error: {}".format(exc))
            messagebox.showerror("MoveC arc error", str(exc))
            return

        self.plan_cartesian_path(
            arc_poses,
            "movec",
            end_pose,
            "MoveC via '{}' end '{}'".format(via_name, end_name),
        )

    def go_move_c_arc(self):
        self.plan_move_c_arc()
        if self.plan is not None:
            self.execute_planned()

    def plan_target(self):
        try:
            self.apply_scaling()
            self.current = self.group.get_current_joint_values()
            bounds_error = self.bounds_error()
            if bounds_error:
                self.plan = None
                self.planned_target = None
                self.status.set(bounds_error)
                messagebox.showwarning("Joint target out of bounds", bounds_error)
                return

            self.group.set_joint_value_target(dict(zip(JOINT_NAMES, self.target)))
            success, plan, error_code = normalize_plan_result(self.group.plan())
            point_count = len(plan.joint_trajectory.points)

            if not success or point_count == 0:
                self.plan = None
                self.planned_target = None
                self.planned_pose = None
                self.plan_kind = None
                self.status.set("Plan failed: {}.".format(error_name(error_code)))
                return

            self.plan = plan
            self.planned_target = list(self.target)
            self.planned_pose = None
            self.plan_kind = "movej"
            self.status.set("MoveJ Plan OK: {} points. Check RViz, then Execute planned.".format(point_count))
            print("MoveJ Plan OK. Target rad:", fmt_rad(self.target))
        except Exception as exc:
            self.plan = None
            self.planned_target = None
            self.planned_pose = None
            self.plan_kind = None
            self.status.set("Plan error: {}".format(exc))
            messagebox.showerror("Plan error", str(exc))

    def plan_pose_target(self):
        try:
            self.apply_scaling()
            self.current = self.group.get_current_joint_values()
            pose = xyz_rpy_to_pose(self.pose_target)
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(pose, self.eef_link)
            success, plan, error_code = normalize_plan_result(self.group.plan())
            self.group.clear_pose_targets()
            point_count = len(plan.joint_trajectory.points)

            if not success or point_count == 0:
                self.plan = None
                self.planned_target = None
                self.planned_pose = None
                self.plan_kind = None
                self.status.set("TCP plan failed: {}.".format(error_name(error_code)))
                return

            self.plan = plan
            self.planned_target = final_joints_from_plan(plan, self.current)
            self.planned_pose = pose
            self.plan_kind = "ik"
            self.show_planned_joint_target()
            self.status.set("IK Plan OK: {} points. Check RViz, then Execute planned.".format(point_count))
            print(
                "IK Plan OK. Target xyz/rpy:",
                "[{:.4f}, {:.4f}, {:.4f}, {:.2f}, {:.2f}, {:.2f}]".format(
                    self.pose_target[0],
                    self.pose_target[1],
                    self.pose_target[2],
                    deg(self.pose_target[3]),
                    deg(self.pose_target[4]),
                    deg(self.pose_target[5]),
                ),
            )
        except Exception as exc:
            self.group.clear_pose_targets()
            self.plan = None
            self.planned_target = None
            self.planned_pose = None
            self.plan_kind = None
            self.status.set("TCP plan error: {}".format(exc))
            messagebox.showerror("TCP plan error", str(exc))

    def plan_cartesian_line_target(self):
        pose = xyz_rpy_to_pose(self.pose_target)
        self.plan_cartesian_path([pose], "movel", pose, "MoveL TCP target")

    def plan_cartesian_path(self, waypoints, plan_kind, final_pose, label):
        try:
            self.apply_scaling()
            self.current = self.group.get_current_joint_values()
            self.group.set_start_state_to_current_state()
            try:
                plan, fraction = self.group.compute_cartesian_path(
                    waypoints,
                    CARTESIAN_EEF_STEP_M,
                    CARTESIAN_JUMP_THRESHOLD,
                    True,
                )
            except TypeError:
                plan, fraction = self.group.compute_cartesian_path(
                    waypoints,
                    CARTESIAN_EEF_STEP_M,
                    CARTESIAN_JUMP_THRESHOLD,
                )

            point_count = len(plan.joint_trajectory.points)
            if point_count == 0 or fraction < MIN_CARTESIAN_FRACTION:
                self.plan = None
                self.planned_target = None
                self.planned_pose = None
                self.plan_kind = None
                self.status.set(
                    "{} failed: cartesian fraction {:.1f}%.".format(label, fraction * 100.0)
                )
                return

            try:
                plan = self.group.retime_trajectory(
                    self.group.get_current_state(),
                    plan,
                    self.velocity.get(),
                    self.acceleration.get(),
                )
            except Exception as exc:
                print("Cartesian retime warning:", exc)

            self.plan = plan
            self.planned_target = final_joints_from_plan(plan, self.current)
            self.planned_pose = final_pose
            self.plan_kind = plan_kind
            self.show_planned_joint_target()
            self.status.set(
                "{} OK: {} points, fraction {:.1f}%. Check RViz, then Execute planned.".format(
                    label,
                    point_count,
                    fraction * 100.0,
                )
            )
            print("{} OK. Cartesian fraction: {:.1f}%".format(label, fraction * 100.0))
        except Exception as exc:
            self.plan = None
            self.planned_target = None
            self.planned_pose = None
            self.plan_kind = None
            self.status.set("{} error: {}".format(label, exc))
            messagebox.showerror("{} error".format(label), str(exc))

    def show_planned_joint_target(self):
        self.target = list(self.planned_target)
        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            offset = deg(self.target[idx] - self.synced[idx])
            min_offset, max_offset = self.offset_bounds_for_joint(joint_name, idx)
            self.joint_vars[joint_name].set(self.clamp(offset, min_offset, max_offset))
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))

    def execute_planned(self):
        if self.plan is None or self.planned_target is None:
            messagebox.showwarning("No plan", "Please click Plan first.")
            return
        if self.plan_kind == "movej" and any(abs(a - b) > 1e-6 for a, b in zip(self.target, self.planned_target)):
            messagebox.showwarning("Target changed", "Target changed after planning. Please Plan again.")
            return
        if self.plan_kind == "ik":
            planned_values = pose_to_xyz_rpy(self.planned_pose)
            if any(abs(a - b) > 1e-6 for a, b in zip(self.pose_target, planned_values)):
                messagebox.showwarning("TCP target changed", "TCP target changed after planning. Please Plan IK again.")
                return

        ok = messagebox.askyesno(
            "Execute real robot",
            "Execute the planned trajectory on the real robot?\n\nKeep the physical E-stop within reach.",
        )
        if not ok:
            return

        try:
            self.apply_scaling()
            self.status.set("Executing planned trajectory...")
            executed = self.group.execute(self.plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            rospy.sleep(1.0)

            feedback_ok, feedback_text = self.feedback_error_text(self.planned_target)
            if self.plan_kind in ("ik", "movel", "movec") and self.planned_pose is not None:
                pose_ok, pose_text = self.pose_feedback_error_text(self.planned_pose)
                feedback_ok = feedback_ok and pose_ok
                feedback_text = "{}. {}".format(feedback_text, pose_text)
            status_text = "Execute result: {}. {}".format(executed, feedback_text)
            print("Execute result:", executed)
            print(feedback_text)
            self.plan = None
            self.planned_target = None
            self.planned_pose = None
            self.plan_kind = None
            self.sync_from_robot(status_text=status_text)
            if not feedback_ok:
                messagebox.showwarning("Feedback warning", feedback_text)
        except Exception as exc:
            self.status.set("Execute error: {}".format(exc))
            messagebox.showerror("Execute error", str(exc))

    # -------------------------------------------------------------------------
    # 5.5 安全与校验
    # -------------------------------------------------------------------------

    def bounds_error(self):
        for joint_name in JOINT_NAMES:
            idx = JOINT_NAMES.index(joint_name)
            if joint_name not in self.joint_limits:
                continue
            lower, upper = self.joint_limits[joint_name]
            value = self.target[idx]
            if value < lower or value > upper:
                return "{} target {:.2f} deg outside URDF limit {:.2f}..{:.2f} deg.".format(
                    joint_name, deg(value), deg(lower), deg(upper)
                )
        return ""

    def feedback_error_text(self, target):
        current = self.group.get_current_joint_values()
        errors = []
        for joint_name in JOINT_NAMES:
            idx = JOINT_NAMES.index(joint_name)
            errors.append((joint_name, abs(current[idx] - target[idx])))
        worst_joint, worst_error = max(errors, key=lambda item: item[1])
        worst_error_deg = deg(worst_error)
        ok = worst_error_deg <= VERIFY_TOLERANCE_DEG
        return ok, "Max feedback error: {} {:.2f} deg".format(worst_joint, worst_error_deg)

    def pose_feedback_error_text(self, target_pose):
        current_pose = self.group.get_current_pose(self.eef_link).pose
        dx = current_pose.position.x - target_pose.position.x
        dy = current_pose.position.y - target_pose.position.y
        dz = current_pose.position.z - target_pose.position.z
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        dot_value = min(1.0, max(-1.0, quaternion_dot_abs(current_pose, target_pose)))
        rotation_error = 2.0 * math.acos(dot_value)
        rotation_error_deg = deg(rotation_error)
        ok = (
            position_error <= VERIFY_POSITION_TOLERANCE_M
            and rotation_error_deg <= VERIFY_ROTATION_TOLERANCE_DEG
        )
        return ok, "TCP error: {:.1f} mm, {:.2f} deg".format(position_error * 1000.0, rotation_error_deg)

    def check_feedback_to_target(self):
        ok, text = self.feedback_error_text(self.target)
        current_pose = self.group.get_current_pose(self.eef_link).pose
        pose_text = pose_to_xyz_rpy(current_pose)
        text = "{}. Current TCP xyz/rpy: {:.4f}, {:.4f}, {:.4f}, {:.1f}, {:.1f}, {:.1f}".format(
            text,
            pose_text[0],
            pose_text[1],
            pose_text[2],
            deg(pose_text[3]),
            deg(pose_text[4]),
            deg(pose_text[5]),
        )
        self.status.set(text)
        if ok:
            messagebox.showinfo("Feedback check", text)
        else:
            messagebox.showwarning("Feedback check", text)

    def stop_motion(self):
        self.group.stop()
        self.status.set("MoveIt stop() sent. Use hardware E-stop for real emergency.")

    def offset_bounds_for_joint(self, joint_name, joint_index):
        panel_min = -self.args.range
        panel_max = self.args.range
        if joint_name not in self.joint_limits:
            return panel_min, panel_max
        lower, upper = self.joint_limits[joint_name]
        lower_offset = deg(lower - self.synced[joint_index])
        upper_offset = deg(upper - self.synced[joint_index])
        min_offset = max(panel_min, lower_offset)
        max_offset = min(panel_max, upper_offset)
        if min_offset > max_offset:
            return 0.0, 0.0
        return min_offset, max_offset

    def limit_text(self, joint_name):
        if joint_name not in self.joint_limits:
            return "unknown"
        lower, upper = self.joint_limits[joint_name]
        return "{:.1f}..{:.1f}".format(deg(lower), deg(upper))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def on_close(self):
        try:
            self.group.stop()
            self.group.clear_pose_targets()
        finally:
            moveit_commander.roscpp_shutdown()
            self.root.destroy()


# =============================================================================
# 6. 主流程
# =============================================================================

def parse_args(argv):
    parser = argparse.ArgumentParser(description="RM75-6F integrated teach pendant.")
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_DEFAULTS.keys()),
        default="real",
        help="real keeps conservative real-robot limits; sim uses wider and faster Gazebo-friendly defaults.",
    )
    parser.add_argument("--axes", type=int, choices=[6, 7], default=DEFAULT_AXES)
    parser.add_argument("--range", type=float, default=None)
    parser.add_argument("--step-degrees", type=float, default=None)
    parser.add_argument("--pose-position-range", type=float, default=None)
    parser.add_argument("--pose-rotation-range", type=float, default=None)
    parser.add_argument("--position-step-m", type=float, default=None)
    parser.add_argument("--rotation-step-degrees", type=float, default=None)
    parser.add_argument("--velocity", type=float, default=None)
    parser.add_argument("--acceleration", type=float, default=None)
    parser.add_argument("--store", default=DEFAULT_STORE)
    parser.add_argument("--eef-link", default=DEFAULT_EEF_LINK)
    args = parser.parse_args(argv)
    apply_profile_defaults(args)
    return args


def apply_profile_defaults(args):
    config = PROFILE_DEFAULTS[args.profile]
    if args.range is None:
        args.range = config["range"]
    if args.step_degrees is None:
        args.step_degrees = config["step_degrees"]
    if args.pose_position_range is None:
        args.pose_position_range = config["pose_position_range"]
    if args.pose_rotation_range is None:
        args.pose_rotation_range = config["pose_rotation_range"]
    if args.position_step_m is None:
        args.position_step_m = config["position_step_m"]
    if args.rotation_step_degrees is None:
        args.rotation_step_degrees = config["rotation_step_degrees"]
    if args.velocity is None:
        args.velocity = config["velocity"]
    if args.acceleration is None:
        args.acceleration = config["acceleration"]

    # 仿真允许更大的角度/速度，实物 profile 仍保留原来的保守上限。
    args.max_range = config["max_range"]
    args.max_scaling = config["max_scaling"]
    args.speed_presets = config["speed_presets"]


def validate_args(args):
    if not 0.0 < args.range <= args.max_range:
        print("Refusing to run: --range must be in (0, {:.1f}] for {} profile.".format(args.max_range, args.profile))
        return 2
    if not 0.0 < args.step_degrees <= args.range:
        print("Refusing to run: --step-degrees must be in (0, --range].")
        return 2
    if not 0.0 < args.pose_position_range <= 0.50:
        print("Refusing to run: --pose-position-range must be in (0, 0.50].")
        return 2
    if not 0.0 < args.pose_rotation_range <= 180.0:
        print("Refusing to run: --pose-rotation-range must be in (0, 180].")
        return 2
    if not 0.0 < args.position_step_m <= args.pose_position_range:
        print("Refusing to run: --position-step-m must be in (0, --pose-position-range].")
        return 2
    if not 0.0 < args.rotation_step_degrees <= args.pose_rotation_range:
        print("Refusing to run: --rotation-step-degrees must be in (0, --pose-rotation-range].")
        return 2
    if not 0.0 < args.velocity <= args.max_scaling:
        print(
            "Refusing to run: --velocity must be in (0, {:.2f}] for {} profile.".format(
                args.max_scaling, args.profile
            )
        )
        return 2
    if not 0.0 < args.acceleration <= args.max_scaling:
        print(
            "Refusing to run: --acceleration must be in (0, {:.2f}] for {} profile.".format(
                args.max_scaling, args.profile
            )
        )
        return 2
    return 0


def main():
    args = parse_args(sys.argv[1:])
    ret = validate_args(args)
    if ret != 0:
        return ret

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("real_teach_pendant", anonymous=True)
    wait_for_joint_states()

    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(8.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)
    rospy.sleep(1.0)

    root = tk.Tk()
    TeachPendant(root, group, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
