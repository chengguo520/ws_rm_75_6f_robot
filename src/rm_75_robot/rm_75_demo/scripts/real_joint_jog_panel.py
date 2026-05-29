#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import math
import sys
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import messagebox, ttk

import moveit_commander
import rospy
from moveit_msgs.msg import MoveItErrorCodes


# =========================
# 1. 全局配置层
# =========================
#
# 这是一个“简化上位机式”的实物关节调试面板：
# - 读取当前真实关节角
# - 用按钮/滑条调整目标关节角
# - 先 Plan，再 Execute planned
# - 不做滑条实时下发，避免手滑时机械臂立刻运动
GROUP_NAME = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
DEFAULT_AXES = 7
MAX_AXES = 7
MAX_PANEL_DEGREES = 20.0
MAX_SCALING = 0.20
VERIFY_TOLERANCE_DEG = 1.0


# =========================
# 2. MoveIt 兼容与格式化层
# =========================

def deg(rad):
    return math.degrees(rad)


def rad(degrees):
    return math.radians(degrees)


def fmt_rad(values):
    return "[" + ", ".join("{:.4f}".format(v) for v in values) + "]"


def fmt_deg(value):
    return "{:.2f}".format(deg(value))


def normalize_plan_result(plan_result):
    """兼容不同 MoveIt 版本 group.plan() 的返回格式。"""
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


def load_joint_limits():
    """从 ROS 参数 /robot_description 里读取 URDF 关节上下限。"""
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


# =========================
# 3. 参数与安全保护层
# =========================

def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Small real-robot joint jog panel for RM75-6F."
    )
    parser.add_argument(
        "--axes",
        type=int,
        choices=[6, 7],
        default=DEFAULT_AXES,
        help="How many joints to show. Use --axes 6 if you only want the first 6 axes.",
    )
    parser.add_argument(
        "--range",
        type=float,
        default=MAX_PANEL_DEGREES,
        help="Allowed target range around the synced pose, in degrees.",
    )
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.05)
    return parser.parse_args(argv)


def validate_args(args):
    if not 0.0 < args.range <= MAX_PANEL_DEGREES:
        print("Refusing to run: --range must be in (0, {:.1f}].".format(MAX_PANEL_DEGREES))
        return 2

    if not 0.0 < args.velocity <= MAX_SCALING:
        print("Refusing to run: --velocity must be in (0, {:.2f}].".format(MAX_SCALING))
        return 2

    if not 0.0 < args.acceleration <= MAX_SCALING:
        print("Refusing to run: --acceleration must be in (0, {:.2f}].".format(MAX_SCALING))
        return 2

    return 0


# =========================
# 4. GUI 主类
# =========================

class JointJogPanel(object):
    def __init__(self, root, group, args):
        self.root = root
        self.group = group
        self.args = args
        self.visible_joints = JOINT_NAMES[:args.axes]
        self.joint_limits = load_joint_limits()

        self.current = [0.0] * MAX_AXES
        self.synced = [0.0] * MAX_AXES
        self.target = [0.0] * MAX_AXES
        self.planned_target = None
        self.plan = None

        self.step_degrees = tk.DoubleVar(value=1.0)
        self.velocity = tk.DoubleVar(value=args.velocity)
        self.acceleration = tk.DoubleVar(value=args.acceleration)
        self.status = tk.StringVar(value="Starting...")

        self.joint_vars = {}
        self.current_labels = {}
        self.target_labels = {}
        self.limit_labels = {}
        self.sliders = {}

        self.build_ui()
        self.sync_from_robot()

    # -------------------------
    # 4.1 界面搭建
    # -------------------------

    def build_ui(self):
        self.root.title("RM75-6F Joint Jog Panel")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        outer = ttk.Frame(self.root, padding=10)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        control = ttk.Frame(outer)
        control.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        ttk.Label(control, text="Step deg").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            control,
            width=6,
            textvariable=self.step_degrees,
            values=[0.5, 1.0, 2.0, 5.0, 10.0],
            state="readonly",
        ).grid(row=0, column=1, padx=(4, 14))

        ttk.Label(control, text="Velocity").grid(row=0, column=2, sticky="w")
        ttk.Scale(
            control,
            from_=0.01,
            to=MAX_SCALING,
            variable=self.velocity,
            orient="horizontal",
            length=120,
        ).grid(row=0, column=3, padx=(4, 14))

        ttk.Label(control, text="Accel").grid(row=0, column=4, sticky="w")
        ttk.Scale(
            control,
            from_=0.01,
            to=MAX_SCALING,
            variable=self.acceleration,
            orient="horizontal",
            length=120,
        ).grid(row=0, column=5, padx=(4, 14))

        ttk.Button(control, text="Sync current", command=self.sync_from_robot).grid(
            row=0, column=6, padx=(0, 6)
        )
        ttk.Button(control, text="Stop MoveIt", command=self.stop_motion).grid(
            row=0, column=7
        )

        header = ttk.Frame(outer)
        header.grid(row=1, column=0, sticky="ew")
        ttk.Label(header, text="Joint", width=8).grid(row=0, column=0)
        ttk.Label(header, text="Current deg", width=12).grid(row=0, column=1)
        ttk.Label(header, text="-").grid(row=0, column=2)
        ttk.Label(header, text="Target", width=40).grid(row=0, column=3)
        ttk.Label(header, text="+").grid(row=0, column=4)
        ttk.Label(header, text="Target deg", width=12).grid(row=0, column=5)
        ttk.Label(header, text="URDF limit deg", width=18).grid(row=0, column=6)

        body = ttk.Frame(outer)
        body.grid(row=2, column=0, sticky="nsew")

        for row, joint_name in enumerate(self.visible_joints):
            self.build_joint_row(body, row, joint_name)

        action = ttk.Frame(outer)
        action.grid(row=3, column=0, sticky="ew", pady=(10, 6))
        ttk.Button(action, text="Plan", command=self.plan_target).grid(
            row=0, column=0, padx=(0, 6)
        )
        ttk.Button(action, text="Execute planned", command=self.execute_planned).grid(
            row=0, column=1, padx=(0, 6)
        )
        ttk.Button(action, text="Reset target", command=self.reset_target).grid(
            row=0, column=2
        )
        ttk.Button(action, text="Check feedback", command=self.check_feedback_to_target).grid(
            row=0, column=3, padx=(6, 0)
        )

        ttk.Label(outer, textvariable=self.status, foreground="#1f4b7a").grid(
            row=4, column=0, sticky="w"
        )

    def build_joint_row(self, parent, row, joint_name):
        joint_index = JOINT_NAMES.index(joint_name)

        ttk.Label(parent, text=joint_name, width=8).grid(row=row, column=0, sticky="w")

        current_label = ttk.Label(parent, text="0.00", width=12)
        current_label.grid(row=row, column=1)
        self.current_labels[joint_name] = current_label

        ttk.Button(
            parent,
            text="-",
            width=3,
            command=lambda idx=joint_index: self.step_joint(idx, -1),
        ).grid(row=row, column=2, padx=(2, 4))

        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(
            parent,
            from_=-self.args.range,
            to=self.args.range,
            variable=var,
            orient="horizontal",
            length=360,
            command=lambda _value, idx=joint_index: self.slider_changed(idx),
        )
        slider.grid(row=row, column=3, sticky="ew")
        self.joint_vars[joint_name] = var
        self.sliders[joint_name] = slider

        ttk.Button(
            parent,
            text="+",
            width=3,
            command=lambda idx=joint_index: self.step_joint(idx, +1),
        ).grid(row=row, column=4, padx=(4, 2))

        target_label = ttk.Label(parent, text="0.00", width=12)
        target_label.grid(row=row, column=5)
        self.target_labels[joint_name] = target_label

        limit_label = ttk.Label(parent, text="unknown", width=18)
        limit_label.grid(row=row, column=6)
        self.limit_labels[joint_name] = limit_label

        parent.columnconfigure(3, weight=1)

    # -------------------------
    # 4.2 状态同步与目标更新
    # -------------------------

    def sync_from_robot(self, status_text=None):
        """从 /joint_states 对应的 MoveIt 当前状态读取真实关节角。"""
        self.current = self.group.get_current_joint_values()
        self.synced = list(self.current)
        self.target = list(self.current)
        self.plan = None
        self.planned_target = None

        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            min_offset, max_offset = self.offset_bounds_for_joint(joint_name, idx)
            self.sliders[joint_name].configure(from_=min_offset, to=max_offset)
            self.joint_vars[joint_name].set(0.0)
            self.current_labels[joint_name].configure(text=fmt_deg(self.current[idx]))
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))
            self.limit_labels[joint_name].configure(text=self.limit_text(joint_name))

        if status_text is None:
            limit_note = "URDF limits loaded." if self.joint_limits else "URDF limits unavailable."
            status_text = "Synced from robot. {} Adjust target, then Plan.".format(limit_note)
        self.status.set(status_text)

    def reset_target(self):
        self.target = list(self.synced)
        self.plan = None
        self.planned_target = None

        for joint_name in self.visible_joints:
            self.joint_vars[joint_name].set(0.0)
            idx = JOINT_NAMES.index(joint_name)
            self.target_labels[joint_name].configure(text=fmt_deg(self.target[idx]))

        self.status.set("Target reset to synced pose.")

    def slider_changed(self, joint_index):
        joint_name = JOINT_NAMES[joint_index]
        min_offset, max_offset = self.offset_bounds_for_joint(joint_name, joint_index)
        offset_degrees = self.clamp(self.joint_vars[joint_name].get(), min_offset, max_offset)
        if abs(offset_degrees - self.joint_vars[joint_name].get()) > 1e-6:
            self.joint_vars[joint_name].set(offset_degrees)
        self.target[joint_index] = self.synced[joint_index] + rad(offset_degrees)
        self.target_labels[joint_name].configure(text=fmt_deg(self.target[joint_index]))

        # 目标一变，旧计划就不能再执行。
        self.plan = None
        self.planned_target = None

    def step_joint(self, joint_index, direction):
        joint_name = JOINT_NAMES[joint_index]
        var = self.joint_vars[joint_name]
        next_value = var.get() + direction * self.step_degrees.get()
        min_offset, max_offset = self.offset_bounds_for_joint(joint_name, joint_index)
        next_value = self.clamp(next_value, min_offset, max_offset)
        var.set(next_value)
        self.slider_changed(joint_index)

    def offset_bounds_for_joint(self, joint_name, joint_index):
        """计算当前同步姿态附近，该关节允许调节的滑条范围。

        面板范围：
            synced +/- --range

        URDF 范围：
            lower <= target <= upper

        最终取两者交集，这样滑条本身不会拖出 MoveIt 关节上/下限。
        """
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
            # 当前姿态已经离 URDF 有效区间超过面板范围。滑条锁在 0，
            # 由 bounds_error() 给出明确提示，避免生成不可执行目标。
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

    # -------------------------
    # 4.3 MoveIt 规划与执行
    # -------------------------

    def apply_scaling(self):
        velocity = self.velocity.get()
        acceleration = self.acceleration.get()

        if not 0.0 < velocity <= MAX_SCALING:
            raise ValueError("Velocity must be in (0, {:.2f}].".format(MAX_SCALING))

        if not 0.0 < acceleration <= MAX_SCALING:
            raise ValueError("Acceleration must be in (0, {:.2f}].".format(MAX_SCALING))

        self.group.set_max_velocity_scaling_factor(velocity)
        self.group.set_max_acceleration_scaling_factor(acceleration)

    def plan_target(self):
        try:
            self.apply_scaling()
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
                self.status.set("Plan failed: {}.".format(error_name(error_code)))
                return

            self.plan = plan
            self.planned_target = list(self.target)
            self.status.set(
                "Plan OK: {} points. Check robot path, then Execute planned.".format(
                    point_count
                )
            )
            print("Plan OK. Target rad:", fmt_rad(self.target))

        except Exception as exc:
            self.plan = None
            self.planned_target = None
            self.status.set("Plan error: {}".format(exc))
            messagebox.showerror("Plan error", str(exc))

    def bounds_error(self):
        for joint_name in JOINT_NAMES:
            idx = JOINT_NAMES.index(joint_name)
            if joint_name not in self.joint_limits:
                continue
            lower, upper = self.joint_limits[joint_name]
            value = self.target[idx]
            if value < lower or value > upper:
                return (
                    "{} target {:.2f} deg is outside URDF limit {:.2f}..{:.2f} deg."
                    .format(joint_name, deg(value), deg(lower), deg(upper))
                )
        return ""

    def feedback_error_text(self, target):
        current = self.group.get_current_joint_values()
        errors = []
        for joint_name in self.visible_joints:
            idx = JOINT_NAMES.index(joint_name)
            errors.append((joint_name, abs(current[idx] - target[idx])))

        if not errors:
            return True, "No visible joints to check."

        worst_joint, worst_error = max(errors, key=lambda item: item[1])
        worst_error_deg = deg(worst_error)
        ok = worst_error_deg <= VERIFY_TOLERANCE_DEG
        text = "Max feedback error: {} {:.2f} deg".format(worst_joint, worst_error_deg)
        return ok, text

    def check_feedback_to_target(self):
        ok, text = self.feedback_error_text(self.target)
        self.status.set(text)
        if ok:
            messagebox.showinfo("Feedback check", text)
        else:
            messagebox.showwarning("Feedback check", text)

    def execute_planned(self):
        if self.plan is None or self.planned_target is None:
            messagebox.showwarning("No plan", "Please click Plan before Execute planned.")
            return

        if any(abs(a - b) > 1e-6 for a, b in zip(self.target, self.planned_target)):
            messagebox.showwarning(
                "Target changed",
                "Target changed after planning. Please click Plan again.",
            )
            return

        ok = messagebox.askyesno(
            "Execute real robot",
            "Execute the planned trajectory on the real robot?\n\n"
            "Keep the physical emergency stop within reach.",
        )
        if not ok:
            return

        try:
            self.apply_scaling()
            self.status.set("Executing planned trajectory...")
            executed = self.group.execute(self.plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            self.current = self.group.get_current_joint_values()
            for joint_name in self.visible_joints:
                idx = JOINT_NAMES.index(joint_name)
                self.current_labels[joint_name].configure(text=fmt_deg(self.current[idx]))

            feedback_ok, feedback_text = self.feedback_error_text(self.planned_target)
            self.plan = None
            self.planned_target = None
            status_text = "Execute result: {}. {}".format(executed, feedback_text)
            print("Execute result:", executed)
            print("After joints rad:", fmt_rad(self.current))
            print(feedback_text)

            # 执行后立刻以真实反馈作为新的调节基准，避免下一次还围绕旧同步姿态调。
            self.sync_from_robot(status_text=status_text)
            if not feedback_ok:
                messagebox.showwarning("Feedback warning", feedback_text)

        except Exception as exc:
            self.status.set("Execute error: {}".format(exc))
            messagebox.showerror("Execute error", str(exc))

    def stop_motion(self):
        self.group.stop()
        self.status.set("MoveIt stop() sent. Use hardware E-stop for real emergency.")

    def on_close(self):
        try:
            self.group.stop()
            self.group.clear_pose_targets()
        finally:
            moveit_commander.roscpp_shutdown()
            self.root.destroy()


# =========================
# 5. 主流程层
# =========================

def main():
    args = parse_args(sys.argv[1:])
    ret = validate_args(args)
    if ret != 0:
        return ret

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("real_joint_jog_panel", anonymous=True)

    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    group.set_planning_time(8.0)
    group.set_num_planning_attempts(10)
    group.set_max_velocity_scaling_factor(args.velocity)
    group.set_max_acceleration_scaling_factor(args.acceleration)
    rospy.sleep(1.0)

    root = tk.Tk()
    JointJogPanel(root, group, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
