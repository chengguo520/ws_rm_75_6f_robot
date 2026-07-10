#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F sim_09 实时诊断曲线。

这个节点订阅 /rm75_z_admittance_probe/state，并通过独立 topic 发布 Start/Stop 调参命令。
它不直接发布关节命令；参数仍由 sim_09 控制节点在 30 Hz 周期边界验证和应用。
它把桌面曲面高度、虚拟 TCP 高度、法向力、z 速度和误差放到同一时间轴上，
用来判断“看起来悬空/贴合不好”到底是视觉错觉、z 导纳跟随误差、速度限幅，
还是 x/y 轨迹切换带来的问题。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import re
import sys
import threading
from collections import deque
from datetime import datetime

import rospy
from std_msgs.msg import String


STATE_TOPIC = "/rm75_z_admittance_probe/state"
TUNING_COMMAND_TOPIC = "/rm75_z_admittance_probe/tuning_command"
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_OUTPUT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../..", "study/data/sim09_runs"))

SCALAR_KEYS = [
    "elapsed",
    "run_id",
    "run_elapsed",
    "tool_down_score",
    "tcp_z",
    "base_surface_z",
    "surface_blend",
    "surface_step",
    "surface_z",
    "eq_tcp_z",
    "tcp_eq_err",
    "penetration",
    "fz_virtual_N",
    "desired_fz_N",
    "force_error_N",
    "control_rate_hz",
    "normal_mass",
    "normal_damping",
    "surface_stiffness",
    "surface_damping",
    "max_z_velocity",
    "line_length",
    "line_speed",
    "wave_height",
    "wave_cycles",
    "surface_blend_time",
    "vz_raw",
    "vz_admittance",
    "xy_error",
    "qdot_max",
    "min_singular",
    "joint_error_max",
]

VECTOR_KEYS = [
    "xy_ref",
    "link_xyz",
    "link7_z_axis",
    "bump_center",
    "cartesian_velocity",
]

VECTOR_COLUMNS = [
    ("xy_ref", ["xy_ref_x", "xy_ref_y"]),
    ("link_xyz", ["link_x", "link_y", "link_z"]),
    ("link7_z_axis", ["link7_z_axis_x", "link7_z_axis_y", "link7_z_axis_z"]),
    ("bump_center", ["bump_center_x", "bump_center_y"]),
    ("cartesian_velocity", ["vx_command", "vy_command", "vz_command"]),
]

CSV_FIELDS = (
    ["wall_time", "ros_time", "reset_index", "mode", "run_state", "surface_source", "tcp_limit"]
    + SCALAR_KEYS
    + [column for _key, columns in VECTOR_COLUMNS for column in columns]
    + ["raw_state"]
)

PLOT_GROUPS = [
    (
        "Surface/TCP height",
        "z height (m)",
        0.025,
        [
            ("surface_z", "surface_z", "tab:blue", "-"),
            ("eq_tcp_z", "eq_tcp_z", "tab:green", "--"),
            ("tcp_z", "tcp_z", "tab:orange", "-"),
        ],
    ),
    (
        "Normal force",
        "force (N)",
        4.0,
        [
            ("fz_virtual_N", "fz_virtual_N", "tab:orange", "-"),
            ("desired_fz_N", "desired_fz_N", "tab:green", "--"),
            ("force_error_N", "force_error_N", "tab:red", ":"),
        ],
    ),
    (
        "Z velocity command",
        "velocity (m/s)",
        0.030,
        [
            ("vz_raw", "vz_raw", "0.45", "--"),
            ("vz_admittance", "vz_admittance", "tab:purple", "-"),
        ],
    ),
    (
        "Contact/tracking error",
        "error (m)",
        0.020,
        [
            ("tcp_eq_err", "tcp_eq_err", "tab:red", "-"),
            ("penetration", "penetration", "tab:blue", "--"),
            ("xy_error", "xy_error", "0.35", "-"),
        ],
    ),
]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Live plot for RM75 sim_09 admittance diagnostics.")
    parser.add_argument("--state-topic", default=STATE_TOPIC)
    parser.add_argument("--tuning-topic", default=TUNING_COMMAND_TOPIC)
    parser.add_argument("--window", type=float, default=60.0, help="Visible time window in seconds.")
    parser.add_argument("--refresh-ms", type=int, default=200, help="Plot refresh period in milliseconds.")
    parser.add_argument("--max-samples", type=int, default=6000, help="Maximum buffered state samples.")
    parser.add_argument("--backend", default="", help="Optional matplotlib backend, for example TkAgg or Qt5Agg.")
    parser.add_argument("--save-data", default="true", choices=["true", "false"], help="Automatically save CSV and PNG.")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Root directory for timestamped run folders.")
    parser.add_argument("--initial-normal-mass", type=float, default=1.5)
    parser.add_argument("--initial-normal-damping", type=float, default=65.0)
    parser.add_argument("--initial-desired-force", type=float, default=6.0)
    return parser.parse_args(argv)


def import_matplotlib(backend):
    try:
        import matplotlib
        if backend:
            matplotlib.use(backend)
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        return matplotlib, plt, FuncAnimation
    except (ImportError, ValueError) as exc:
        print("ERROR: matplotlib backend could not be loaded: {}".format(exc))
        print("Install matplotlib with: sudo apt install python3-matplotlib")
        return None, None, None


def extract_float(text, key):
    pattern = r"(?:^|\s){}=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)".format(re.escape(key))
    match = re.search(pattern, text)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def extract_token(text, key):
    match = re.search(r"(?:^|\s){}=([^\s]+)".format(re.escape(key)), text)
    return match.group(1) if match else ""


def extract_vector(text, key):
    match = re.search(r"(?:^|\s){}=\[([^\]]*)\]".format(re.escape(key)), text)
    if not match:
        return []
    values = []
    for part in match.group(1).split(","):
        try:
            values.append(float(part.strip()))
        except ValueError:
            pass
    return values


def is_finite(value):
    return value is not None and math.isfinite(value)


def is_noninteractive_backend(backend):
    """TkAgg/Qt5Agg 也包含字符串 agg，不能用简单的子串判断。"""
    backend_name = str(backend).lower()
    return backend_name in {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"} or (
        backend_name.startswith("module://matplotlib_inline")
    )


def parse_state_text(text, fallback_elapsed):
    sample = {
        "mode": extract_token(text, "mode"),
        "run_state": extract_token(text, "run_state"),
        "surface_source": extract_token(text, "surface_source"),
        "tcp_limit": extract_token(text, "tcp_limit"),
    }
    for key in SCALAR_KEYS:
        sample[key] = extract_float(text, key)
    for key in VECTOR_KEYS:
        sample[key] = extract_vector(text, key)

    if not is_finite(sample.get("elapsed")):
        sample["elapsed"] = fallback_elapsed
    return sample


class StateBuffer(object):
    """ROS 回调线程写入、matplotlib 主线程读取，中间只共享一个短缓冲。"""

    def __init__(self, max_samples):
        self._lock = threading.Lock()
        self._samples = deque(maxlen=max(10, max_samples))
        self._start_wall_time = None
        self._last_elapsed = None
        self._message_count = 0
        self._reset_count = 0

    def append_from_text(self, text):
        now = rospy.Time.now().to_sec()
        if self._start_wall_time is None:
            self._start_wall_time = now
        fallback_elapsed = now - self._start_wall_time
        sample = parse_state_text(text, fallback_elapsed)
        with self._lock:
            elapsed = sample["elapsed"]
            # Gazebo reset或控制节点重启后 elapsed 会回到 0；清空旧数据，避免时间轴回折。
            if is_finite(self._last_elapsed) and elapsed < self._last_elapsed - 0.5:
                self._samples.clear()
                self._reset_count += 1
            self._samples.append(sample)
            self._last_elapsed = elapsed
            self._message_count += 1
            sample["ros_time"] = now
            sample["reset_index"] = self._reset_count
            return sample

    def snapshot(self):
        with self._lock:
            return list(self._samples), self._message_count, self._reset_count


def sample_to_csv_row(sample, raw_state):
    row = {
        "wall_time": datetime.now().isoformat(timespec="milliseconds"),
        "ros_time": sample.get("ros_time"),
        "reset_index": sample.get("reset_index", 0),
        "mode": sample.get("mode", ""),
        "run_state": sample.get("run_state", ""),
        "surface_source": sample.get("surface_source", ""),
        "tcp_limit": sample.get("tcp_limit", ""),
        "raw_state": raw_state,
    }
    for key in SCALAR_KEYS:
        row[key] = sample.get(key)
    for vector_key, columns in VECTOR_COLUMNS:
        values = sample.get(vector_key) or []
        for index, column in enumerate(columns):
            row[column] = values[index] if index < len(values) else None
    return row


class RunRecorder(object):
    """持续写 CSV；即使图窗意外关闭，也只丢失最多约一秒的缓冲数据。"""

    def __init__(self, output_root, enabled):
        self.enabled = enabled
        self.run_dir = ""
        self.csv_path = ""
        self.png_path = ""
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._rows_since_flush = 0
        if not enabled:
            return

        run_name = "sim09_{}_pid{}".format(datetime.now().strftime("%Y%m%d_%H%M%S"), os.getpid())
        self.run_dir = os.path.join(os.path.abspath(os.path.expanduser(output_root)), run_name)
        self.csv_path = os.path.join(self.run_dir, "state.csv")
        self.png_path = os.path.join(self.run_dir, "plot.png")
        try:
            os.makedirs(self.run_dir)
            self._file = open(self.csv_path, "w", newline="")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
            self._file.flush()
        except OSError as exc:
            self.enabled = False
            rospy.logerr("Cannot create sim_09 output directory %s: %s", self.run_dir, exc)

    def append(self, sample, raw_state):
        if not self.enabled or self._writer is None:
            return
        with self._lock:
            self._writer.writerow(sample_to_csv_row(sample, raw_state))
            self._rows_since_flush += 1
            if self._rows_since_flush >= 30:
                self._file.flush()
                self._rows_since_flush = 0

    def close(self):
        with self._lock:
            if self._file is not None:
                self._file.flush()
                self._file.close()
                self._file = None


class LivePlotter(object):
    def __init__(self, args, matplotlib_module, plt_module, animation_class):
        self.args = args
        self.matplotlib = matplotlib_module
        self.plt = plt_module
        self.animation_class = animation_class
        self.buffer = StateBuffer(args.max_samples)
        self.recorder = RunRecorder(args.output_dir, args.save_data == "true")
        self.lines = {}
        self.axes = []
        self.figure = None
        self.animation = None
        self.status_text = None
        self.control_status_text = None
        self.tuning_pub = None
        self.mass_input = None
        self.damping_input = None
        self.force_input = None
        self.start_button = None
        self.stop_button = None
        self.input_error = ""
        self._save_lock = threading.Lock()
        self._outputs_saved = False

    def state_callback(self, msg):
        sample = self.buffer.append_from_text(msg.data)
        self.recorder.append(sample, msg.data)

    def build_figure(self, include_controls=True):
        fig, axes = self.plt.subplots(len(PLOT_GROUPS), 1, sharex=True, figsize=(11.5, 8.5))
        self.figure = fig
        self.axes = list(axes)
        if fig.canvas.manager is not None:
            fig.canvas.manager.set_window_title("RM75 sim_09 admittance live plot")

        for axis, (title, ylabel, _min_span, line_specs) in zip(self.axes, PLOT_GROUPS):
            axis.set_title(title, fontsize=10)
            axis.set_ylabel(ylabel)
            axis.grid(True, color="0.88", linewidth=0.8)
            if "height" not in ylabel:
                axis.axhline(0.0, color="0.70", linewidth=0.8)
            for key, label, color, style in line_specs:
                line, = axis.plot([], [], linestyle=style, color=color, linewidth=1.6, label=label)
                self.lines[key] = line
            axis.legend(loc="upper right", fontsize=8, ncol=min(3, len(line_specs)))

        self.axes[-1].set_xlabel("elapsed time (s)")
        self.status_text = fig.text(0.01, 0.012, "waiting for {}".format(self.args.state_topic), fontsize=9)
        bottom_margin = 0.20 if include_controls else 0.07
        fig.tight_layout(rect=[0.0, bottom_margin, 1.0, 0.98])
        if include_controls:
            self.build_tuning_controls(fig)
        fig.canvas.mpl_connect("close_event", self.handle_close)
        return fig

    def build_tuning_controls(self, fig):
        from matplotlib.widgets import Button, TextBox

        self.mass_input = TextBox(
            fig.add_axes([0.12, 0.130, 0.14, 0.035]),
            "M (kg)",
            initial="{:.1f}".format(self.args.initial_normal_mass),
            color="white",
            hovercolor="#f2f6fa",
        )
        self.damping_input = TextBox(
            fig.add_axes([0.43, 0.130, 0.14, 0.035]),
            "D (N s/m)",
            initial="{:.0f}".format(self.args.initial_normal_damping),
            color="white",
            hovercolor="#f2f6fa",
        )
        self.force_input = TextBox(
            fig.add_axes([0.74, 0.130, 0.14, 0.035]),
            "Fd (N)",
            initial="{:.1f}".format(self.args.initial_desired_force),
            color="white",
            hovercolor="#f2f6fa",
        )

        self.start_button = Button(
            fig.add_axes([0.08, 0.065, 0.14, 0.042]),
            "Apply & Start",
            color="#dcefdc",
            hovercolor="#c5e5c5",
        )
        self.stop_button = Button(
            fig.add_axes([0.24, 0.065, 0.10, 0.042]),
            "Stop",
            color="#f3dddd",
            hovercolor="#e9c4c4",
        )
        self.start_button.on_clicked(self.publish_start)
        self.stop_button.on_clicked(self.publish_stop)
        self.control_status_text = fig.text(0.39, 0.078, "controller: waiting for state", fontsize=9)

    def publish_start(self, _event):
        if self.tuning_pub is None:
            return
        try:
            normal_mass = float(self.mass_input.text.strip())
            normal_damping = float(self.damping_input.text.strip())
            desired_force = float(self.force_input.text.strip())
            if not 0.3 <= normal_mass <= 5.0:
                raise ValueError("M must be 0.3..5.0 kg")
            if not 10.0 <= normal_damping <= 120.0:
                raise ValueError("D must be 10..120 N s/m")
            if not 1.0 <= desired_force <= 15.0:
                raise ValueError("Fd must be 1..15 N")
            if not all(math.isfinite(value) for value in [normal_mass, normal_damping, desired_force]):
                raise ValueError("values must be finite")
        except ValueError as exc:
            self.input_error = str(exc)
            self.control_status_text.set_color("tab:red")
            self.control_status_text.set_text("invalid input: {}".format(self.input_error))
            self.figure.canvas.draw_idle()
            return

        payload = {
            "action": "start",
            "normal_mass": normal_mass,
            "normal_damping": normal_damping,
            "desired_normal_force": desired_force,
        }
        self.input_error = ""
        self.tuning_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.control_status_text.set_color("black")
        self.control_status_text.set_text(
            "command sent: Start  M={:.2f}  D={:.0f}  Fd={:.1f}".format(
                payload["normal_mass"],
                payload["normal_damping"],
                payload["desired_normal_force"],
            )
        )
        self.figure.canvas.draw_idle()

    def publish_stop(self, _event):
        if self.tuning_pub is None:
            return
        self.input_error = ""
        self.tuning_pub.publish(String(data=json.dumps({"action": "stop"})))
        self.control_status_text.set_color("black")
        self.control_status_text.set_text("command sent: Stop")
        self.figure.canvas.draw_idle()

    def handle_close(self, _event):
        self.publish_stop(None)
        self.save_outputs()
        rospy.signal_shutdown("plot window closed")

    def visible_samples(self, samples):
        if not samples:
            return []
        latest_time = samples[-1].get("elapsed") or 0.0
        min_time = max(0.0, latest_time - max(1.0, self.args.window))
        return [sample for sample in samples if (sample.get("elapsed") or 0.0) >= min_time]

    def update_axis_limits(self, axis, samples, line_specs, min_span):
        values = []
        for key, _label, _color, _style in line_specs:
            values.extend(sample.get(key) for sample in samples if is_finite(sample.get(key)))
        if not values:
            return
        y_min = min(values)
        y_max = max(values)
        span = max(y_max - y_min, min_span)
        center = 0.5 * (y_min + y_max)
        axis.set_ylim(center - 0.6 * span, center + 0.6 * span)

    def update_lines(self, _frame_index):
        samples, message_count, reset_count = self.buffer.snapshot()
        visible = self.visible_samples(samples)
        if not visible:
            self.axes[-1].set_xlim(0.0, max(1.0, self.args.window))
            return list(self.lines.values())

        times = [sample.get("elapsed") or 0.0 for sample in visible]
        for _title, _ylabel, _min_span, line_specs in PLOT_GROUPS:
            for key, _label, _color, _style in line_specs:
                line = self.lines[key]
                xs = []
                ys = []
                for time_value, sample in zip(times, visible):
                    value = sample.get(key)
                    if is_finite(value):
                        xs.append(time_value)
                        ys.append(value)
                line.set_data(xs, ys)

        latest = visible[-1]
        latest_time = times[-1]
        received_hz = 0.0
        if len(times) > 1 and times[-1] > times[0]:
            received_hz = float(len(times) - 1) / (times[-1] - times[0])
        min_time = max(0.0, latest_time - max(1.0, self.args.window))
        self.axes[-1].set_xlim(min_time, max(min_time + 1.0, latest_time + 0.5))
        for axis, (_title, _ylabel, min_span, line_specs) in zip(self.axes, PLOT_GROUPS):
            self.update_axis_limits(axis, visible, line_specs, min_span)

        self.status_text.set_text(
            "rx={:.1f}Hz  refresh={:.1f}Hz  samples={}  resets={}  state={}  run={}  src={}  blend={:.2f}  limit={}  "
            "t={:.2f}s  fz={:.2f}N  vz={:.4f}m/s  eq_err={:.4f}m".format(
                received_hz,
                1000.0 / max(50, self.args.refresh_ms),
                message_count,
                reset_count,
                latest.get("run_state") or "?",
                int(latest.get("run_id")) if is_finite(latest.get("run_id")) else -1,
                latest.get("surface_source") or "?",
                latest.get("surface_blend") if is_finite(latest.get("surface_blend")) else float("nan"),
                latest.get("tcp_limit") or "?",
                latest_time,
                latest.get("fz_virtual_N") if is_finite(latest.get("fz_virtual_N")) else float("nan"),
                latest.get("vz_admittance") if is_finite(latest.get("vz_admittance")) else float("nan"),
                latest.get("tcp_eq_err") if is_finite(latest.get("tcp_eq_err")) else float("nan"),
            )
        )
        if self.control_status_text is not None:
            if self.input_error:
                self.control_status_text.set_color("tab:red")
                self.control_status_text.set_text("invalid input: {}".format(self.input_error))
            else:
                self.control_status_text.set_color("black")
                self.control_status_text.set_text(
                    "active: {}  run={}  M={:.2f}  D={:.0f}  Fd={:.1f}N  run_t={:.1f}s".format(
                        latest.get("run_state") or "?",
                        int(latest.get("run_id")) if is_finite(latest.get("run_id")) else -1,
                        latest.get("normal_mass") if is_finite(latest.get("normal_mass")) else float("nan"),
                        latest.get("normal_damping") if is_finite(latest.get("normal_damping")) else float("nan"),
                        latest.get("desired_fz_N") if is_finite(latest.get("desired_fz_N")) else float("nan"),
                        latest.get("run_elapsed") if is_finite(latest.get("run_elapsed")) else float("nan"),
                    )
                )
        return list(self.lines.values())

    def save_outputs(self):
        with self._save_lock:
            if self._outputs_saved:
                return
            self._outputs_saved = True

        samples, message_count, _reset_count = self.buffer.snapshot()
        if self.recorder.enabled and samples and self.figure is not None:
            original_window = self.args.window
            try:
                # 关闭时临时扩展时间窗，让 PNG 包含本轮缓冲中的完整实验曲线。
                first_time = samples[0].get("elapsed") or 0.0
                last_time = samples[-1].get("elapsed") or first_time
                self.args.window = max(1.0, last_time - first_time + 1.0)
                self.update_lines(0)
                self.figure.savefig(self.recorder.png_path, dpi=160)
                rospy.loginfo("Saved sim_09 run: samples=%d csv=%s png=%s", message_count, self.recorder.csv_path, self.recorder.png_path)
            except (OSError, ValueError) as exc:
                rospy.logerr("Failed to save sim_09 plot PNG: %s", exc)
            finally:
                self.args.window = original_window
        self.recorder.close()

    def run(self):
        # 预备动作可能比图窗启动更慢；latch 保证提前点击的最后一条 Start/Stop 不会丢失。
        self.tuning_pub = rospy.Publisher(self.args.tuning_topic, String, queue_size=5, latch=True)
        rospy.Subscriber(self.args.state_topic, String, self.state_callback, queue_size=200)
        backend = self.matplotlib.get_backend()
        rospy.loginfo("sim_09 live plot subscribing to %s with matplotlib backend %s", self.args.state_topic, backend)
        rospy.loginfo("sim_09 tuning controls publishing to %s", self.args.tuning_topic)
        if self.recorder.enabled:
            rospy.loginfo("sim_09 recording CSV/PNG under %s", self.recorder.run_dir)
        headless = is_noninteractive_backend(backend)
        if headless:
            rospy.loginfo("Matplotlib backend is %s; running as a headless CSV/PNG recorder.", backend)

        fig = self.build_figure(include_controls=not headless)
        rospy.on_shutdown(self.save_outputs)
        if headless:
            # 非交互后端的 show() 会立即返回。由 ROS spin 保持订阅，退出时再绘制完整 PNG。
            rospy.spin()
            self.save_outputs()
            return

        # FuncAnimation 对象必须保存在 self 上，否则会被 Python 回收，窗口不再刷新。
        self.animation = self.animation_class(
            fig,
            self.update_lines,
            interval=max(50, self.args.refresh_ms),
            blit=False,
            cache_frame_data=False,
        )
        self.plt.show()
        self.save_outputs()


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    matplotlib_module, plt_module, animation_class = import_matplotlib(args.backend)
    if matplotlib_module is None:
        return 2

    rospy.init_node("sim_09_admittance_live_plot", anonymous=True)
    plotter = LivePlotter(args, matplotlib_module, plt_module, animation_class)
    plotter.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
