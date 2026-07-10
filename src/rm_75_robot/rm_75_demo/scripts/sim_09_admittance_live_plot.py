#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-6F sim_09 实时诊断曲线。

这个节点只订阅 /rm75_z_admittance_probe/state，不发布控制命令。
它把桌面曲面高度、虚拟 TCP 高度、法向力、z 速度和误差放到同一时间轴上，
用来判断“看起来悬空/贴合不好”到底是视觉错觉、z 导纳跟随误差、速度限幅，
还是 x/y 轨迹切换带来的问题。
"""

from __future__ import print_function

import argparse
import math
import re
import sys
import threading
from collections import deque

import rospy
from std_msgs.msg import String


STATE_TOPIC = "/rm75_z_admittance_probe/state"

SCALAR_KEYS = [
    "elapsed",
    "tool_down_score",
    "tcp_z",
    "base_surface_z",
    "surface_step",
    "surface_z",
    "eq_tcp_z",
    "tcp_eq_err",
    "penetration",
    "fz_virtual_N",
    "desired_fz_N",
    "force_error_N",
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
    parser.add_argument("--window", type=float, default=60.0, help="Visible time window in seconds.")
    parser.add_argument("--refresh-ms", type=int, default=200, help="Plot refresh period in milliseconds.")
    parser.add_argument("--max-samples", type=int, default=6000, help="Maximum buffered state samples.")
    parser.add_argument("--backend", default="", help="Optional matplotlib backend, for example TkAgg or Qt5Agg.")
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

    def snapshot(self):
        with self._lock:
            return list(self._samples), self._message_count, self._reset_count


class LivePlotter(object):
    def __init__(self, args, matplotlib_module, plt_module, animation_class):
        self.args = args
        self.matplotlib = matplotlib_module
        self.plt = plt_module
        self.animation_class = animation_class
        self.buffer = StateBuffer(args.max_samples)
        self.lines = {}
        self.axes = []
        self.animation = None
        self.status_text = None

    def state_callback(self, msg):
        self.buffer.append_from_text(msg.data)

    def build_figure(self):
        fig, axes = self.plt.subplots(len(PLOT_GROUPS), 1, sharex=True, figsize=(11.5, 8.5))
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
        fig.tight_layout(rect=[0.0, 0.035, 1.0, 0.98])
        fig.canvas.mpl_connect("close_event", lambda _event: rospy.signal_shutdown("plot window closed"))
        return fig

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
            "topic={}  rx={:.1f}Hz  refresh={:.1f}Hz  samples={}  resets={}  mode={}  src={}  limit={}  "
            "t={:.2f}s  fz={:.2f}N  vz={:.4f}m/s  eq_err={:.4f}m".format(
                self.args.state_topic,
                received_hz,
                1000.0 / max(50, self.args.refresh_ms),
                message_count,
                reset_count,
                latest.get("mode") or "?",
                latest.get("surface_source") or "?",
                latest.get("tcp_limit") or "?",
                latest_time,
                latest.get("fz_virtual_N") if is_finite(latest.get("fz_virtual_N")) else float("nan"),
                latest.get("vz_admittance") if is_finite(latest.get("vz_admittance")) else float("nan"),
                latest.get("tcp_eq_err") if is_finite(latest.get("tcp_eq_err")) else float("nan"),
            )
        )
        return list(self.lines.values())

    def run(self):
        rospy.Subscriber(self.args.state_topic, String, self.state_callback, queue_size=200)
        backend = self.matplotlib.get_backend()
        rospy.loginfo("sim_09 live plot subscribing to %s with matplotlib backend %s", self.args.state_topic, backend)
        if is_noninteractive_backend(backend):
            rospy.logwarn("Matplotlib backend is %s; a live GUI window may not open. Check DISPLAY or pass --backend TkAgg.", backend)

        fig = self.build_figure()
        # FuncAnimation 对象必须保存在 self 上，否则会被 Python 回收，窗口不再刷新。
        self.animation = self.animation_class(
            fig,
            self.update_lines,
            interval=max(50, self.args.refresh_ms),
            blit=False,
            cache_frame_data=False,
        )
        self.plt.show()


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
