#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 sim_09 CSV 数据质量，并按 run_id 汇总柔顺控制指标。"""

from __future__ import print_function

import argparse
import csv
import math
import os
import sys
from collections import defaultdict


REQUIRED_FLOATS = [
    "run_id", "run_elapsed", "normal_mass", "normal_damping", "desired_fz_N",
    "wave_height", "line_speed",
    "fz_virtual_N", "force_error_N", "tcp_eq_err", "vz_admittance",
    "max_z_velocity", "xy_error", "min_singular", "joint_error_max",
    "surface_step", "link_x",
]

OPTIONAL_FLOATS = [
    "x_error",
    "y_error",
    "max_xy_velocity",
]

SUMMARY_FIELDS = [
    "run_id", "M_kg", "D_Ns_m", "Fd_N", "wave_height_m", "line_speed_m_s", "max_xy_velocity_m_s",
    "samples", "duration_s", "sample_hz",
    "missing_values", "duplicate_times", "time_reversals", "parameter_changes", "quality",
    "force_mean_N", "force_bias_N", "force_rms_N", "force_peak_steady_N",
    "force_peak_all_N", "eq_rms_mm", "eq_max_mm", "vz_peak_m_s", "vz_sat_pct",
    "x_rms_mm", "x_max_mm", "y_rms_mm", "y_max_mm", "xy_rms_mm", "xy_max_mm",
    "fz_min_N", "contact_loss_pct", "x_span_mm",
    "surface_span_mm", "min_singular", "joint_error_max_rad", "acceptance",
]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Analyze RM75 sim_09 tuning CSV files.")
    parser.add_argument(
        "csv_paths",
        nargs="+",
        help="One or more state.csv files; append @RUN_ID to select one run from a multi-run CSV.",
    )
    parser.add_argument("--steady-after", type=float, default=5.0, help="Ignore this initial transient for RMS metrics.")
    parser.add_argument("--no-write", action="store_true", help="Do not write summary.csv next to each input file.")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Write comparison.png only when at least two runs are selected.",
    )
    parser.add_argument(
        "--combined-output",
        default="",
        help="Write all input runs to one CSV and a PNG with the same filename stem.",
    )
    return parser.parse_args(argv)


def finite_float(text):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def parse_path_spec(path_spec):
    path, separator, run_text = path_spec.rpartition("@")
    if separator and run_text.isdigit():
        return path, int(run_text)
    return path_spec, None


def mean(values):
    return sum(values) / float(len(values)) if values else float("nan")


def rms(values):
    return math.sqrt(mean([value * value for value in values])) if values else float("nan")


def abs_max(values):
    return max([abs(value) for value in values]) if values else float("nan")


def span(values):
    return max(values) - min(values) if values else float("nan")


def fmt(value, digits=3):
    if isinstance(value, str):
        return value
    if value is None or not math.isfinite(float(value)):
        return "nan"
    return ("{:." + str(digits) + "f}").format(value)


def summarize_run(run_id, rows, steady_after):
    parsed = []
    missing = 0
    for row in rows:
        values = {key: finite_float(row.get(key)) for key in REQUIRED_FLOATS + OPTIONAL_FLOATS}
        missing += sum(values[key] is None for key in REQUIRED_FLOATS)
        values["row"] = row
        parsed.append(values)

    valid = [item for item in parsed if item["run_elapsed"] is not None]
    times = [item["run_elapsed"] for item in valid]
    duplicate_times = sum(curr == prev for prev, curr in zip(times, times[1:]))
    time_reversals = sum(curr < prev for prev, curr in zip(times, times[1:]))
    duration = max(times) - min(times) if len(times) > 1 else 0.0
    sample_hz = (len(times) - 1) / duration if duration > 0.0 else float("nan")

    parameters = []
    for item in valid:
        parameter_set = (
            item["normal_mass"], item["normal_damping"], item["desired_fz_N"],
            item["wave_height"], item["line_speed"],
        )
        if all(value is not None for value in parameter_set):
            parameters.append(tuple(round(value, 9) for value in parameter_set))
    unique_parameters = sorted(set(parameters))
    parameter_changes = max(0, len(unique_parameters) - 1)
    mass, damping, desired_force, wave_height, line_speed = (
        unique_parameters[0] if unique_parameters else (float("nan"),) * 5
    )

    steady = [item for item in valid if item["run_elapsed"] >= steady_after]
    if not steady:
        steady = valid
    force_errors = [item["force_error_N"] for item in steady if item["force_error_N"] is not None]
    all_force_errors = [item["force_error_N"] for item in valid if item["force_error_N"] is not None]
    forces = [item["fz_virtual_N"] for item in steady if item["fz_virtual_N"] is not None]
    eq_errors = [item["tcp_eq_err"] for item in steady if item["tcp_eq_err"] is not None]
    velocities = [item["vz_admittance"] for item in valid if item["vz_admittance"] is not None]
    x_errors = [item["x_error"] for item in steady if item["x_error"] is not None]
    y_errors = [item["y_error"] for item in steady if item["y_error"] is not None]
    xy_errors = [item["xy_error"] for item in steady if item["xy_error"] is not None]
    singular_values = [item["min_singular"] for item in valid if item["min_singular"] is not None]
    joint_errors = [item["joint_error_max"] for item in valid if item["joint_error_max"] is not None]
    link_x = [item["link_x"] for item in valid if item["link_x"] is not None]
    surface_steps = [item["surface_step"] for item in valid if item["surface_step"] is not None]
    velocity_limits = [item["max_z_velocity"] for item in valid if item["max_z_velocity"] is not None]
    xy_velocity_limits = [item["max_xy_velocity"] for item in valid if item["max_xy_velocity"] is not None]
    max_velocity = mean(velocity_limits)

    saturation_count = 0
    if velocities and math.isfinite(max_velocity) and max_velocity > 0.0:
        saturation_count = sum(abs(value) >= 0.99 * max_velocity for value in velocities)
    saturation_pct = 100.0 * saturation_count / len(velocities) if velocities else float("nan")
    contact_loss_pct = (
        100.0 * sum(value < 0.2 * desired_force for value in forces) / len(forces)
        if forces and math.isfinite(desired_force) else float("nan")
    )

    force_mean = mean(forces)
    force_bias = force_mean - desired_force
    metrics = {
        "run_id": run_id,
        "M_kg": mass,
        "D_Ns_m": damping,
        "Fd_N": desired_force,
        "wave_height_m": wave_height,
        "line_speed_m_s": line_speed,
        "max_xy_velocity_m_s": mean(xy_velocity_limits),
        "samples": len(rows),
        "duration_s": duration,
        "sample_hz": sample_hz,
        "missing_values": missing,
        "duplicate_times": duplicate_times,
        "time_reversals": time_reversals,
        "parameter_changes": parameter_changes,
        "force_mean_N": force_mean,
        "force_bias_N": force_bias,
        "force_rms_N": rms(force_errors),
        "force_peak_steady_N": abs_max(force_errors),
        "force_peak_all_N": abs_max(all_force_errors),
        "eq_rms_mm": 1000.0 * rms(eq_errors),
        "eq_max_mm": 1000.0 * abs_max(eq_errors),
        "vz_peak_m_s": abs_max(velocities),
        "vz_sat_pct": saturation_pct,
        "x_rms_mm": 1000.0 * rms(x_errors),
        "x_max_mm": 1000.0 * abs_max(x_errors),
        "y_rms_mm": 1000.0 * rms(y_errors),
        "y_max_mm": 1000.0 * abs_max(y_errors),
        "xy_rms_mm": 1000.0 * rms(xy_errors),
        "xy_max_mm": 1000.0 * abs_max(xy_errors),
        "fz_min_N": min(forces) if forces else float("nan"),
        "contact_loss_pct": contact_loss_pct,
        "x_span_mm": 1000.0 * span(link_x),
        "surface_span_mm": 1000.0 * span(surface_steps),
        "min_singular": min(singular_values) if singular_values else float("nan"),
        "joint_error_max_rad": max(joint_errors) if joint_errors else float("nan"),
    }
    quality_ok = (
        missing == 0 and duplicate_times == 0 and time_reversals == 0 and parameter_changes == 0
        and 25.0 <= sample_hz <= 35.0 and len(rows) >= 30
    )
    metrics["quality"] = "PASS" if quality_ok else "CHECK"

    # 这些是当前固定曲面、固定速度下的学习目标，不代表实物安全标准。
    acceptance_ok = quality_ok and all([
        abs(force_bias) <= 0.05 * desired_force,
        metrics["force_rms_N"] <= 0.10 * desired_force,
        metrics["force_peak_steady_N"] <= 0.25 * desired_force,
        metrics["eq_rms_mm"] <= 1.0,
        metrics["eq_max_mm"] <= 2.0,
        metrics["vz_sat_pct"] <= 1.0,
        metrics["xy_rms_mm"] <= 5.0,
        metrics["xy_max_mm"] <= 10.0,
        metrics["min_singular"] >= 0.02,
        metrics["joint_error_max_rad"] <= 0.01,
        metrics["contact_loss_pct"] == 0.0,
    ])
    metrics["acceptance"] = "PASS" if acceptance_ok else "CHECK"
    return metrics


def read_runs(path):
    groups = defaultdict(list)
    total_rows = 0
    with open(path, "r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        missing_columns = [column for column in REQUIRED_FLOATS + ["run_state"] if column not in (reader.fieldnames or [])]
        if missing_columns:
            raise ValueError("missing columns: {}".format(", ".join(missing_columns)))
        for row in reader:
            total_rows += 1
            if row.get("run_state") != "running":
                continue
            run_id = finite_float(row.get("run_id"))
            if run_id is not None:
                groups[int(run_id)].append(row)
    return total_rows, groups


def write_summary(path, summaries):
    output_path = os.path.join(os.path.dirname(os.path.abspath(path)), "summary.csv")
    with open(output_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(summary)
    return output_path


def most_common_parameter(summaries, key):
    counts = defaultdict(int)
    for item in summaries:
        value = item.get(key)
        if value is not None and math.isfinite(value):
            counts[round(value, 9)] += 1
    if not counts:
        return float("nan")
    return sorted(counts, key=lambda value: (-counts[value], value))[0]


def build_mdf_sweeps(summaries):
    """Extract one-variable M/D/Fd sweeps around the most common baseline."""
    definitions = [
        ("M", "M_kg", "kg", ("D_Ns_m", "Fd_N")),
        ("D", "D_Ns_m", "N s/m", ("M_kg", "Fd_N")),
        ("Fd", "Fd_N", "N", ("M_kg", "D_Ns_m")),
    ]
    baselines = {
        key: most_common_parameter(summaries, key)
        for key in ("M_kg", "D_Ns_m", "Fd_N")
    }
    sweeps = []
    for label, key, unit, fixed_keys in definitions:
        if not math.isfinite(baselines[key]):
            continue
        selected = []
        for item in summaries:
            if all(abs(item[fixed_key] - baselines[fixed_key]) < 1e-9 for fixed_key in fixed_keys):
                selected.append(item)
        values = sorted(set(item[key] for item in selected if math.isfinite(item[key])))
        if len(values) >= 2:
            sweeps.append({
                "label": label,
                "key": key,
                "unit": unit,
                "baseline": baselines[key],
                "values": values,
                "runs": selected,
            })
    return sweeps


def metric_mean_and_range(items, key):
    values = [item[key] for item in items if item.get(key) is not None and math.isfinite(item[key])]
    if not values:
        return float("nan"), 0.0, 0.0
    value_mean = mean(values)
    return value_mean, value_mean - min(values), max(values) - value_mean


def write_mdf_sweep_plot(summaries, output_path):
    """Draw numeric parameter sweeps; a line is clearer than bars for M/D/Fd."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except (ImportError, ValueError) as exc:
        raise ValueError("matplotlib is required for --plot: {}".format(exc))

    sweeps = build_mdf_sweeps(summaries)
    if not sweeps:
        return ""

    metrics = [
        ("Force error RMS", "N", "force_rms_N", 3),
        ("Steady force error peak", "N", "force_peak_steady_N", 3),
        ("TCP equilibrium error RMS", "mm", "eq_rms_mm", 3),
        ("Z velocity saturation", "%", "vz_sat_pct", 2),
    ]
    figure, axes = plt.subplots(len(metrics), len(sweeps), figsize=(13.8, 10.8), squeeze=False)
    colors = ["#1f77b4", "#e07a3f", "#2a9d8f", "#c94f45"]

    height_values = set(item["wave_height_m"] for item in summaries)
    speed_values = set(item["line_speed_m_s"] for item in summaries)
    condition = ""
    if len(height_values) == 1 and len(speed_values) == 1:
        condition = " (H={} mm, V={} mm/s)".format(
            fmt(1000.0 * next(iter(height_values)), 0),
            fmt(1000.0 * next(iter(speed_values)), 0),
        )
    figure.suptitle("RM75 sim_09 M/D/Fd controlled-variable sweeps" + condition, fontsize=14)

    for column, sweep in enumerate(sweeps):
        groups = defaultdict(list)
        for item in sweep["runs"]:
            groups[item[sweep["key"]]].append(item)
        x_values = sweep["values"]
        for row, (title, ylabel, metric_key, digits) in enumerate(metrics):
            axis = axes[row][column]
            means = []
            lower_errors = []
            upper_errors = []
            for x_value in x_values:
                value_mean, lower, upper = metric_mean_and_range(groups[x_value], metric_key)
                means.append(value_mean)
                lower_errors.append(lower)
                upper_errors.append(upper)

            axis.errorbar(
                x_values,
                means,
                yerr=[lower_errors, upper_errors],
                color=colors[row],
                marker="o",
                markersize=5,
                linewidth=2.0,
                capsize=4,
            )
            if sweep["baseline"] in groups:
                baseline_index = x_values.index(sweep["baseline"])
                axis.scatter(
                    [sweep["baseline"]], [means[baseline_index]], marker="*", s=105,
                    color="#111111", zorder=4,
                )
            for x_value, value in zip(x_values, means):
                axis.annotate(
                    fmt(value, digits), (x_value, value), xytext=(0, 7), textcoords="offset points",
                    ha="center", fontsize=8,
                )
            axis.set_xticks(x_values)
            axis.grid(axis="y", color="0.88", linewidth=0.8)
            axis.margins(x=0.18, y=0.22)
            if row == 0:
                axis.set_title("{} sweep".format(sweep["label"]), fontsize=11)
            if column == 0:
                axis.set_ylabel("{} ({})".format(title, ylabel), fontsize=9)
            if row == len(metrics) - 1:
                axis.set_xlabel("{} ({})".format(sweep["label"], sweep["unit"]), fontsize=9)

    figure.text(
        0.5, 0.012,
        "Each point is the mean of matching runs; whiskers show the min/max range; black star marks the baseline setting.",
        ha="center", fontsize=8,
    )
    figure.tight_layout(rect=[0.0, 0.035, 1.0, 0.955])
    output_path = os.path.abspath(output_path)
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return output_path


def write_comparison_plot(path, summaries, output_path=""):
    if len(summaries) < 2:
        return ""
    if not output_path:
        output_path = os.path.join(os.path.dirname(os.path.abspath(path)), "comparison.png")
    mdf_sweeps = build_mdf_sweeps(summaries)
    if len(mdf_sweeps) == 3:
        return write_mdf_sweep_plot(summaries, output_path)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except (ImportError, ValueError) as exc:
        raise ValueError("matplotlib is required for --plot: {}".format(exc))

    robustness_plot = len(set((item["wave_height_m"], item["line_speed_m_s"]) for item in summaries)) > 1
    if robustness_plot:
        labels = [
            "c{}\nH {} mm\nV {} mm/s".format(
                item.get("case_id", item["run_id"]),
                fmt(1000.0 * item["wave_height_m"], 0),
                fmt(1000.0 * item["line_speed_m_s"], 0),
            )
            for item in summaries
        ]
        figure_title = "RM75 sim_09 surface-height / wiping-speed robustness"
    else:
        labels = [
            "r{}\nM {}\nD {}\nFd {}".format(
                item["run_id"], fmt(item["M_kg"], 2), fmt(item["D_Ns_m"], 0), fmt(item["Fd_N"], 0)
            )
            for item in summaries
        ]
        figure_title = "RM75 sim_09 M/D/Fd controlled-variable comparison"
    colors = ["#c94f45" if item["acceptance"] != "PASS" else "#3b82a0" for item in summaries]
    panels = [
        ("Force error RMS", "N", "force_rms_N"),
        ("TCP equilibrium error RMS", "mm", "eq_rms_mm"),
        ("Z velocity saturation", "%", "vz_sat_pct"),
        ("Joint tracking error max", "rad", "joint_error_max_rad"),
    ]
    figure, axes = plt.subplots(2, 2, figsize=(13.5, 8.4))
    x_values = list(range(len(summaries)))
    for axis, (title, ylabel, key) in zip(axes.flat, panels):
        panel_values = [item[key] for item in summaries]
        axis.bar(x_values, panel_values, color=colors, width=0.72)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.set_xticks(x_values)
        axis.set_xticklabels(labels, fontsize=8)
        axis.grid(axis="y", color="0.88", linewidth=0.8)
        if key == "vz_sat_pct" and not any(panel_values):
            axis.set_ylim(0.0, 1.0)
            axis.text(0.5, 0.52, "All cases: 0.00%", ha="center", va="center", transform=axis.transAxes)
    figure.suptitle(figure_title)
    figure.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
    output_path = os.path.abspath(output_path)
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def write_combined_summary(output_path, summaries):
    output_path = os.path.abspath(output_path)
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    fields = ["case_id", "source_csv"] + SUMMARY_FIELDS
    with open(output_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(summary)
    return output_path


def print_summaries(path, total_rows, summaries, output_path):
    print("\nFILE {}".format(os.path.abspath(path)))
    print("rows={} running_runs={} summary={}".format(total_rows, len(summaries), output_path or "not written"))
    header = "run H(mm) V(mm/s)  M    D   Fd   n   Hz   F_rms F_peak F_bias eqRMS eqMax vzSat xyRMS xyMax minSig jointMax quality accept"
    print(header)
    for item in summaries:
        print(
            "{run_id:>3} {height:>5} {speed:>7} {M_kg:>4.2f} {D_Ns_m:>4.0f} {Fd_N:>4.1f} {samples:>4} {hz:>4} "
            "{frms:>6} {fpeak:>6} {fbias:>6} {eqrms:>5} {eqmax:>5} {sat:>5} "
            "{xyrms:>5} {xymax:>5} {sing:>6} {joint:>8} {quality:>7} {acceptance:>6}".format(
                height=fmt(1000.0 * item["wave_height_m"], 0),
                speed=fmt(1000.0 * item["line_speed_m_s"], 0),
                hz=fmt(item["sample_hz"], 1), frms=fmt(item["force_rms_N"]),
                fpeak=fmt(item["force_peak_steady_N"]), fbias=fmt(item["force_bias_N"]),
                eqrms=fmt(item["eq_rms_mm"]), eqmax=fmt(item["eq_max_mm"]),
                sat=fmt(item["vz_sat_pct"], 2), xyrms=fmt(item["xy_rms_mm"]),
                xymax=fmt(item["xy_max_mm"]), sing=fmt(item["min_singular"], 4),
                joint=fmt(item["joint_error_max_rad"], 5), **item
            )
        )


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    return_code = 0
    combined_summaries = []
    first_real_path = ""
    for path_spec in args.csv_paths:
        try:
            path, selected_run = parse_path_spec(path_spec)
            if not first_real_path:
                first_real_path = path
            total_rows, groups = read_runs(path)
            run_ids = [selected_run] if selected_run is not None else sorted(groups)
            missing_runs = [run_id for run_id in run_ids if run_id not in groups]
            if missing_runs:
                raise ValueError("missing selected run_id: {}".format(", ".join(map(str, missing_runs))))
            summaries = [summarize_run(run_id, groups[run_id], args.steady_after) for run_id in run_ids]
            output_path = "" if args.no_write else write_summary(path, summaries)
            comparison_path = write_comparison_plot(path, summaries) if args.plot else ""
            print_summaries(path, total_rows, summaries, output_path)
            if comparison_path:
                print("comparison={}".format(comparison_path))
            for summary in summaries:
                combined = dict(summary)
                combined["case_id"] = len(combined_summaries) + 1
                combined["source_csv"] = os.path.abspath(path)
                combined_summaries.append(combined)
            if not summaries:
                return_code = 1
        except (OSError, ValueError) as exc:
            print("ERROR {}: {}".format(path, exc), file=sys.stderr)
            return_code = 2
    if args.combined_output and combined_summaries:
        combined_csv = write_combined_summary(args.combined_output, combined_summaries)
        combined_png = os.path.splitext(combined_csv)[0] + ".png"
        write_comparison_plot(first_real_path, combined_summaries, combined_png)
        print("combined_csv={}".format(combined_csv))
        print("combined_plot={}".format(combined_png))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
