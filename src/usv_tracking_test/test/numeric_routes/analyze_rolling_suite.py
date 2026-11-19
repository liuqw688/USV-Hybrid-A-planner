#!/usr/bin/env python3
"""汇总并绘制使用 50 m 滚动 B 样条的闭环测试结果."""

import argparse
import json
from pathlib import Path
import re

from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import numpy as np  # noqa: E402
from route_shapes import CompositeRouteModel  # noqa: E402
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree  # noqa: E402


def publish_label(case):
    """返回固定、调制或随机发布频率的图例标签."""
    mode = case.get("publish_rate_mode", "fixed")
    if mode == "random":
        return (
            f"Random {case.get('publish_rate_min_hz', 1):g}–"
            f"{case.get('publish_rate_max_hz', 10):g} Hz"
        )
    frequency = int(case.get("publish_jitter_hz", 0))
    return "Stable 10 Hz" if frequency == 0 else f"10 Hz, {frequency} Hz modulation"


def load_csv(folder, name):
    """读取指定测试 CSV，跳过表头并确保结果为二维数组."""
    path = folder / f"{name}.csv"
    return np.atleast_2d(np.loadtxt(path, delimiter=",", skiprows=1))


def plot_state_timeline(suite, cases):
    """绘制实际状态机区间，并放大显示起步和到达阶段."""
    state_colors = {
        "WAIT_ODOM": "#9e9e9e",
        "WAIT_REFERENCE": "#e6ab02",
        "WAIT_START": "#a6761d",
        "TRACKING": "#1b9e77",
        "ARRIVAL_CONFIRM": "#7570b3",
        "SAFE_STOP": "#d95f02",
        "FINISHED": "#377eb8",
    }
    records = []
    for case in cases:
        path = Path(case["run_dir"]) / "run_status.json"
        if not path.exists():
            continue
        status = json.loads(path.read_text())
        transitions = status.get("tracking_transitions", [])
        if transitions:
            records.append((case, status, transitions))
    if not records:
        return

    fig = plt.figure(figsize=(14, 7))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.1, 1.0])
    full_ax = fig.add_subplot(grid[0, :])
    start_ax = fig.add_subplot(grid[1, 0])
    finish_ax = fig.add_subplot(grid[1, 1])
    startup_end = 2.0
    for row, (case, status, transitions) in enumerate(records):
        finish = status.get("finished_at_s")
        first_tracking = next(
            (t["wall_s"] for t in transitions if t["state"] == "TRACKING"), 0.0)
        startup_end = max(startup_end, first_tracking + 1.0)
        for index, transition in enumerate(transitions):
            start = transition["wall_s"]
            end = (transitions[index + 1]["wall_s"]
                   if index + 1 < len(transitions) else status["elapsed_s"])
            state = transition["state"].split(":", 1)[0]
            color = state_colors.get(state, "#000000")
            for axis in (full_ax, start_ax):
                axis.broken_barh([(start, end - start)],
                                 (row - 0.3, 0.6), facecolors=color)
            if finish is not None:
                finish_ax.broken_barh([(start - finish, end - start)],
                                      (row - 0.3, 0.6), facecolors=color)
            if state == "SAFE_STOP":
                full_ax.annotate(transition["state"], (start, row),
                                 xytext=(3, 9), textcoords="offset points",
                                 fontsize=7, color=color)
    labels = [f"phi={case['initial_phi_deg']} deg" for case, _, _ in records]
    for axis in (full_ax, start_ax, finish_ax):
        axis.set_yticks(range(len(records)), labels)
        axis.set_ylim(len(records) - 0.5, -0.5)
        axis.grid(axis="x", linewidth=0.4, alpha=0.35)
    full_ax.set_xlim(0.0, max(status["elapsed_s"] for _, status, _ in records))
    full_ax.set_title("Observed NMPC state transitions: complete closed-loop run")
    full_ax.set_xlabel("Elapsed wall time (s)")
    start_ax.set_xlim(0.0, startup_end)
    start_ax.set_title("Startup")
    start_ax.set_xlabel("Elapsed wall time (s)")
    finish_ax.set_xlim(-3.0, 2.0)
    finish_ax.axvline(0.0, color="#333333", linewidth=0.8, linestyle="--")
    finish_ax.set_title("Arrival, aligned to each run's finish event")
    finish_ax.set_xlabel("Time relative to FINISHED (s)")
    fig.legend(handles=[Patch(color=color, label=state)
                        for state, color in state_colors.items()],
               loc="lower center", ncol=4, frameon=False, fontsize=9)
    fig.tight_layout(rect=(0.0, 0.1, 1.0, 1.0))
    fig.savefig(suite / "state_timeline.png", dpi=180)
    plt.close(fig)


def analyze_run(case, route_tree, route):
    """
    统计单次闭环误差、控制性能、样条接收及发布周期指标.

    :param case: 包含 run_dir 等测试信息的字典。
    :param route_tree: 设计路线的平面最近邻索引。
    :param route: 设计路线模型。

    :return: 指标字典、真值、去重片段记录、路线距离、全局样条参考和动力学参考。
    """
    folder = Path(case["run_dir"])
    truth = load_csv(folder, "truth")
    odometry = load_csv(folder, "odom")
    starts = load_csv(folder, "spline_starts")
    signature_columns = [1, 2, 6, 7, 8]
    if starts.shape[1] >= 14:
        signature_columns.extend((11, 12, 13))
    signature = starts[:, signature_columns]
    new_reference = np.ones(len(starts), dtype=bool)
    new_reference[1:] = np.any(np.abs(np.diff(signature, axis=0)) > 1e-6, axis=1)
    unique_starts = starts[new_reference]
    diagnostics = load_csv(folder, "diagnostics")
    status = json.loads((folder / "run_status.json").read_text())
    setup = json.loads((folder / "test_setup.json").read_text())
    global_reference = None
    dynamics_reference = None
    global_reference_metrics = {}
    global_reference_error = None
    global_spline_path = folder / "global_reference_spline.json"
    if global_spline_path.exists():
        spline_data = json.loads(global_spline_path.read_text())
        degree = spline_data["degree"]
        controls = np.asarray(spline_data["controls"])
        knots = np.asarray(spline_data["knots"])
        spline = BSpline(knots, controls, degree)
        start, end = knots[degree], knots[len(controls)]
        sample_times = np.linspace(start, end, max(2000, int((end - start) * 50)))
        global_reference = spline(sample_times)[:, :2]
        global_reference_metrics = spline_data.get("generation", {})
        dynamics_reference = np.asarray(
            global_reference_metrics.get("dynamics_trajectory_xy", []), dtype=float)
        if dynamics_reference.size == 0:
            initial_state = np.array([
                global_reference_metrics["start_east_m"],
                global_reference_metrics["start_north_m"],
                global_reference_metrics["initial_heading_rad"],
                0.0, 0.0, 0.0,
            ])
            _, regenerated_metrics = route.generate_global_spline(initial_state)
            dynamics_reference = np.asarray(
                regenerated_metrics["dynamics_trajectory_xy"], dtype=float)
        global_reference_error = cKDTree(global_reference).query(truth[:, 1:3])[0]
    global_reference_to_route_error = (
        route_tree.query(global_reference)[0] if global_reference is not None else None
    )
    route_distance = route_tree.query(truth[:, 1:3])[0]
    run_time = truth[:, 0] - truth[0, 0]
    diag_time = diagnostics[:, 0] - diagnostics[0, 0]
    diag = diagnostics[:, 1:]
    true_at_diagnostic = np.column_stack(
        [np.interp(diagnostics[:, 0], truth[:, 0], truth[:, column]) for column in (1, 2)]
    )
    odom_at_truth = np.column_stack(
        [np.interp(truth[:, 0], odometry[:, 0], odometry[:, column]) for column in (1, 2)]
    )
    odometry_position_error = np.linalg.norm(odom_at_truth - truth[:, 1:3], axis=1)
    raw_position_measurement_error = np.linalg.norm(truth[:, 7:9] - truth[:, 1:3], axis=1)
    # 接收时刻插值包含求解和传输延迟；误差计算应使用同一诊断消息中的状态和参考。
    local_reference_error = np.linalg.norm(
        true_at_diagnostic - diag[:, [4, 5]], axis=1
    )
    odom_reference_error = np.linalg.norm(
        diag[:, [16, 17]] - diag[:, [4, 5]], axis=1
    )
    lateral_error = np.abs(diag[:, 29])
    cruise = (diag[:, 7] >= 3.95) & (np.abs(diag[:, 9]) < 0.01)
    after_initial_alignment = diag_time >= 2.0
    rolling_progress = []
    progress = 0.0
    for row in starts:
        progress = route.nearest_progress(row[1], row[2], progress, forward=12.0)
        rolling_progress.append(progress)
    rolling_progress = np.asarray(rolling_progress)
    progress_steps = np.diff(rolling_progress)
    receive_periods = np.diff(starts[:, 0])
    valid_receive_periods = receive_periods[receive_periods > 0.0]
    local_deviation_at_10m = starts[:, 11] if starts.shape[1] >= 14 else np.array([])
    local_deviation_p95 = starts[:, 12] if starts.shape[1] >= 14 else np.array([])
    local_deviation_p95 = local_deviation_p95[np.isfinite(local_deviation_p95)]
    local_deviation_at_10m = local_deviation_at_10m[np.isfinite(local_deviation_at_10m)]
    after_startup = run_time >= 2.0
    steady_track_error = (
        global_reference_error[after_startup]
        if global_reference_error is not None and np.any(after_startup)
        else global_reference_error
    )
    controller_log = folder / "controller.log"
    log_text = controller_log.read_text(errors="replace") if controller_log.exists() else ""
    rejected_count = len(re.findall("拒绝 USV B 样条参考", log_text))
    accepted_count = len(re.findall("已接收 B 样条 traj_id", log_text))
    # 到达完成诊断复用上一次代价值，不代表本周期执行了求解。
    solve_rows = np.ones(len(diag), dtype=bool)
    if diag.shape[1] >= 69:
        solve_rows = diag[:, 67] != 6.0
    elif status.get("finished_at_s") is not None:
        solve_rows[-1] = False
    solve_success = diag[solve_rows, 13] == 1.0
    within_2m = np.flatnonzero(route_distance <= 2.0)
    first_30s = route_distance[run_time <= 30.0]
    last_30s = route_distance[run_time >= max(0.0, run_time[-1] - 30.0)]

    metrics = {
        "name": case["name"],
        "initial_phi_deg": case["initial_phi_deg"],
        "initial_east_m": case["start_offset_x_m"],
        "initial_north_m": case["start_offset_y_m"],
        "initial_offset_distance_m": case["start_gap_m"],
        "duration_s": float(truth[-1, 0] - truth[0, 0]),
        "finished": status.get("finished_at_s") is not None,
        "finish_time_s": status.get("finished_at_s"),
        "tracking_state_sequence": [
            transition["state"] for transition in status.get("tracking_transitions", [])
        ],
        "safe_stop_transition_count": sum(
            transition["state"].startswith("SAFE_STOP")
            for transition in status.get("tracking_transitions", [])
        ),
        "test_conditions": {
            key: setup.get(key) for key in (
                "nmpc_horizon_steps", "nmpc_dt_s", "nmpc_horizon_s",
                "noise_stddev_m", "heading_noise_stddev_deg", "noise_seed",
                "publish_jitter_hz", "publish_interval_jitter_fraction",
                "publish_rate_mode", "publish_rate_min_hz", "publish_rate_max_hz",
                "publish_seed",
                "local_deviation_stddev_m", "local_deviation_seed",
            )
        },
        "final_east_m": float(truth[-1, 1]),
        "final_north_m": float(truth[-1, 2]),
        "route_error_p50_m": float(np.percentile(route_distance, 50)),
        "route_error_p95_m": float(np.percentile(route_distance, 95)),
        "route_error_max_m": float(np.max(route_distance)),
        "global_spline_track_error_p50_m": float(np.percentile(global_reference_error, 50))
        if global_reference_error is not None else None,
        "global_spline_track_error_p95_m": float(np.percentile(global_reference_error, 95))
        if global_reference_error is not None else None,
        "global_spline_track_error_max_m": float(np.max(global_reference_error))
        if global_reference_error is not None else None,
        "global_spline_track_error_max_time_s": float(
            run_time[np.argmax(global_reference_error)])
        if global_reference_error is not None else None,
        "global_spline_track_error_within_0_3m_fraction": float(
            np.mean(global_reference_error <= 0.3))
        if global_reference_error is not None else None,
        "global_spline_track_error_rmse_m": float(
            np.sqrt(np.mean(global_reference_error ** 2)))
        if global_reference_error is not None else None,
        "global_spline_track_error_after_2s_p95_m": float(
            np.percentile(steady_track_error, 95))
        if steady_track_error is not None else None,
        "global_spline_track_error_after_2s_max_m": float(np.max(steady_track_error))
        if steady_track_error is not None else None,
        "global_spline_to_designed_route_p95_m": float(
            np.percentile(global_reference_to_route_error, 95))
        if global_reference_to_route_error is not None else None,
        "global_spline_model_residual_p95": global_reference_metrics.get(
            "dynamics_residual_p95"),
        "global_spline_model_residual_max": global_reference_metrics.get(
            "dynamics_residual_max"),
        "route_error_first_30s_p95_m": float(np.percentile(first_30s, 95)),
        "route_error_last_30s_p95_m": float(np.percentile(last_30s, 95)),
        "true_to_local_reference_p50_m": float(np.percentile(local_reference_error, 50)),
        "true_to_local_reference_p95_m": float(np.percentile(local_reference_error, 95)),
        "true_to_local_reference_max_m": float(np.max(local_reference_error)),
        "true_to_local_reference_after_2s_p95_m": float(
            np.percentile(local_reference_error[after_initial_alignment], 95)
        ) if np.any(after_initial_alignment) else None,
        "true_to_local_reference_within_0_3m_fraction": float(
            np.mean(local_reference_error <= 0.3)
        ),
        "odom_to_local_reference_p95_m": float(np.percentile(odom_reference_error, 95)),
        "odom_to_local_reference_max_m": float(np.max(odom_reference_error)),
        "odom_to_local_reference_within_0_3m_fraction": float(
            np.mean(odom_reference_error <= 0.3)),
        "synchronized_lateral_error_p95_m": float(np.percentile(lateral_error, 95)),
        "synchronized_lateral_error_max_m": float(np.max(lateral_error)),
        "u_error_rmse_mps": float(np.sqrt(np.mean(diag[:, 31] ** 2))),
        "v_error_rmse_mps": float(np.sqrt(np.mean(diag[:, 32] ** 2))),
        "r_error_rmse_radps": float(np.sqrt(np.mean(diag[:, 33] ** 2))),
        "straight_cruise_u_median_mps": float(np.median(diag[cruise, 19]))
        if np.any(cruise) else None,
        "final_true_speed_mps": float(np.hypot(truth[-1, 4], truth[-1, 5])),
        "input_amplitude_cost_max": float(np.max(np.abs(diag[:, 40]))),
        "odometry_position_error_radial_rms_m": float(
            np.sqrt(np.mean(odometry_position_error ** 2))
        ),
        "raw_position_measurement_noise_radial_rms_m": float(
            np.sqrt(np.mean(raw_position_measurement_error ** 2))
        ),
        "first_time_within_2m_s": float(run_time[within_2m[0]]) if len(within_2m) else None,
        "route_error_within_2m_fraction": float(np.mean(route_distance <= 2.0)),
        "new_spline_count": int(len(unique_starts)),
        "repeated_spline_message_count": int(len(starts) - len(unique_starts)),
        "start_gap_max_m": float(np.max(unique_starts[:, 5])),
        "start_gap_over_2m_count": int(np.sum(unique_starts[:, 5] > 2.0)),
        "segment_length_min_m": float(np.min(unique_starts[:, 7])),
        "segment_length_max_m": float(np.max(unique_starts[:, 7])),
        "rolling_progress_start_m": float(rolling_progress[0]),
        "rolling_progress_end_m": float(rolling_progress[-1]),
        "rolling_progress_nonpositive_steps": int(np.sum(progress_steps <= 1e-4)),
        "rolling_progress_min_step_m": float(np.min(progress_steps))
        if len(progress_steps) else 0.0,
        "message_receive_rate_median_hz": float(1.0 / np.median(valid_receive_periods))
        if len(valid_receive_periods)
        else None,
        "message_receive_rate_mean_hz": float(
            (len(starts) - 1) / max(starts[-1, 0] - starts[0, 0], 1e-12)
        ) if len(starts) > 1 else None,
        "message_receive_interval_min_s": float(np.min(valid_receive_periods))
        if len(valid_receive_periods) else None,
        "message_receive_interval_p95_s": float(np.percentile(valid_receive_periods, 95))
        if len(valid_receive_periods) else None,
        "message_receive_interval_max_s": float(np.max(valid_receive_periods))
        if len(valid_receive_periods) else None,
        "local_path_signed_deviation_at_10m_p95_abs_m": float(
            np.percentile(np.abs(local_deviation_at_10m), 95)
        ) if len(local_deviation_at_10m) else None,
        "local_path_deviation_per_segment_p95_m": float(
            np.percentile(local_deviation_p95, 95)
        ) if len(local_deviation_p95) else None,
        "successful_solve_fraction": float(np.mean(solve_success)),
        "successful_solve_count": int(np.sum(solve_success)),
        "failed_solve_count": int(np.sum(~solve_success)),
        "accepted_spline_log_count": accepted_count,
        "rejected_spline_log_count": rejected_count,
        "all_messages_gap_over_2m_count": status.get("start_gap_over_2m_count"),
    }
    return metrics, truth, unique_starts, route_distance, global_reference, dynamics_reference


def plot_publish_jitter(suite, traces):
    """对比发布周期、随机局部重规划和闭环路线误差."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    colors = {0: "#2678a8", 1: "#d17a22", 5: "#3a8f68"}
    has_random_rates = any(case.get("publish_rate_mode") == "random" for case, *_ in traces)
    rate_inset = axes[0].inset_axes([0.11, 0.13, 0.32, 0.72])
    for case, truth, starts, _, global_reference, _ in traces:
        frequency = int(case.get("publish_jitter_hz", 0))
        random_mode = case.get("publish_rate_mode") == "random"
        color = "#bf5b17" if random_mode else colors.get(
            frequency, plt.get_cmap("tab10")(frequency % 10))
        label = publish_label(case)
        elapsed = starts[:, 0] - starts[0, 0]
        periods = np.diff(starts[:, 0])
        valid = periods > 0.0
        axes[0].plot(
            elapsed[1:][valid], periods[valid], color=color,
            linewidth=0.65, label=label)
        zoomed = valid & (elapsed[1:] <= 2.0)
        rate_inset.plot(
            elapsed[1:][zoomed], periods[zoomed], color=color,
            linewidth=0.9, label=label)
        if starts.shape[1] >= 14:
            if frequency == 0:
                axes[1].plot(
                    elapsed, starts[:, 11], color=color, linewidth=0.65,
                    label="Shared seeded local offset")
            axes[3].plot(
                elapsed, starts[:, 5], color=color, linewidth=0.65, label=label)
        if global_reference is not None:
            track_error = cKDTree(global_reference).query(truth[:, 1:3])[0]
            axes[2].plot(
                truth[:, 0] - truth[0, 0], track_error,
                color=color, linewidth=0.8, label=label)

    axes[0].axhline(0.1, color="#444444", linewidth=0.8, linestyle="--")
    axes[0].set_ylim(0.05, 1.05 if has_random_rates else 0.16)
    axes[0].set_ylabel("Message receive interval (s)")
    rate_inset.set_xlim(0.0, 2.0)
    rate_inset.set_ylim(0.05, 1.05 if has_random_rates else 0.16)
    if not has_random_rates:
        rate_inset.axhline(0.1, color="#444444", linewidth=0.65, linestyle="--")
    rate_inset.grid(True, linewidth=0.35, alpha=0.35)
    rate_inset.set_title("Startup detail", fontsize=8)
    rate_inset.tick_params(labelsize=7)
    axes[1].axhline(0.0, color="#444444", linewidth=0.7)
    axes[1].set_ylabel("Signed local path\noffset at 10 m (m)")
    axes[2].axhline(0.3, color="#444444", linewidth=0.8, linestyle="--")
    axes[2].set_ylabel("Vessel to global\nspline (m)")
    axes[3].axhline(2.0, color="#444444", linewidth=0.8, linestyle="--")
    axes[3].set_ylabel("Segment start gap (m)")
    axes[3].set_xlabel("Elapsed time (s)")
    for axis in axes:
        axis.grid(True, linewidth=0.4, alpha=0.35)
        axis.legend(loc="upper right", ncol=3, fontsize=8, frameon=False)
    fig.suptitle("Complex-route local spline publication and tracking response")
    fig.tight_layout()
    fig.savefig(suite / "publish_jitter_and_route_deviation.png", dpi=180)
    plt.close(fig)


def main():
    """读取测试套件数据，输出轨迹、控制量、状态机图和汇总指标."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    args = parser.parse_args()
    suite = args.suite.resolve()
    cases = json.loads((suite / "suite_status.json").read_text())["cases"]
    route = CompositeRouteModel()
    route_tree = cKDTree(route.xy[::2])
    palette = plt.get_cmap("tab10")
    analyzed = []
    traces = []
    for case in cases:
        folder = Path(case["run_dir"])
        if not (folder / "truth.csv").exists() or not (folder / "spline_starts.csv").exists():
            continue
        metrics, truth, starts, route_distance, global_reference, dynamics_reference = analyze_run(
            case, route_tree, route)
        analyzed.append(metrics)
        traces.append((case, truth, starts, route_distance, global_reference, dynamics_reference))

    if not traces:
        raise RuntimeError(f"No usable run data in {suite}")

    fig, (path_ax, error_ax, gap_ax) = plt.subplots(
        3, 1, figsize=(12, 10), gridspec_kw={"height_ratios": [3.2, 1.3, 1.1]}
    )
    path_ax.plot(
        route.xy[:, 0], route.xy[:, 1], color="#333333", linewidth=1.2,
        label="Designed route", zorder=1,
    )
    for index, (
        case, truth, starts, route_distance, global_reference, dynamics_reference
    ) in enumerate(traces):
        color = palette(index % 10)
        label = publish_label(case)
        if dynamics_reference is not None:
            path_ax.plot(
                dynamics_reference[:, 0], dynamics_reference[:, 1], color=color,
                linewidth=0.65, linestyle=":", alpha=0.65,
                label=f"model-integrated ref {label}", zorder=1)
        if global_reference is not None:
            path_ax.plot(global_reference[:, 0], global_reference[:, 1], color=color,
                         linewidth=1.0, linestyle="--", alpha=0.72,
                         label=f"B-spline ref {label}", zorder=1)
        path_ax.plot(truth[:, 1], truth[:, 2], color=color, linewidth=1.0,
                     alpha=0.95, label=f"dynamics track {label}", zorder=2)
        path_ax.scatter(truth[0, 1], truth[0, 2], color=color, marker="o", s=18, zorder=3)
        path_ax.scatter(truth[-1, 1], truth[-1, 2], color=color, marker="x", s=24, zorder=3)
        track_error = (cKDTree(global_reference).query(truth[:, 1:3])[0]
                       if global_reference is not None else route_distance)
        error_ax.plot(truth[:, 0] - truth[0, 0], track_error,
                      color=color, linewidth=0.9, label=label)
        peak = int(np.argmax(track_error))
        if track_error[peak] > 0.3:
            peak_time = truth[peak, 0] - truth[0, 0]
            error_ax.scatter(peak_time, track_error[peak], color=color, s=18, zorder=4)
            error_ax.annotate(
                f"{case['initial_phi_deg']} deg: {track_error[peak]:.3f} m",
                (peak_time, track_error[peak]), xytext=(12, 7),
                textcoords="offset points", fontsize=8, color=color)
        gap_ax.plot(starts[:, 0], starts[:, 5], color=color, linewidth=0.9, label=label)

    path_ax.set_title("Dynamics-generated global B-spline reference and NMPC vessel tracks")
    path_ax.set_xlabel("East position (m)")
    path_ax.set_ylabel("North position (m)")
    path_ax.grid(True, linewidth=0.4, alpha=0.35)
    path_ax.legend(loc="upper right", ncol=2, fontsize=8, frameon=False)
    path_ax.set_aspect("equal", adjustable="datalim")
    inset = path_ax.inset_axes([0.06, 0.48, 0.28, 0.44])
    inset.plot(route.xy[:, 0], route.xy[:, 1], color="#333333", linewidth=1.0)
    for index, (_, truth, _, _, global_reference, dynamics_reference) in enumerate(traces):
        if dynamics_reference is not None:
            inset.plot(
                dynamics_reference[:, 0], dynamics_reference[:, 1],
                color=palette(index % 10), linewidth=0.65, linestyle=":", alpha=0.65)
        if global_reference is not None:
            inset.plot(global_reference[:, 0], global_reference[:, 1],
                       color=palette(index % 10), linewidth=0.9, linestyle="--", alpha=0.72)
        inset.plot(truth[:, 1], truth[:, 2], color=palette(index % 10), linewidth=0.8)
    inset.set_xlim(-12.0, 70.0)
    inset.set_ylim(-28.0, 36.0)
    inset.set_aspect("equal", adjustable="box")
    inset.grid(True, linewidth=0.3, alpha=0.3)
    inset.set_title("Start and R7 turn", fontsize=8)
    inset.tick_params(labelsize=7)

    error_ax.axhline(0.3, color="#333333", linestyle="--", linewidth=0.9,
                     label="0.3 m tracking target")
    error_ax.margins(y=0.16)
    error_ax.set_ylabel("Distance to global\nB-spline (m)")
    error_ax.grid(True, linewidth=0.4, alpha=0.35)
    error_ax.legend(loc="upper right", ncol=3, fontsize=8, frameon=False)

    gap_ax.axhline(2.0, color="#333333", linestyle="--", linewidth=0.9,
                   label="2 m acceptance limit")
    gap_ax.set_xlabel("Elapsed time (s)")
    gap_ax.set_ylabel("Spline start gap (m)")
    gap_ax.grid(True, linewidth=0.4, alpha=0.35)
    gap_ax.legend(loc="upper right", ncol=3, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(suite / "tracking_paths.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
    for index, (case, _, _, _, _, _) in enumerate(traces):
        diagnostic = load_csv(Path(case["run_dir"]), "diagnostics")
        time_s = diagnostic[:, 0]
        diag = diagnostic[:, 1:]
        color = palette(index % 10)
        label = publish_label(case)
        for axis, state_column, reference_column, scale in (
            (axes[0], 19, 7, 1.0),
            (axes[1], 20, 8, 1.0),
            (axes[2], 21, 9, 180.0 / np.pi),
        ):
            axis.plot(time_s, scale * diag[:, state_column], color=color,
                      linewidth=0.85, label=label)
            axis.plot(time_s, scale * diag[:, reference_column], color=color,
                      linewidth=0.7, linestyle="--", alpha=0.65)
        axes[3].plot(time_s, diag[:, 61], color=color, linewidth=0.8)
        axes[4].plot(time_s, np.degrees(diag[:, 62]), color=color, linewidth=0.8)
        error = np.linalg.norm(diag[:, [16, 17]] - diag[:, [4, 5]], axis=1)
        axes[5].plot(time_s, error, color=color, linewidth=0.7)
    axes[0].set_title("State tracking (solid: feedback, dashed: reference) and commands")
    for axis, ylabel in zip(axes, (
        "u (m/s)", "v (m/s)", "r (deg/s)", "V_cmd (m/s)",
        "W_cmd (deg/s)", "Synchronized odom-to-ref\nposition error (m)",
    )):
        axis.set_ylabel(ylabel)
        axis.grid(True, linewidth=0.4, alpha=0.35)
    axes[0].legend(loc="upper right", ncol=5, fontsize=8, frameon=False)
    axes[5].axhline(0.3, color="#333333", linestyle="--", linewidth=0.9)
    axes[5].set_xlabel("Elapsed time (s)")
    fig.tight_layout()
    fig.savefig(suite / "control_channels.png", dpi=180)
    plt.close(fig)
    plot_publish_jitter(suite, traces)
    plot_state_timeline(suite, cases)
    (suite / "rolling_summary.json").write_text(
        json.dumps(analyzed, ensure_ascii=False, indent=2)
    )
    print(json.dumps(analyzed, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
