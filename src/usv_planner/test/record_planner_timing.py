#!/usr/bin/env python3
"""只读记录运行中规划器的计算耗时、样条发布间隔与轨迹版本切换及参考速度连续性。"""

import argparse
import csv
import json
from pathlib import Path
import re
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.interpolate import BSpline
from std_msgs.msg import Float64MultiArray, String
from usv_interfaces.msg import UsvBspline


def statistics(values):
    """返回一组同单位样本的统计量；无样本时返回空对象。"""
    if not values:
        return {}
    return {"count": len(values), "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)), "max": float(max(values))}


def planning_rows(path):
    """提取节点现有日志中的完整规划调用耗时，包含失败调用。"""
    if path is None:
        return []
    pattern = re.compile(
        r"\[INFO\] \[([0-9.]+)\].*Planning wall time: ([0-9.]+) ms success=([01])")
    return [{"time_s": float(t), "wall_ms": float(ms), "success": int(ok)}
            for t, ms, ok in pattern.findall(path.read_text(errors="replace"))]


def write_csv(path, rows):
    """保存具有相同字段的原始记录。"""
    if rows:
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def plot(output, messages, plans, start):
    """分别展示消息发布间隔、新版本间隔及规划计算时间。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(11, 8), constrained_layout=True)
    if len(messages) > 1:
        stamps = [m["publish_s"] for m in messages]
        axes[0].plot([t - start for t in stamps[1:]], np.diff(stamps) * 1000, lw=1)
    axes[0].axhline(100, color="tab:orange", ls="--", label="10 Hz = 100 ms")
    axes[0].set(ylabel="Message interval (ms)", title="B-spline stream (includes unchanged references)")
    axes[0].legend()
    changes = [m for m in messages if m["new_version"]]
    if changes:
        axes[1].step([m["publish_s"] - start for m in changes],
                     [m["traj_id"] for m in changes], where="post", marker="o")
    axes[1].set(ylabel="Trajectory ID", title="New plans: updates are triggered by distance / safety / goal")
    if plans:
        times = [p["time_s"] - start for p in plans]
        colors = ["tab:blue" if p["success"] else "tab:red" for p in plans]
        axes[2].scatter(times, [p["wall_ms"] for p in plans], c=colors, s=24)
    axes[2].set(ylabel="Planning wall time (ms)", xlabel="Time from observation start (s)",
                title="Planning calls during observation (blue = success, red = failure)")
    for axis in axes:
        axis.grid(alpha=.25)
    figure.savefig(output / "planner_timing.png", dpi=160)
    plt.close(figure)


def spline_curve(msg):
    """按接收端规则展开导航角并构造样条，保持原消息绝对时间。"""
    controls = np.array([[p.x, p.y, p.phi] for p in msg.control_points])
    controls[:, 2] = np.unwrap(controls[:, 2])
    curve = BSpline(msg.knots, controls, msg.order, extrapolate=False)
    return {"pose": curve, "derivative": curve.derivative(),
            "start": msg.start_time.sec + msg.start_time.nanosec * 1e-9,
            "lower": msg.knots[msg.order], "upper": msg.knots[len(controls)]}


def spline_state(curve, absolute_time):
    """复现 NMPC 取参考规则：段内求导，段后按末端世界速度外推。"""
    elapsed = max(0.0, absolute_time - curve["start"])
    t = min(curve["lower"] + elapsed, curve["upper"])
    pose = curve["pose"](t)
    derivative = curve["derivative"](t)
    pose = pose + derivative * max(0.0, curve["lower"] + elapsed - curve["upper"])
    dx, dy, r = derivative
    phi = pose[2]
    return np.array([*pose, -np.sin(phi) * dx + np.cos(phi) * dy,
                     np.cos(phi) * dx + np.sin(phi) * dy, r])


def plot_speed(output, diagnostics, switches, start):
    """对照参考、实际速度与同一时刻的新旧样条速度差。"""
    if not diagnostics:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), constrained_layout=True)
    times = [d["time_s"] - start for d in diagnostics]
    for axis, key, unit in zip(axes[:3], ["u", "v", "r"], ["m/s", "m/s", "rad/s"]):
        axis.plot(times, [d["ref_" + key] for d in diagnostics], label="NMPC reference", lw=1.3)
        axis.plot(times, [d["actual_" + key] for d in diagnostics], label="Actual", lw=1, alpha=.8)
        axis.set_ylabel(f"{key} ({unit})")
        for switch in switches:
            axis.axvline(switch["time_s"] - start, color="grey", alpha=.25, lw=.7)
        axis.legend(loc="upper right")
    if switches:
        st = [r["time_s"] - start for r in switches]
        for key in ["u", "v", "r"]:
            axes[3].plot(st, [r["delta_" + key] for r in switches], "o-", label="Delta " + key)
    axes[0].set_title("Live B-spline tracking: grey lines = newly published trajectory versions")
    axes[3].set(title="New minus old reference evaluated at the SAME reception time",
                ylabel="Delta (m/s; rad/s)", xlabel="Time from observation start (s)")
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=.25)
    fig.savefig(output / "reference_speed.png", dpi=160)
    plt.close(fig)


def record(args):
    """订阅现有话题；不发布目标、船位、控制指令或参数。"""
    rclpy.init()
    node = rclpy.create_node("record_planner_timing")
    messages, states, diagnostics, switches, versions = [], [], [], [], []
    last_version, active_curve = None, None

    def receive(msg, info):
        nonlocal last_version, active_curve
        received = time.time()
        source_stamp = (info.get("source_timestamp", 0) if isinstance(info, dict)
                        else info.source_timestamp)
        publisher_gid = (info.get("publisher_gid", []) if isinstance(info, dict)
                         else info.publisher_gid)
        source = source_stamp * 1e-9
        version = (bytes(publisher_gid).hex(), msg.traj_id, msg.start_time.sec, msg.start_time.nanosec)
        start_time = msg.start_time.sec + msg.start_time.nanosec * 1e-9
        duration = (msg.knots[-msg.order - 1] - msg.knots[msg.order]
                    if len(msg.knots) > 2 * msg.order + 1 else 0.0)
        messages.append({"receive_s": received,
                         "publish_s": source if source > 0.0 else received,
                         "dds_delay_ms": (received - source) * 1000 if source > 0.0 else None,
                         "publisher_gid": bytes(publisher_gid).hex(),
                         "traj_id": msg.traj_id, "start_s": start_time,
                         "reference_elapsed_s": node.get_clock().now().nanoseconds * 1e-9 - start_time,
                         "duration_s": duration, "control_points": len(msg.control_points),
                         "new_version": int(version != last_version)})
        if version != last_version:
            curve = spline_curve(msg)
            if active_curve is not None:
                old, new = spline_state(active_curve, received), spline_state(curve, received)
                delta = new - old
                row = {"time_s": received, "old_id": last_version[1], "new_id": msg.traj_id,
                       "position_delta_m": float(np.linalg.norm(delta[:2])),
                       "heading_delta_rad": float(np.arctan2(np.sin(delta[2]), np.cos(delta[2]))),
                       "old_in_domain": int(active_curve["start"] <= received <=
                           active_curve["start"] + active_curve["upper"] - active_curve["lower"]),
                       "new_in_domain": int(curve["start"] <= received <=
                           curve["start"] + curve["upper"] - curve["lower"])}
                for i, key in enumerate(["u", "v", "r"], 3):
                    row["old_" + key], row["new_" + key], row["delta_" + key] = map(float, (old[i], new[i], delta[i]))
                switches.append(row)
            active_curve = curve
            versions.append({"receive_s": received, "traj_id": msg.traj_id,
                             "start_s": start_time, "order": msg.order, "knots": list(msg.knots),
                             "controls": [[p.x, p.y, p.phi] for p in msg.control_points]})
        at_publish = spline_state(active_curve, source if source > 0.0 else received)
        at_receive = spline_state(active_curve, received)
        for i, key in enumerate(["u", "v", "r"], 3):
            messages[-1]["publish_" + key] = float(at_publish[i])
            messages[-1]["receive_" + key] = float(at_receive[i])
            messages[-1]["transport_delta_" + key] = float(at_receive[i] - at_publish[i])
        last_version = version

    def receive_diagnostic(msg):
        data = msg.data
        if len(data) < 69:
            return
        diagnostics.append({"time_s": data[64], "receive_s": time.time(),
                            "revision": int(data[15]), "ref_x": data[4], "ref_y": data[5],
                            "ref_phi": data[6], "ref_u": data[7], "ref_v": data[8], "ref_r": data[9],
                            "actual_u": data[19], "actual_v": data[20], "actual_r": data[21],
                            "command_speed": data[61], "command_yaw": data[62],
                            "solver_success": int(data[13]), "fsm_state": int(data[67]),
                            "stop_reason": int(data[68])})

    subscriptions = [
        node.create_subscription(UsvBspline, args.topic, receive, 100),
        node.create_subscription(Float64MultiArray, "/nmpc_tracking_diagnostics", receive_diagnostic, 100),
        node.create_subscription(String, "/nmpc_tracking_state",
                                 lambda msg: states.append({"time_s": time.time(), "state": msg.data}),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    start, monotonic_start = time.time(), time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - monotonic_start < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        publishers = [{"node": p.node_name, "namespace": p.node_namespace}
                      for p in node.get_publishers_info_by_topic(args.topic)]
        node.destroy_node()
        rclpy.shutdown()
        del subscriptions

    end = time.time()
    all_plans = planning_rows(args.planner_log)
    current_plans = [p for p in all_plans if start <= p["time_s"] <= end]
    publish_gaps = [b["publish_s"] - a["publish_s"] for a, b in zip(messages, messages[1:])]
    receive_gaps = [b["receive_s"] - a["receive_s"] for a, b in zip(messages, messages[1:])]
    changes = [m for m in messages if m["new_version"]]
    # 首次观测可能位于旧轨迹中途，不将首个不完整版本间隔计入统计。
    new_gaps = [b["publish_s"] - a["publish_s"] for a, b in zip(changes[1:], changes[2:])]
    summary = {"observation_start_s": start, "observation_end_s": end,
               "topic": args.topic, "publishers": publishers, "message_count": len(messages),
               "observed_versions": len(changes), "publish_interval_s": statistics(publish_gaps),
               "receive_interval_s": statistics(receive_gaps),
               "new_version_interval_s": statistics(new_gaps),
               "dds_delay_ms": statistics([m["dds_delay_ms"] for m in messages if m["dds_delay_ms"] is not None]),
               "current_planning_wall_ms": statistics([p["wall_ms"] for p in current_plans]),
               "log_planning_wall_ms": statistics([p["wall_ms"] for p in all_plans]),
               "current_failed_plans": sum(not p["success"] for p in current_plans),
               "log_failed_plans": sum(not p["success"] for p in all_plans),
               "controller_states": states, "planner_log": str(args.planner_log)}
    summary["same_time_version_switches"] = {
        key: statistics([abs(row[key]) for row in switches])
        for key in ["position_delta_m", "heading_delta_rad", "delta_u", "delta_v", "delta_r"]}
    summary["transport_speed_change"] = {
        key: statistics([abs(row["transport_delta_" + key]) for row in messages])
        for key in ["u", "v", "r"]}
    summary["controller_speed"] = {
        key: statistics([row[key] for row in diagnostics])
        for key in ["ref_u", "ref_v", "ref_r", "actual_u", "actual_v", "actual_r"]}
    summary["diagnostic_count"] = len(diagnostics)
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "reference_switches.csv", switches)
    write_csv(args.output / "nmpc_reference.csv", diagnostics)
    (args.output / "spline_versions.json").write_text(json.dumps(versions, indent=2) + "\n")
    plot_speed(args.output, diagnostics, switches, start)
    write_csv(args.output / "bspline_messages.csv", messages)
    write_csv(args.output / "planning_calls.csv", all_plans)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    plot(args.output, messages, current_plans, start)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"Results: {args.output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="CSV、统计和图表的输出目录")
    parser.add_argument("--duration", type=float, default=45.0, help="观测时间，单位秒")
    parser.add_argument("--topic", default="/planning/bspline")
    parser.add_argument("--planner-log", type=Path, help="待分析的规划器节点日志")
    arguments = parser.parse_args()
    if arguments.duration <= 0.0:
        parser.error("--duration 必须大于零")
    record(arguments)
