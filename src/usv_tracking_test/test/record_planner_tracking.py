#!/usr/bin/env python3
"""记录真实 ROS 规划跟踪闭环，保存原始数据并绘制航道、参考与实际船迹。"""

import argparse
import csv
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, Twist
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib import font_manager
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, String
from usv_interfaces.msg import UsvBspline


def stamp_seconds(stamp):
    """将 ROS 时间戳转换为秒，不改变消息时间基准。"""
    return stamp.sec + stamp.nanosec * 1e-9


def record(args):
    """订阅实际节点，确认地图和里程计就绪后发布一次 RViz 格式目标。"""
    rclpy.init()
    node = rclpy.create_node("record_planner_tracking")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data = {key: [] for key in ("odom", "truth", "commands", "diagnostics", "splines", "events")}
    origin = []
    grid = []
    finished_at = None
    goal_time = None

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_odom(msg):
        q = msg.pose.pose.orientation
        phi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        p, v = msg.pose.pose.position, msg.twist.twist
        data["odom"].append([stamp_seconds(msg.header.stamp), p.x, p.y, phi,
                             v.linear.y, v.linear.x, v.angular.z])

    def on_state(msg):
        if len(msg.data) != 19 or not origin:
            return
        d = msg.data
        lat, lon = map(math.radians, origin[:2])
        # 与船模发布端使用同一球面位置表达，提取未加入测量噪声的真值。
        east = (d[14] - lon) * 6378137.0 * math.cos(lat)
        north = (d[13] - lat) * 6378137.0
        data["truth"].append([now(), east, north, d[16], d[10], d[9], d[12]])

    def on_spline(msg):
        data["splines"].append({"received": now(), "start": stamp_seconds(msg.start_time),
                                "id": msg.traj_id, "order": msg.order,
                                "knots": list(msg.knots),
                                "points": [[p.x, p.y, p.phi] for p in msg.control_points]})
        print(f"spline {msg.traj_id}: {len(msg.control_points)} control points", flush=True)

    def on_grid(msg):
        if not grid:
            grid.append(msg)

    def on_fix(msg):
        origin[:] = [msg.latitude, msg.longitude, msg.altitude]

    def on_finished(msg):
        nonlocal finished_at
        if msg.data and goal_time is not None and finished_at is None:
            finished_at = now()

    def on_event(msg):
        data["events"].append([now(), msg.data])
        print(f"controller: {msg.data}", flush=True)

    subscriptions = [
        node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray, "/USV_state", on_state, qos_profile_sensor_data),
        node.create_subscription(NavSatFix, "/fix", on_fix, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, "/channel_costmap", on_grid, latched),
        node.create_subscription(UsvBspline, "/planning/bspline", on_spline, 100),
        node.create_subscription(Twist, "/cmd", lambda m: data["commands"].append(
            [now(), m.linear.x, m.angular.z]), 100),
        node.create_subscription(Float64MultiArray, "/nmpc_tracking_diagnostics",
                                 lambda m: data["diagnostics"].append([now(), *m.data]), 100),
        node.create_subscription(String, "/nmpc_tracking_state", on_event, latched),
        node.create_subscription(Bool, "/USV_tracking_finished", on_finished, latched),
    ]
    publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)
    deadline = time.monotonic() + 20.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if data["odom"] and grid and origin and publisher.get_subscription_count():
            break
    else:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("地图、原点、里程计或目标订阅端未在 20 秒内就绪")

    # 发现订阅端后继续处理两秒回调，等待 DDS 数据端点完成匹配。
    ready_at = time.monotonic() + 2.0
    while time.monotonic() < ready_at:
        rclpy.spin_once(node, timeout_sec=0.1)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = args.east, args.north
    # RViz 目标箭头为局部 +x；此处只构造交互输入，船体状态保持原生 RFU。
    arrow_yaw = math.radians(args.phi + 90.0)
    goal.pose.orientation.z = math.sin(arrow_yaw / 2)
    goal.pose.orientation.w = math.cos(arrow_yaw / 2)
    goal_time = now()
    publisher.publish(goal)
    publisher.wait_for_all_acked(rclpy.duration.Duration(seconds=3.0))
    print(f"goal published: ({args.east}, {args.north}), phi={args.phi} deg", flush=True)
    next_report = time.monotonic() + 10.0
    try:
        while rclpy.ok() and now() - goal_time < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if finished_at is not None and now() - finished_at > 2:
                break
            if time.monotonic() >= next_report:
                row = data["truth"][-1] if data["truth"] else data["odom"][-1]
                print(f"t={now()-goal_time:.1f}s xy=({row[1]:.2f},{row[2]:.2f}) "
                      f"u={row[4]:.2f} splines={len(data['splines'])}", flush=True)
                next_report += 10.0
    except KeyboardInterrupt:
        pass
    finally:
        args.output.mkdir(parents=True, exist_ok=True)
        for key in ("odom", "truth", "commands", "diagnostics"):
            with (args.output / f"{key}.csv").open("w") as handle:
                writer = csv.writer(handle)
                writer.writerows(data[key])
        metadata = {"goal_time": goal_time, "end_time": now(), "finished_at": finished_at,
                    "goal": [args.east, args.north, args.phi], "origin": origin,
                    "splines": data["splines"], "events": data["events"],
                    "state_columns": ["ros_time", "x_E", "y_N", "phi", "u", "v", "r"],
                    "command_columns": ["ros_time", "V_cmd", "W_cmd"]}
        (args.output / "record.json").write_text(json.dumps(metadata, indent=2))
        msg = grid[0]
        np.savez_compressed(args.output / "costmap.npz",
                            cost=np.array(msg.data, dtype=np.int8).reshape(
                                msg.info.height, msg.info.width), resolution=msg.info.resolution,
                            origin=[msg.info.origin.position.x, msg.info.origin.position.y])
        node.destroy_node()
        rclpy.try_shutdown()
        del subscriptions


def load_csv(folder, name):
    """读取可为空的记录文件，保留单行数据的二维形状。"""
    path = folder / f"{name}.csv"
    return np.loadtxt(path, delimiter=",", ndmin=2) if path.stat().st_size else np.empty((0, 0))


def statistics(values):
    """统计误差的均方根、95 分位和最大绝对值。"""
    values = np.asarray(values)
    return {"rmse": float(np.sqrt(np.mean(values**2))),
            "p95_abs": float(np.percentile(np.abs(values), 95)),
            "max_abs": float(np.max(np.abs(values)))} if values.size else None


def plot(folder):
    """用实际收到的样条和同步诊断绘图，不将控制点误认为曲线端点。"""
    meta = json.loads((folder / "record.json").read_text())
    t0 = meta["goal_time"]
    odom, truth, cmd, diag = [load_csv(folder, key) for key in
                              ("odom", "truth", "commands", "diagnostics")]
    odom = odom[odom[:, 0] >= t0]
    truth = truth[truth[:, 0] >= t0] if truth.size else odom
    cmd = cmd[cmd[:, 0] >= t0]
    diag = diag[diag[:, 0] >= t0] if diag.size else diag
    curves = []
    for msg in meta["splines"]:
        knots, points, degree = msg["knots"], np.array(msg["points"]), msg["order"]
        spline = BSpline(knots, points, degree)
        times = np.linspace(knots[degree], knots[len(points)], 2001)
        curves.append(spline(times))
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
    else:
        font_name = "DejaVu Sans"
    plt.rcParams.update({"font.family": font_name, "axes.unicode_minus": False,
                         "font.size": 10, "figure.dpi": 150})
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), gridspec_kw={"width_ratios": [1.25, 1]})
    ax = axes[0, 0]
    xy = np.vstack([truth[:, 1:3], *[c[:, :2] for c in curves], [meta["goal"][:2]]])
    lo, hi = xy.min(axis=0) - 12, xy.max(axis=0) + 12
    costmap = np.load(folder / "costmap.npz")
    cost, res, origin = costmap["cost"], float(costmap["resolution"]), costmap["origin"]
    a = np.maximum(0, np.floor((lo - origin) / res).astype(int))
    b = np.minimum(cost.shape[::-1], np.ceil((hi - origin) / res).astype(int))
    colors = ListedColormap(["#e7f5e9", "#fff2cf", "#ffc8c8", "#555e68"])
    ax.imshow(cost[a[1]:b[1], a[0]:b[0]], origin="lower",
              extent=[origin[0]+a[0]*res, origin[0]+b[0]*res,
                      origin[1]+a[1]*res, origin[1]+b[1]*res],
              cmap=colors, norm=BoundaryNorm([-1, 5, 15, 75, 101], 4), alpha=0.8)
    for i, curve in enumerate(curves):
        ax.plot(curve[:, 0], curve[:, 1], "--", color="#3478b8", alpha=0.65,
                label="规划 B 样条" if i == 0 else None)
    ax.plot(odom[:, 1], odom[:, 2], color="#d5a0d2", linewidth=0.8, label="滤波 /odom")
    ax.plot(truth[:, 1], truth[:, 2], color="#cf402f", linewidth=1.6, label="船模实际真值")
    selected = np.linspace(0, len(truth)-1, 7, dtype=int)
    ax.quiver(truth[selected, 1], truth[selected, 2], -np.sin(truth[selected, 3]),
              np.cos(truth[selected, 3]), angles="xy", scale_units="xy", scale=0.35,
              color="#cf402f", width=0.006)
    ax.scatter(*meta["goal"][:2], marker="*", s=180, color="black", label="任务目标")
    ax.scatter(*truth[0, 1:3], marker="o", s=35, color="black", label="实际起点")
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel="东向 x_E / m", ylabel="北向 y_N / m",
           title="实际航道内闭环运行（0°北，−90°东）", aspect="equal")
    ax.legend(fontsize=8, loc="best")
    ax.text(0.02, 0.02, "底图：绿=同向0  黄=航道外10  红=对向20  灰=障碍100",
            transform=ax.transAxes, fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"})
    summary = {"finished": meta["finished_at"] is not None,
               "elapsed_s": meta["end_time"] - t0, "spline_count": len(curves),
               "finished_after_s": meta["finished_at"] - t0 if meta["finished_at"] else None,
               "final_truth_goal_distance_m": float(np.linalg.norm(truth[-1, 1:3]-meta["goal"][:2])),
               "events": [[t-t0, state] for t, state in meta["events"]]}
    ax = axes[1, 0]
    if curves:
        distance = np.full(len(truth), np.nan)
        for i, (msg, curve) in enumerate(zip(meta["splines"], curves)):
            end = meta["splines"][i+1]["received"] if i+1 < len(curves) else np.inf
            mask = (truth[:, 0] >= msg["received"]) & (truth[:, 0] < end)
            distance[mask] = cKDTree(curve[:, :2]).query(truth[mask, 1:3])[0]
        ax.plot(truth[:, 0]-t0, distance, label="真值至当时最新曲线最近距离", color="#cf402f")
        summary["truth_to_latest_published_curve_m"] = statistics(distance[np.isfinite(distance)])
    if diag.size:
        d = diag[:, 1:]
        sync_error = np.hypot(d[:, 16]-d[:, 4], d[:, 17]-d[:, 5])
        ax.plot(diag[:, 0]-t0, np.abs(d[:, 29]), alpha=0.7, label="同步诊断横向误差")
        summary["synchronous_lateral_error_m"] = statistics(d[:, 29])
        summary["synchronous_position_error_m"] = statistics(sync_error)
        summary["solver_success_rate"] = float(np.mean(d[:, 13] > 0.5))
        axes[0, 1].plot(diag[:, 0]-t0, d[:, 7], "--", label="ref u", color="#3478b8")
        axes[0, 1].plot(diag[:, 0]-t0, d[:, 8], "--", label="ref v", color="#e9a027")
    ax.axhline(0.3, color="black", linestyle=":", label="0.3 m")
    ax.set(xlabel="目标发布后时间 / s", ylabel="误差 / m", title="几何跟踪误差")
    ax.legend(fontsize=8)
    axes[0, 1].plot(truth[:, 0]-t0, truth[:, 4], label="实际 u", color="#3478b8")
    axes[0, 1].plot(truth[:, 0]-t0, truth[:, 5], label="实际 v", color="#e9a027")
    axes[0, 1].set(xlabel="时间 / s", ylabel="速度 / (m/s)", title="参考与实际船体速度")
    axes[0, 1].legend(fontsize=8)
    ax = axes[1, 1]
    ax.plot(cmd[:, 0]-t0, cmd[:, 1], color="#3478b8", label="V_cmd")
    ax.set(xlabel="时间 / s", ylabel="V_cmd / (m/s)", title="实际发布控制指令")
    other = ax.twinx()
    other.plot(cmd[:, 0]-t0, cmd[:, 2], color="#cf402f", label="W_cmd")
    other.set_ylabel("W_cmd / (rad/s)")
    ax.legend(loc="upper left", fontsize=8)
    other.legend(loc="upper right", fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    stop_count = sum(state.startswith("SAFE_STOP") for _, state in meta["events"])
    summary["safe_stop_count"] = stop_count
    result = "已完成" if summary["finished"] else "观察期内未完成"
    fig.suptitle(f"usv_planner → NMPC → 船模实际闭环 | {result} | {len(curves)} 条样条", fontsize=14)
    fig.tight_layout()
    fig.savefig(folder / "tracking_overview.png")
    plt.close(fig)
    (folder / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


def main():
    """解析目标与观察时长；重绘模式只读既有记录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--east", type=float, default=50.0)
    parser.add_argument("--north", type=float, default=0.0)
    parser.add_argument("--phi", type=float, default=-90.0, help="原生船首航向，单位度")
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--plot-only", action="store_true")
    args = parser.parse_args()
    if not args.plot_only:
        record(args)
    plot(args.output)


if __name__ == "__main__":
    main()
