#!/usr/bin/env python3
"""隔离验证受阻 Path 前沿的安全选点及绕障；里程计直接反馈已发布样条。"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.interpolate import BSpline
from scipy.ndimage import distance_transform_edt
from usv_interfaces.msg import UsvBspline


def run(grid, points, initial_state, output, duration=75.0):
    """运行真实规划器，保存样条、船位及选点日志；不评估控制器跟踪性能。"""
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("test_blocked_path")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, "/test_map", latched)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    path_pub = node.create_publisher(PathMsg, "/path", 10)
    curves, samples, positions = {}, [], []
    current = None
    state = np.array(initial_state, dtype=float)
    occupied = np.array(grid.data).reshape(grid.info.height, grid.info.width) >= 75
    clearance = distance_transform_edt(~occupied) * grid.info.resolution
    minimum_clearance = float("inf")

    def receive_spline(msg):
        nonlocal current, minimum_clearance
        if msg.traj_id not in curves:
            curve = BSpline(msg.knots, [[p.x, p.y, p.phi] for p in msg.control_points], msg.order)
            times = np.linspace(curve.t[msg.order], curve.t[-msg.order - 1],
                                max(2, math.ceil((curve.t[-msg.order - 1] - curve.t[msg.order]) / 0.02)))
            poses = curve(times)
            indices = np.floor((poses[:, :2] - [grid.info.origin.position.x,
                                               grid.info.origin.position.y]) / grid.info.resolution).astype(int)
            assert np.all((indices[:, 0] >= 0) & (indices[:, 0] < grid.info.width))
            assert np.all((indices[:, 1] >= 0) & (indices[:, 1] < grid.info.height))
            distances = clearance[indices[:, 1], indices[:, 0]]
            assert np.all(distances >= 3.0), f"样条侵入 3 m 膨胀区: {distances.min():.2f} m"
            minimum_clearance = min(minimum_clearance, float(distances.min()))
            curves[msg.traj_id] = curve
            samples.append(poses.tolist())
        current = msg

    subscription = node.create_subscription(UsvBspline, "/planning/bspline", receive_spline, 100)

    def step(seconds):
        nonlocal state
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if current is not None:
                curve = curves[current.traj_id]
                elapsed = node.get_clock().now().nanoseconds * 1e-9 - (
                    current.start_time.sec + current.start_time.nanosec * 1e-9)
                t = np.clip(curve.t[current.order] + elapsed,
                            curve.t[current.order], curve.t[-current.order - 1])
                x, y, phi = curve(t)
                dx, dy, r = curve(t, 1)
                state = np.array([x, y, phi, -math.sin(phi) * dx + math.cos(phi) * dy,
                                  math.cos(phi) * dx + math.sin(phi) * dy, r])
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.child_frame_id = "usv_body_rfu"
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.pose.pose.position.x, odom.pose.pose.position.y = state[:2].tolist()
            odom.pose.pose.orientation.z = math.sin(state[2] / 2)
            odom.pose.pose.orientation.w = math.cos(state[2] / 2)
            odom.twist.twist.linear.y, odom.twist.twist.linear.x = state[3:5].tolist()
            odom.twist.twist.angular.z = float(state[5])
            odom_pub.publish(odom)
            positions.append(state.tolist())
            rclpy.spin_once(node, timeout_sec=0.02)

    executable = Path(get_package_prefix("usv_planner")) / "lib/usv_planner/usv_planner_node"
    config = Path(get_package_share_directory("usv_planner")) / "config/usv_planner.yaml"
    command = [str(executable), "--ros-args", "--params-file", str(config),
               "-p", "goal/input_mode:=2", "-p", "sdf_map/grid_topic:=/test_map",
               "-p", "search/shot_trigger_distance:=30.0", "-r", "/sdf_map/odom:=/odom"]
    grid_pub.publish(grid)
    with open(output / "planner.log", "w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            step(3.0)
            assert process.poll() is None, "规划器启动失败"
            path = PathMsg()
            path.header.frame_id = "map"
            path.header.stamp = node.get_clock().now().to_msg()
            for x, y in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            assert path_pub.get_subscription_count() >= 1, "未发现 Path 订阅者"
            path_pub.publish(path)
            started = time.monotonic()
            while time.monotonic() - started < duration:
                step(0.5)
                assert process.poll() is None, "规划器异常退出"
                # 只检验绕过首个障碍，不要求此短测跑完整条任务。
                if np.linalg.norm(state[:2] - initial_state[:2]) >= 90.0:
                    break
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            del subscription

    text = (output / "planner.log").read_text()
    targets = [(float(p), float(x), float(y), terminal == "true")
               for p, x, y, terminal in re.findall(
                   r"Path rolling target: progress=([\d.]+) m, range=[\d.]+ m, "
                   r"goal=\(([-\d.]+), ([-\d.]+)\), terminal=(true|false)", text)]
    result = dict(unique_splines=len(curves), minimum_grid_clearance_m=minimum_clearance,
                  final_state=state.tolist(), targets=targets, positions=positions,
                  splines=samples, points=np.asarray(points).tolist())
    (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    assert "Blocked Path frontier" in text, "没有覆盖前沿受阻分支"
    assert curves, "没有生成可执行样条"
    assert targets and targets[-1][0] > targets[0][0] + 60.0, "绕障后航路进度没有继续推进"
    assert all(b[0] >= a[0] for a, b in zip(targets, targets[1:])), "航路进度倒退"
    print(json.dumps({k: result[k] for k in
                      ("unique_splines", "minimum_grid_clearance_m", "final_state")}, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/usv_blocked_path_test"))
    parser.add_argument("--domain-id", type=int, default=193)
    args = parser.parse_args()
    # 独立 DDS 域只连接本脚本创建的规划器，不向运行中的船模发布输入。
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width, grid.info.height = 320, 440
    grid.info.resolution = 0.5
    grid.info.origin.position.x, grid.info.origin.position.y = -80.0, -40.0
    grid.info.origin.orientation.w = 1.0
    east = -80.0 + (np.arange(grid.info.width) + 0.5) * grid.info.resolution
    north = -40.0 + (np.arange(grid.info.height) + 0.5) * grid.info.resolution
    mask = east[None, :] ** 2 + (north[:, None] - 60.0) ** 2 <= 6.0 ** 2
    grid.data = np.where(mask, 100, 0).astype(np.int8).ravel().tolist()
    run(grid, [(0., 0.), (0., 60.), (0., 150.)], np.zeros(6), args.output)
