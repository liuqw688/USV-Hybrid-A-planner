#!/usr/bin/env python3
"""隔离验证规划器持续发布、新轨迹切换和障碍失效停车；反馈直接取样条真值。"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.interpolate import BSpline
from std_msgs.msg import Float64MultiArray, String
from usv_interfaces.msg import UsvBspline


def run(planner_executable):
    """启动真实规划器与 NMPC，检查通信契约，不评估船体跟踪误差。"""
    rclpy.init()
    node = rclpy.create_node("test_reference_stream")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, "/test_map", latched)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
    messages, states, diagnostics, commands = [], [], [], []
    curves = {}

    def receive_spline(msg):
        messages.append((time.monotonic(), msg))
        if msg.traj_id not in curves:
            curves[msg.traj_id] = BSpline(
                msg.knots, [[p.x, p.y, p.phi] for p in msg.control_points], msg.order)

    subscriptions = [
        node.create_subscription(UsvBspline, "/planning/bspline", receive_spline, 100),
        node.create_subscription(String, "/nmpc_tracking_state",
                                 lambda m: states.append(m.data), latched),
        node.create_subscription(Float64MultiArray, "/nmpc_tracking_diagnostics",
                                 lambda m: diagnostics.append(list(m.data)), 100),
        node.create_subscription(Twist, "/cmd", lambda m: commands.append(m), 100),
    ]

    def step(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.child_frame_id = "usv_body_rfu"
            odom.header.stamp = node.get_clock().now().to_msg()
            phi = -math.pi / 2
            if messages:
                msg = messages[-1][1]
                curve = curves[msg.traj_id]
                elapsed = node.get_clock().now().nanoseconds * 1e-9 - (
                    msg.start_time.sec + msg.start_time.nanosec * 1e-9)
                t = min(curve.t[-msg.order - 1], curve.t[msg.order] + max(0., elapsed))
                x, y, phi = curve(t)
                dx, dy, r = curve(t, 1)
                odom.pose.pose.position.x = float(x)
                odom.pose.pose.position.y = float(y)
                odom.twist.twist.linear.y = float(-math.sin(phi) * dx + math.cos(phi) * dy)
                odom.twist.twist.linear.x = float(math.cos(phi) * dx + math.sin(phi) * dy)
                odom.twist.twist.angular.z = float(r)
            odom.pose.pose.orientation.z = math.sin(phi / 2)
            odom.pose.pose.orientation.w = math.cos(phi / 2)
            odom_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.02)

    def goal(east, north):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = east, north
        msg.pose.orientation.w = 1.0  # RViz 箭头局部 +x 向东，对应船首 phi=-pi/2。
        goal_pub.publish(msg)

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = grid.info.height = 400
    grid.info.resolution = 1.0
    grid.info.origin.position.x = grid.info.origin.position.y = -100.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 160000
    grid_pub.publish(grid)
    processes = []
    with tempfile.TemporaryDirectory(prefix="usv_reference_stream_") as output:
        logs = []
        try:
            planner_config = Path(get_package_share_directory("usv_planner")) / "config/usv_planner.yaml"
            nmpc_config = Path(get_package_share_directory("usv_nmpc_controller")) / "config/usv_nmpc.yaml"
            nmpc_exe = Path(get_package_prefix("usv_nmpc_controller")) / "lib/usv_nmpc_controller/usv_nmpc_node"
            for name, command in [
                ("planner", [str(planner_executable), "--ros-args", "--params-file", str(planner_config),
                             "-p", "sdf_map/grid_topic:=/test_map", "-r", "/sdf_map/odom:=/odom"]),
                ("nmpc", [str(nmpc_exe), "--ros-args", "--params-file", str(nmpc_config),
                          "-p", "nmpc.horizon_steps:=10", "-p", "terminal_table.enabled:=false"]),
            ]:
                log = open(Path(output) / f"{name}.log", "w+")
                logs.append(log)
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
            step(3.0)
            goal(140.0, 0.0)
            step(6.0)
            assert len(messages) >= 30, f"参考发布不足: {len(messages)}"
            assert states[-1] == "TRACKING", states
            assert not any("SAFE_STOP" in s for s in states), states
            first_id = messages[0][1].traj_id
            repeats = [m for _, m in messages if m.traj_id == first_id]
            assert len(repeats) >= 10 and all(m == repeats[0] for m in repeats)
            assert diagnostics[-1][4] > diagnostics[0][4] + 0.5, "参考时间没有向前推进"

            goal(140.0, 8.0)
            step(2.0)
            assert messages[-1][1].traj_id > first_id, "新轨迹未切换"
            assert states[-1] == "TRACKING" and not any("SAFE_STOP" in s for s in states), states
            gaps = [b[0] - a[0] for a, b in zip(messages, messages[1:])]
            assert max(gaps) < 0.5, f"参考流中断: {max(gaps):.3f} s"
            print(f"PASS: {len(messages)} 条参考，最大间隔 {max(gaps):.3f} s；新轨迹连续接管。")

            grid.data = [100] * 160000
            grid.header.stamp = node.get_clock().now().to_msg()
            grid_pub.publish(grid)
            step(0.8)
            count = len(messages)
            step(2.5)
            assert len(messages) == count, "障碍失效后仍在重发旧轨迹"
            assert states[-1] == "WAIT_REFERENCE", states
            assert commands[-1].linear.x == 0.0 and commands[-1].angular.z == 0.0
            print("PASS: 障碍使旧轨迹失效后停止发布，NMPC 接收撤销通知后输出零指令并等待新参考。")
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-8000:])
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()
            del subscriptions


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner-executable", type=Path)
    args = parser.parse_args()
    # 使用独立 DDS 域，避免测试输入影响正在运行的船模或控制器。
    os.environ["ROS_DOMAIN_ID"] = "91"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    executable = args.planner_executable or (
        Path(get_package_prefix("usv_planner")) / "lib/usv_planner/usv_planner_node")
    run(executable)
