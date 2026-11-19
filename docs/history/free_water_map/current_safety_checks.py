#!/usr/bin/env python3
"""真实河道节点：核验小船尺寸/圆圈/最高速度参数/独立当前距离停车。

最后重设虚拟船到本船后方是边界注入测试，不代表物理避碰工况。
脚本独立启动并停止整链路；推荐ROS_DOMAIN_ID=65避免与其他演示串扰。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from visualization_msgs.msg import MarkerArray


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def run(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    state = {}
    commands = []
    child = None
    rclpy.init()
    node = rclpy.create_node('current_safety_probe')
    for key, topic, kind in [
        ('own', '/odom', Odometry), ('target', '/target_boat/odom', Odometry),
        ('model', '/target_boat/markers', MarkerArray),
        ('rings', '/colregs/risk_markers', MarkerArray),
        ('path', '/hybrid_a_star_trajectory', RosPath),
    ]:
        node.create_subscription(kind, topic,
            lambda msg, key=key: state.__setitem__(key, msg), 20)
    node.create_subscription(Twist, '/cmd_vel',
        lambda msg: commands.append((time.monotonic(), msg.linear.x, msg.angular.z)), 20)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    target_cmd = node.create_publisher(Twist, '/target_boat/command', 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    errors = []
    report = {}
    with (output / 'current_safety.log').open('w') as log:
        try:
            child = subprocess.Popen([
                'ros2', 'launch', 'channel_navigation_manager', 'river_navigation.launch.py',
                'use_rviz:=false', 'use_virtual_boat:=true', 'target_x:=-150.0',
                'target_y:=-270.0', 'target_yaw:=0.0',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            start = time.monotonic()
            goal_sent = False
            while time.monotonic() - start < 45:
                rclpy.spin_once(node, timeout_sec=0.02)
                if not goal_sent and time.monotonic() - start > 9:
                    goal = PoseStamped()
                    goal.header.frame_id = 'odom'
                    goal.pose.position.x = -324.0
                    goal.pose.position.y = -101.0
                    goal.pose.orientation.z = math.sin(math.radians(110) / 2)
                    goal.pose.orientation.w = math.cos(math.radians(110) / 2)
                    goal_pub.publish(goal)
                    goal_sent = True
                if all(key in state for key in ['own', 'target', 'model', 'rings', 'path']):
                    if len(state['path'].poses) > 3 and commands and commands[-1][1] > 0.5:
                        break
            if not all(key in state for key in ['own', 'target', 'model', 'rings', 'path']):
                raise RuntimeError('missing actual ROS inputs')
            hull = next(m for m in state['model'].markers if m.ns == 'virtual_boat_hull')
            report['virtual_dimensions'] = [hull.scale.x, hull.scale.y]
            report['virtual_speed'] = state['target'].twist.twist.linear.x
            if report['virtual_dimensions'] != [1.0, 1.5]: errors.append('wrong hull dimensions')
            if abs(report['virtual_speed'] - 2.0) > 0.01: errors.append('wrong target speed')
            expected = dict(range_monitor=80, range_action=48, range_emergency=20,
                dcpa_monitor=10, dcpa_action=9, dcpa_emergency=8,
                avoidance_range=9, hard_separation=7.5, current_stop=8)
            radii = {}
            for marker in state['rings'].markers:
                if marker.ns in expected and marker.points:
                    radii[marker.ns] = (max(p.x for p in marker.points) -
                        min(p.x for p in marker.points)) / 2
            report['actual_ring_radii'] = radii
            for key, radius in expected.items():
                if key not in radii or abs(radii[key] - radius) > 1e-6:
                    errors.append('wrong ring: ' + key)
            if not commands or commands[-1][1] <= 0.5: errors.append('no movement before injection')
            # 目标已在后方且静止，TCPA<0。应由8m距离保护而非未来CPA碰撞停车。
            own = state['own'].pose.pose
            yaw = math.atan2(2 * own.orientation.w * own.orientation.z,
                1 - 2 * own.orientation.z ** 2)
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'odom'
            initial.pose.pose.position.x = own.position.x - 7.0 * math.cos(yaw)
            initial.pose.pose.position.y = own.position.y - 7.0 * math.sin(yaw)
            initial.pose.pose.orientation.w = 1.0
            injection = time.monotonic()
            while time.monotonic() - injection < 1.5:
                target_cmd.publish(Twist())
                initial_pub.publish(initial)
                rclpy.spin_once(node, timeout_sec=0.02)
            stopped = [c for c in commands if c[0] > injection + 0.5]
            report['stop_command_samples'] = len(stopped)
            if len(stopped) < 5 or any(abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9 for c in stopped):
                errors.append('current-distance STOP failed')
        except Exception as error:
            errors.append(str(error))
        finally:
            if child is not None:
                os.killpg(child.pid, signal.SIGINT)
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
    if 'Target inside 8.0m current-distance stop circle' not in (output / 'current_safety.log').read_text():
        errors.append('independent guard branch not exercised')
    report.update(passed=not errors, errors=errors)
    (output / 'current_safety.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    return 0 if not errors else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/home/l/work_ws/test_results/current')
    raise SystemExit(run(parser.parse_args().output))
