#!/usr/bin/env python3
"""隔离ROS域的回归：末航点无解搜索期间旧安全路径持续发布，撤销任务立即清空。

source install/setup.bash 后运行；不启动/改变用户的RViz或仿真节点。
"""
import math
import os
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/usv_path_freshness_logs'
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_prefix


def main():
    rclpy.init()
    node = rclpy.create_node('path_freshness_fixture')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/freshness_map', qos)
    route_pub = node.create_publisher(Path, '/freshness_route', qos)
    goal_pub = node.create_publisher(PoseStamped, '/freshness_goal', qos)
    odom_pub = node.create_publisher(Odometry, '/odom', 20)
    tf = TransformBroadcaster(node)
    received = []
    statuses = []
    node.create_subscription(String, '/hybrid_a_star/planning_status',
                             lambda m: statuses.append(m.data), qos)
    node.create_subscription(Path, '/freshness_path',
                             lambda p: received.append((time.monotonic(), len(p.poses))), qos)
    log = open('/tmp/usv_path_freshness_ros.log', 'w')
    process = subprocess.Popen([
        get_package_prefix('hybrid_a_star_planner') + '/lib/hybrid_a_star_planner/hybrid_a_star_node', '--ros-args',
        '-p', 'channel.enabled:=false', '-p', 'costmap_topic:=/freshness_map',
        '-p', 'goal_topic:=/freshness_goal', '-p', 'trajectory_topic:=/freshness_path',
        '-p', 'global_route_topic:=/freshness_route', '-p', 'search_time_limit:=2.0',
        '-p', 'planner_frequency:=1.0',
        '-p', 'max_iterations:=250000', '-p', 'base_frame:=freshness_boat',
        '-p', 'usv.gentle_turn_sampling:=false', '-p', 'step_size:=1.0',
        '-p', 'cruise_speed:=4.0', '-p', 'prediction.goal_braking:=false'], stdout=log, stderr=log)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = 4.0
            odom_pub.publish(odom)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = 'freshness_boat'
            transform.transform.rotation.w = 1.0
            tf.sendTransform(transform)
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.header.stamp = node.get_clock().now().to_msg()
        grid.info.resolution = 1.0
        grid.info.width, grid.info.height = 100, 10
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 1000
        map_pub.publish(grid)
        first = PoseStamped()
        first.header = grid.header
        first.pose.position.x = 30.0
        first.pose.orientation.w = 1.0
        last = PoseStamped()
        last.header = grid.header
        last.pose.position.x = 90.0
        last.pose.orientation.z = math.sin(math.pi / 2)
        last.pose.orientation.w = math.cos(math.pi / 2)
        route = Path()
        route.header = grid.header
        route.poses = [first, last]
        route_pub.publish(route)
        goal_pub.publish(first)
        spin_for(8.0)
        assert any(n >= 2 for _, n in received), 'No initial straight path'
        assert 'tracking' in statuses, 'Planner tracking status missing'
        received.clear()
        goal_pub.publish(last)
        start = time.monotonic()
        spin_for(9.0)
        valid = [t for t, n in received if n >= 2]
        assert len(valid) >= 6, ('Insufficient safe path heartbeats', received)
        gaps = [b - a for a, b in zip([start] + valid, valid + [time.monotonic()])]
        assert max(gaps) < 2.5, ('Path heartbeat gap exceeds MPC timeout margin', gaps)
        assert not any(n == 0 for _, n in received), 'Safe old path cleared during final-leg search'
        received.clear()
        route.poses = []
        route_pub.publish(route)
        spin_for(2.0)
        assert any(n == 0 for _, n in received), 'Mission cancellation did not clear path'
        assert statuses[-1] == 'cancelled', ('Wrong cancellation status', statuses)
        print(f'PASS: safe path heartbeat during infeasible final-leg searches; max gap={max(gaps):.3f}s; cancellation brakes')
    finally:
        process.terminate()
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
