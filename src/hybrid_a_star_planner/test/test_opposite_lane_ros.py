#!/usr/bin/env python3
"""隔离域187：真实航道节点 + Hybrid节点，短机动、跨道掉头、硬墙回归。"""
import os
os.environ['ROS_DOMAIN_ID'] = '187'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/usv_opposite_lane_logs'
import json
import math
from pathlib import Path as FilePath
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(p):
    return 2 * math.atan2(p.orientation.z, p.orientation.w)


def main():
    rclpy.init()
    node = rclpy.create_node('opposite_lane_fixture')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, '/ol/map', qos)
    land_pub = node.create_publisher(OccupancyGrid, '/ol/land', qos)
    chart_pub = node.create_publisher(String, '/ol/chart', qos)
    goal_pub = node.create_publisher(PoseStamped, '/ol/goal', qos)
    route_pub = node.create_publisher(Path, '/ol/route', qos)
    local_goal_pub = node.create_publisher(PoseStamped, '/ol/local_goal', qos)
    odom_pub = node.create_publisher(Odometry, '/odom', 20)
    paths, profiles, lane_maps, statuses = [], [], [], []
    node.create_subscription(Path, '/ol/path', paths.append, qos)
    node.create_subscription(TwistStamped, '/hybrid_a_star/speed_limit', profiles.append, qos)
    node.create_subscription(OccupancyGrid, '/ol/lane_map', lane_maps.append, qos)
    node.create_subscription(String, '/hybrid_a_star/planning_status', lambda m: statuses.append(m.data), qos)
    processes, logs, report = [], [], []
    boat = [0., -10., 0., 4.]
    mission_stamp = [0]

    def start(package, executable, args):
        prefix = get_package_prefix(package)
        log = open('/tmp/usv_opposite_lane_' + package + '.log', 'w')
        logs.append(log)
        processes.append(subprocess.Popen([prefix + '/lib/' + package + '/' + executable,
            '--ros-args', *args], stdout=log, stderr=log))

    def spin(seconds=.025):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            m = Odometry()
            m.header.frame_id = 'odom'; m.header.stamp = node.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = boat[:2]
            m.pose.pose.orientation.z = math.sin(boat[2] / 2)
            m.pose.pose.orientation.w = math.cos(boat[2] / 2)
            m.twist.twist.linear.x = boat[3]
            odom_pub.publish(m)
            rclpy.spin_once(node, timeout_sec=.02)

    def goal(gx, gy, direction):
        m = PoseStamped(); m.header.frame_id = 'odom'
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x = gx; m.pose.position.y = gy
        m.pose.orientation.z = math.sin(direction / 2); m.pose.orientation.w = math.cos(direction / 2)
        goal_pub.publish(m)
        # Allow the manager to publish the new task's lane roles before the local goal.
        spin(.25)
        m.header.stamp = node.get_clock().now().to_msg()
        mission_stamp[0] = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        paths.clear(); profiles.clear(); statuses.clear()
        route = Path(); route.header = m.header; route.poses = [m]
        route_pub.publish(route); local_goal_pub.publish(m)

    def lane_value(y):
        m = lane_maps[-1]
        x = int((0 - m.info.origin.position.x) / m.info.resolution)
        iy = int((y - m.info.origin.position.y) / m.info.resolution)
        return m.data[iy * m.info.width + x]

    def wait_path(gx, gy, direction):
        begin = time.monotonic()
        while time.monotonic() - begin < 7:
            spin()
            for path in reversed(paths):
                stamp = path.header.stamp.sec * 1000000000 + path.header.stamp.nanosec
                if stamp < mission_stamp[0] or len(path.poses) < 2:
                    continue
                end = path.poses[-1].pose
                if math.hypot(end.position.x - gx, end.position.y - gy) >= 1.01:
                    continue
                if abs(angle(yaw(end) - direction)) > math.pi / 6 + 1e-6:
                    continue
                profile = next((p for p in reversed(profiles) if p.header == path.header), None)
                if profile:
                    return path, profile.twist.linear.x, time.monotonic() - begin
        raise AssertionError(('No valid paired path', boat, gx, gy, direction, statuses[-5:]))

    try:
        prefix = get_package_prefix('hybrid_a_star_planner')
        start('channel_navigation_manager', 'channel_navigation_node', [
            '-p', 'lane_data_topic:=/ol/chart', '-p', 'costmap_topic:=/ol/map',
            '-p', 'goal_topic:=/ol/goal', '-p', 'lane_costmap_topic:=/ol/lane_map'])
        start('hybrid_a_star_planner', 'hybrid_a_star_node', [
            '--params-file', prefix + '/share/hybrid_a_star_planner/config/hybrid_a_star_params.yaml',
            '-p', 'costmap_topic:=/ol/map', '-p', 'channel.lane_costmap_topic:=/ol/lane_map',
            '-p', 'channel.static_land_topic:=/ol/land', '-p', 'trajectory_topic:=/ol/path',
            '-p', 'goal_topic:=/ol/local_goal', '-p', 'global_route_topic:=/ol/route'])
        grid = OccupancyGrid(); grid.header.frame_id = 'odom'
        grid.info.width = 240; grid.info.height = 56; grid.info.resolution = 1.
        grid.info.origin.position.x = -120.; grid.info.origin.position.y = -28.
        grid.info.origin.orientation.w = 1.; grid.data = [0] * (240 * 56)
        chart = String(); chart.data = json.dumps({'frame_id': 'odom', 'lanes': [
            {'lane_id': 1, 'orient_deg': 90., 'points': [[-120, -24], [120, -24], [120, 0], [-120, 0], [-120, -24]]},
            {'lane_id': 2, 'orient_deg': 270., 'points': [[-120, 0], [120, 0], [120, 24], [-120, 24], [-120, 0]]}]})
        grid_pub.publish(grid); land_pub.publish(grid); chart_pub.publish(chart); spin(1.)
        # Both original physical lanes, 10/20m, right/left/rear goal heading.
        for side in [-1., 1.]:
            boat[:] = [0., side * 10., 0. if side < 0 else math.pi, 4.]
            for distance in [10., 20.]:
                for delta in [-math.pi / 2, math.pi / 2, math.pi]:
                    spin(.3); paths.clear(); profiles.clear(); statuses.clear()
                    gx = distance * math.cos(boat[2]); gy = boat[1]; direction = angle(boat[2] + delta)
                    goal(gx, gy, direction)
                    assert lane_maps and lane_value(boat[1]) <= 20, 'Same-lane short maneuver swapped roles'
                    path, speed, delay = wait_path(gx, gy, direction)
                    report.append({'kind': 'short', 'lane_side': side, 'distance': distance,
                        'heading_change': delta, 'speed': speed, 'delay': delay, 'poses': len(path.poses)})
        for heading_error in [-.35, 0., .35]:
            boat[:] = [0., -10., 0., 4.]; spin(.3); paths.clear(); profiles.clear()
            direction = angle(math.pi + heading_error)
            goal(-20., 10., direction)
            assert lane_value(10.) <= 20 and lane_value(-10.) >= 65, 'Rear target lane did not become main'
            path, speed, delay = wait_path(-20., 10., direction)
            first = next(angle(yaw(p.pose) - boat[2]) for p in path.poses if abs(angle(yaw(p.pose) - boat[2])) > .08)
            assert first > 0, ('Turned away from the left target lane', first)
            report.append({'kind': 'rear_adjacent', 'heading_error': heading_error,
                'speed': speed, 'delay': delay, 'first_turn': first, 'poses': len(path.poses)})
        # Hard wall blocks every speed; lane preference cannot authorize crossing land.
        for y in range(56):
            grid.data[y * 240 + 115] = 100
        grid_pub.publish(grid); land_pub.publish(grid); spin(.3)
        paths.clear(); statuses.clear(); goal(-20., 10., math.pi)
        spin(4.)
        assert 'blocked' in statuses and paths and not paths[-1].poses, 'Hard wall was not respected'
        FilePath('/tmp/usv_opposite_lane_results.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print('PASS: 12 short missions, 3 rear lane transfers, paired low-speed profiles, hard wall rejection')
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for log in logs:
            log.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
