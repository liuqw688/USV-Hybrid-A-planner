"""隔离 ROS Domain 验证两个真实 C++ 节点；不读取或覆盖正式海图."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from usv_interfaces.msg import ObstacleDetection, ObstacleDetectionArray
from visualization_msgs.msg import MarkerArray


def test_channel_center_gradient(tmp_path):
    """验证真实节点的横断面梯度，同时保留航道外、对向与障碍优先级。"""
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    node = rclpy.create_node('channel_center_regression', context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    chart = node.create_publisher(OccupancyGrid, '/chart_costmap', qos)
    lanes = node.create_publisher(String, '/chart_lane_data', qos)
    odom = node.create_publisher(Odometry, '/odom', 10)
    goal = node.create_publisher(PoseStamped, '/goal_pose', 10)
    path = node.create_publisher(NavPath, '/path', 10)
    grids = []
    subscription = node.create_subscription(OccupancyGrid, '/channel_costmap', grids.append, qos)
    executable = (Path(get_package_prefix('channel_navigation_manager')) /
                  'lib/channel_navigation_manager/channel_navigation_node')
    with (tmp_path / 'center_gradient.log').open('w') as log:
        process = subprocess.Popen([str(executable)], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while chart.get_subscription_count() == 0 or odom.get_subscription_count() == 0:
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.04)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.resolution = 1.0
            grid.info.width = grid.info.height = 50
            grid.info.origin.position.x = grid.info.origin.position.y = -25.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0]*2500
            grid.data[25*50+5] = 100
            chart.publish(grid)
            lanes.publish(String(data=json.dumps({'frame_id': 'map', 'lanes': [
                {'points': [[-8, -20], [8, -20], [8, 20], [-8, 20]], 'orient_deg': 0, 'direction': 'CW'},
                {'points': [[10, -20], [22, -20], [22, 20], [10, 20]], 'orient_deg': 180, 'direction': 'CCW'}
            ]})))
            state = Odometry()
            state.header.frame_id = 'map'
            state.pose.pose.orientation.w = 1.0  # 原生航向 phi=0，船首朝北。
            odom.publish(state)
            indices = [25*50+x for x in (24, 30, 32, 34, 41, 5)]
            expected = [1, 6, 8, 20, 21, 100]
            while not grids or [grids[-1].data[i] for i in indices] != expected:
                assert time.monotonic() < deadline, (
                    [grids[-1].data[i] for i in indices] if grids else [],
                    (tmp_path / 'center_gradient.log').read_text())
                executor.spin_once(timeout_sec=0.04)
            target = PoseStamped()
            target.header.frame_id = 'map'
            target.pose.position.y = -20.0
            goal.publish(target)
            while grids[-1].data[indices[0]] < 20:
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.04)
            route = NavPath()
            route.header.frame_id = 'map'
            route.poses = [PoseStamped(), PoseStamped()]
            route.poses[1].pose.position.y = 20.0
            path.publish(route)
            while grids[-1].data[indices[0]] >= 20:
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.04)
            # 始终保留任务航道方向，即使船首暂时转向相反方向。
            state.pose.pose.orientation.z = 1.0
            state.pose.pose.orientation.w = 0.0
            odom.publish(state)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.04)
            assert grids[-1].data[indices[0]] < 20
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            node.destroy_subscription(subscription)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
            if previous_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = previous_domain


def test_perception_pipeline(tmp_path):
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    node = rclpy.create_node('perception_regression_probe', context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    chart_pub = node.create_publisher(OccupancyGrid, '/chart_costmap', qos)
    lanes_pub = node.create_publisher(String, '/chart_lane_data', qos)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    detections_pub = node.create_publisher(ObstacleDetectionArray, '/obstacle_detections', 1)
    grids, scans, markers = [], [], []
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/channel_costmap', grids.append, qos),
        node.create_subscription(ObstacleDetectionArray, '/obstacle_detections', scans.append, 10),
        node.create_subscription(MarkerArray, '/perception_markers', markers.append, qos),
    ]
    static_tf = StaticTransformBroadcaster(node)
    dynamic_tf = TransformBroadcaster(node)
    prefix = Path(get_package_prefix('channel_navigation_manager')) / 'lib/channel_navigation_manager'
    processes, logs = [], []
    passed = []

    def start(executable, *args):
        log = (tmp_path / f'{executable}.log').open('w')
        logs.append(log)
        process = subprocess.Popen([str(prefix / executable), '--ros-args', *args],
                                   stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def until(predicate, timeout=6):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (
                'Timed out', {p.args[0]: p.poll() for p in processes},
                [p.read_text() for p in tmp_path.glob('*.log')],
                list(grids[-1].data) if grids else [])
            executor.spin_once(timeout_sec=0.04)

    def drain(seconds=0.25):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def expect(name, samples):
        try:
            until(lambda: grids and all(grids[-1].data[i] == value for i, value in samples.items()))
        except AssertionError:
            actual = {i: grids[-1].data[i] for i in samples} if grids else None
            raise AssertionError(f'{name}: expected {samples}, got {actual}') from None
        passed.append(name)

    def rectangle(x0, y0, x1, y1):
        return [[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]

    def ship(x, y, yaw=-math.pi / 2):
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.header.stamp = stamp
        transform.child_frame_id = 'usv_body_rfu'
        transform.transform.translation.x = float(x - 10)
        transform.transform.translation.y = float(y + 20)
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        dynamic_tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom_pub.publish(odom)
        return stamp

    def report(*items, frame='usv_body_rfu', stamp=None):
        message = ObstacleDetectionArray()
        message.header.frame_id = frame
        message.header.stamp = stamp or node.get_clock().now().to_msg()
        message.obstacles = list(items)
        detections_pub.publish(message)
        return message

    def sensed_at(x, y, radius=1.0):
        # 船首朝东、船位 (2,2)，其 RFU 右向为南。
        dx, dy = x - 2, y - 2
        obstacle = ObstacleDetection()
        obstacle.label = 'UNKNOWN'
        obstacle.bearing_rad = math.atan2(-dy, dx)
        obstacle.range_m = math.hypot(dx, dy)
        obstacle.radius_m = radius
        return obstacle

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)

    try:
        manager = start('channel_navigation_node', '-p', 'detection_timeout_s:=2.0',
                        '-p', 'lane_center_cost:=0')
        simulator = start('obstacle_perception_sim_node')
        until(lambda: chart_pub.get_subscription_count() == 1 and
              lanes_pub.get_subscription_count() == 2 and odom_pub.get_subscription_count() == 2 and
              detections_pub.get_subscription_count() == 2)  # 管理器和测试观测器。
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = 10.0
        transform.transform.translation.y = -20.0
        transform.transform.rotation.w = 1.0
        static_tf.sendTransform(transform)
        lanes = [{'points': rectangle(0, 0, 12, 6), 'orient_deg': 90},
                 {'points': rectangle(0, 6, 12, 12), 'orient_deg': 270}]
        reefs = [rectangle(2.8, 2.8, 5.2, 5.2), rectangle(16.8, 2.8, 19.2, 5.2),
                 rectangle(62, 1, 64, 3)]
        payload = {'frame_id': 'map', 'lanes': lanes,
                   'obstacles': [{'kind': 'REEF', 'points': p} for p in reefs]}
        lanes_pub.publish(String(data=json.dumps(payload)))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 2.0
        grid.info.width, grid.info.height = 16, 8
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 128
        grid.data[0] = 100
        chart_pub.publish(grid)
        expect('no_odometry_and_truth_does_not_create_obstacles', {0: 100, 18: 20, 20: 20})
        stamp = ship(2, 2)
        until(lambda: scans and len(scans[-1].obstacles) == 3)
        expect('detected_reefs_overlay_channel_and_open_water', {0:100, 21:0, 68:20, 18:100, 25:100})
        edge = scans[-1].obstacles[-1]
        assert edge.range_m > 60.0  # 最近边缘刚进入圆，重心可以在圆外。
        assert edge.label == 'REEF'
        assert set(ObstacleDetection.get_fields_and_field_types()) == {
            'label', 'bearing_rad', 'range_m', 'radius_m'}
        assert abs(scans[-1].obstacles[0].radius_m - math.hypot(1.2, 1.2)) < 1e-6
        assert scans[-1].header.frame_id == 'usv_body_rfu'
        assert scans[-1].header.stamp == stamp
        assert abs(scans[-1].obstacles[0].bearing_rad + math.pi / 4) < 1e-6
        until(lambda: markers and any(m.ns == 'perception_radius' for m in markers[-1].markers))
        passed.append('rfu_bearing_timestamp_edge_detection_and_independent_markers')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = -100.0
        goal.pose.position.y = 2.0
        goal_pub.publish(goal)
        expect('goal_reverses_lane_costs_without_changing_detections', {21:20, 68:0, 18:100})
        ship(200, 200)
        expect('leaving_circle_restores_lane_and_water_cost', {0:100, 18:20, 25:20})
        until(lambda: scans and not scans[-1].obstacles)
        # 仿真消失后，用从未在海图中出现的目标验证消费者确实只依赖话题。
        stop(simulator)
        stamp = ship(2, 2)
        goal.pose.position.x = 100.0
        goal_pub.publish(goal)
        expect('normal_lane_costs_after_simulator_stops', {20:0, 68:20})
        drain()
        normal_costs = list(grids[-1].data)
        stamp = ship(2, 2)
        report(sensed_at(9, 3), stamp=stamp)
        # 第 22 格接触航道多边形边界，应保留航道代价。
        expect('external_unmapped_target_needs_no_truth_lookup', {20:100, 18:0, 22:0, 23:20, 25:20})
        # 大小 1 m + 膨胀 2 m = 3 m，逐格验证圆形范围及外部原始代价。
        for index, value in enumerate(grids[-1].data):
            x = (index % 16 + 0.5) * 2
            y = (index // 16 + 0.5) * 2
            expected = 100 if math.hypot(x - 9, y - 3) <= 3 else normal_costs[index]
            assert value == expected, (index, value, expected)
        passed.append('four_field_message_and_exact_circle_with_two_meter_inflation')
        report(sensed_at(19, 3), stamp=stamp)
        expect('new_snapshot_replaces_previous_targets', {20:0, 25:100})
        report(frame='missing_frame')  # 空数组不需要 TF 就能清除。
        expect('empty_snapshot_clears_immediately_without_tf', {20:0, 25:20})
        report(sensed_at(9, 3), stamp=stamp)
        expect('external_target_reappears', {20:100})
        expect('timeout_clears_target_without_any_more_messages', {20:0, 25:20})
        report(sensed_at(9, 3), stamp=stamp)  # 时间已过期，重复发送不能续命。
        drain(0.35)
        assert grids[-1].data[20] == 0
        passed.append('stale_timestamp_cannot_reactivate_target')
        stamp = ship(2, 2)
        drain()
        report(sensed_at(9, 3), stamp=stamp)
        expect('fresh_target_after_timeout', {20:100})
        # 缺失 TF 的非空帧不允许沿用为新目标；TF 到达后能重试处理。
        stamp = node.get_clock().now().to_msg()
        report(sensed_at(19, 3), frame='delayed_sensor', stamp=stamp)
        drain(0.25)
        assert grids[-1].data[20] == 100 and grids[-1].data[25] == 20
        delayed = TransformStamped()
        delayed.header.frame_id = 'map'
        delayed.header.stamp = stamp
        delayed.child_frame_id = 'delayed_sensor'
        delayed.transform.translation.x = 2.0
        delayed.transform.translation.y = 2.0
        delayed.transform.rotation.z = math.sin(-math.pi / 4)
        delayed.transform.rotation.w = math.cos(-math.pi / 4)
        static_tf.sendTransform(delayed)
        expect('delayed_tf_retries_observation_at_its_timestamp', {20:0, 25:100})
        # 有效帧中的 NaN 不得使进程退出或画入地图。
        invalid = sensed_at(9, 3)
        invalid.range_m = float('nan')
        report(invalid, stamp=node.get_clock().now().to_msg(), frame='delayed_sensor')
        drain(0.3)
        assert manager.poll() is None and grids[-1].data[20] == 0
        report(frame='missing_frame')
        expect('invalid_target_and_explicit_clear', {20:0, 25:20})
        stamp = ship(2, 2)
        drain()
        report(sensed_at(19, 3), stamp=stamp)
        expect('target_before_grid_resize', {25:100})
        grid.info.width = 8
        grid.info.height = 4
        grid.info.origin.position.x = 16.0
        grid.data = [0] * 32
        grid.data[0] = 100
        chart_pub.publish(grid)
        until(lambda: grids and grids[-1].info.width == 8 and grids[-1].data[9] == 100)
        assert grids[-1].data[0] == 100
        report()
        expect('grid_resize_reprojects_and_clear_restores_water', {0:100, 9:20})
        assert manager.poll() is None
        print(json.dumps({'passed': passed, 'failed': []}, ensure_ascii=False, indent=2))
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
