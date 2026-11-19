#!/usr/bin/env python3
"""从默认海图的顺时针航道生成一周航点，发布一次 nav_msgs/Path。"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def polygon_centroid(points):
    """计算多边形面积质心，退化几何直接报错。"""
    points = [(float(p[0]), float(p[1])) for p in points]
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('航道多边形无效')
    cross = [points[i][0] * points[(i + 1) % len(points)][1] -
             points[(i + 1) % len(points)][0] * points[i][1] for i in range(len(points))]
    area2 = sum(cross)
    if abs(area2) < 1e-8:
        raise ValueError('航道多边形面积为零')
    center = tuple(sum((points[i][axis] + points[(i + 1) % len(points)][axis]) * cross[i]
                       for i in range(len(points))) / (3 * area2) for axis in range(2))
    inside = False
    for i, (x1, y1) in enumerate(points):
        x2, y2 = points[(i + 1) % len(points)]
        if (y1 > center[1]) != (y2 > center[1]) and center[0] < (
                (x2 - x1) * (center[1] - y1) / (y2 - y1) + x1):
            inside = not inside
    if not inside:
        raise ValueError('航道多边形质心不在航道内，不能用作航点')
    return center


def clockwise_route(chart, position, entry_lookahead):
    """按海图航段编号连接 CW 中心；从最近航段沿航道前进后选择入口。"""
    lanes = sorted((lane for lane in chart['lanes'] if lane.get('direction', '').upper() == 'CW'),
                   key=lambda lane: lane['lane_id'])
    if len(lanes) < 3 or len({lane['lane_id'] for lane in lanes}) != len(lanes):
        raise ValueError('顺时针航道段不足或编号重复')
    points = [polygon_centroid(lane['points']) for lane in lanes]
    signed_area = 0.0
    for i, lane in enumerate(lanes):
        x, y = points[i]
        nx, ny = points[(i + 1) % len(points)]
        dx, dy = nx - x, ny - y
        length = math.hypot(dx, dy)
        orient = math.radians(float(lane['orient_deg']))  # S-57：北零、顺时针为正。
        if not math.isfinite(orient) or not 0.1 < length <= 50.0 or (
                dx * math.sin(orient) + dy * math.cos(orient)) / length < 0.5:
            raise ValueError('航道编号不能构成连续顺时针闭环，不能自动生成航点')
        signed_area += x * ny - nx * y
    if signed_area >= 0:
        raise ValueError('航道顺序不是顺时针闭环')
    start = min(range(len(points)), key=lambda i: math.dist(points[i], position))
    distance = 0.0
    for _ in range(len(points) - 1):
        if distance >= entry_lookahead:
            break
        next_index = (start + 1) % len(points)
        distance += math.dist(points[start], points[next_index])
        start = next_index
    points = points[start:] + points[:start]
    return points + [points[0]]


class RoutePublisher(Node):
    """等待地图、里程计和规划器连接，然后只发送一次任务。"""

    def __init__(self, topic, entry_lookahead):
        super().__init__('publish_clockwise_waypoints')
        self.chart = None
        self.odom = None
        self.entry_lookahead = entry_lookahead
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/chart_lane_data', self.on_chart, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.publisher = self.create_publisher(Path, topic, 10)

    def on_chart(self, msg):
        try:
            self.chart = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'海图 JSON 解析失败: {exc}')

    def on_odom(self, msg):
        self.odom = msg

    def publish_route(self):
        frame = self.chart.get('frame_id', 'map')
        if self.odom.header.frame_id != frame:
            raise ValueError(f'里程计坐标系 {self.odom.header.frame_id!r} 与海图 {frame!r} 不一致')
        own = self.odom.pose.pose.position
        if not math.isfinite(own.x) or not math.isfinite(own.y):
            raise ValueError('里程计位置无效')
        points = clockwise_route(self.chart, (own.x, own.y), self.entry_lookahead)
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = x, y
            a, b = (points[i], points[i + 1]) if i + 1 < len(points) else (points[i - 1], points[i])
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(p)
        self.publisher.publish(path)
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.get_logger().info(f'已发布 {len(points)} 个航点，闭环长 {length:.1f} m；'
                               f'首点距船 {math.dist(points[0], (own.x, own.y)):.1f} m。')


def main():
    """解析话题与等待时限；超时或海图不适用时以非零状态退出。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/path', help='航点任务话题，默认 /path')
    parser.add_argument('--timeout', type=float, default=30.0, help='等待地图、里程计和订阅者的秒数')
    parser.add_argument('--entry-lookahead', type=float, default=30.0,
                        help='最近航点沿顺时针方向前移的入口距离，默认 30 m')
    parser.add_argument('--planner-node', default='usv_planner_node', help='等待连接的规划节点名称')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout 必须为正数')
    if not math.isfinite(args.entry_lookahead) or args.entry_lookahead < 0:
        parser.error('--entry-lookahead 必须为非负数')
    rclpy.init()
    node = RoutePublisher(args.topic, args.entry_lookahead)
    result = 1
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            planner_connected = any(info.node_name == args.planner_node
                                    for info in node.get_subscriptions_info_by_topic(args.topic))
            if node.chart is not None and node.odom is not None and planner_connected:
                node.publish_route()
                # 允许可靠传输完成；不重复发布以免重置航点进度。
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                result = 0
                break
        else:
            node.get_logger().error('等待超时：请检查 /chart_lane_data、/odom 和 Path 订阅者。')
    except (KeyError, TypeError, ValueError) as exc:
        node.get_logger().error(f'无法生成航点: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
