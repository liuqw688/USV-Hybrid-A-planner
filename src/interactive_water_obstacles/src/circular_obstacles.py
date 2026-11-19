#!/usr/bin/env python3
"""Overlay persistent circular occupied cells on a separate base OccupancyGrid."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


class CircularObstacles(Node):
    def __init__(self):
        super().__init__('circular_water_obstacles')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('base_map_topic', '/free_water/base_map')
        self.declare_parameter('output_map_topic', '/map')
        self.declare_parameter('pose_topic', '/initialpose')
        self.radius = float(self.get_parameter('radius').value)
        base = self.get_parameter('base_map_topic').value
        output = self.get_parameter('output_map_topic').value
        if not math.isfinite(self.radius) or self.radius <= 0.0 or base == output:
            raise ValueError('radius must be finite and positive; map topics must differ')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base = None
        self.circles = []
        self.publisher = self.create_publisher(OccupancyGrid, output, qos)
        self.create_subscription(OccupancyGrid, base, self.on_map, qos)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter('pose_topic').value, self.on_pose, 10)
        self.get_logger().info(f'Circular obstacles: radius {self.radius:.2f} m, {base} -> {output}')

    def on_map(self, msg):
        info = msg.info
        if not msg.header.frame_id or info.width == 0 or info.height == 0 or len(msg.data) != info.width * info.height or not math.isfinite(info.resolution) or info.resolution <= 0:
            self.get_logger().warning('Invalid base map; ignoring')
            return
        self.base = msg
        self.publish_map()

    def on_pose(self, msg):
        if self.base is None:
            self.get_logger().warning('No base map yet; obstacle not added')
            return
        if msg.header.frame_id != self.base.header.frame_id:
            self.get_logger().warning('Pose frame differs from map; obstacle not added')
            return
        p = msg.pose.pose.position
        if not math.isfinite(p.x) or not math.isfinite(p.y):
            self.get_logger().warning('Non-finite pose; obstacle not added')
            return
        self.circles.append((p.x, p.y, self.radius))
        self.publish_map()
        self.get_logger().info(f'Added obstacle {len(self.circles)} at ({p.x:.2f}, {p.y:.2f}), r={self.radius:.2f} m')

    def publish_map(self):
        if self.base is None:
            return
        msg = copy.deepcopy(self.base)
        info = msg.info
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        c, s, res = math.cos(yaw), math.sin(yaw), info.resolution
        data = list(msg.data)
        for x, y, r in self.circles:
            dx, dy = x-info.origin.position.x, y-info.origin.position.y
            cx, cy = (c*dx+s*dy)/res, (-s*dx+c*dy)/res
            rad = r/res
            xmin, xmax = max(0, math.floor(cx-rad)), min(info.width-1, math.floor(cx+rad))
            ymin, ymax = max(0, math.floor(cy-rad)), min(info.height-1, math.floor(cy+rad))
            for row in range(ymin, ymax+1):
                for col in range(xmin, xmax+1):
                    # Mark every grid square intersecting the circle, conservatively.
                    px = min(max(cx, col), col+1)
                    py = min(max(cy, row), row+1)
                    if (px-cx)**2+(py-cy)**2 <= rad**2:
                        data[row*info.width+col] = 100
        msg.data = data
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CircularObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
