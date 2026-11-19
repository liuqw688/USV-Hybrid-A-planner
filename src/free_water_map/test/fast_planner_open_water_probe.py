#!/usr/bin/env python3
"""Publish one simulated RViz goal and record observable planning/tracking metrics."""
import json
import math
import sys
import time
import numpy as np
from scipy.interpolate import BSpline as Spline
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from plan_manage.msg import Bspline


class Probe(Node):
    def __init__(self, x, y, yaw):
        super().__init__('fast_planner_open_water_probe')
        self.target = (x, y, yaw)
        self.odom = []
        self.splines = []
        self.path = None
        self.errors = []
        self.time_errors = []
        self.yaw_errors = []
        self.active_spline = None
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Path, '/fast_planner/path', self.on_path, 10)
        self.create_subscription(Bspline, '/planning/bspline', self.on_spline, 10)

    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, yaw = self.target
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pub.publish(msg)

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((time.monotonic(), p.x, p.y, msg.twist.twist.linear.x))
        if self.active_spline:
            spline, start_time, duration = self.active_spline
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - start_time
            if 0 <= t <= duration:
                ref = spline(spline.t[spline.k] + t)
                self.time_errors.append(math.hypot(p.x-ref[0], p.y-ref[1]))
                q = msg.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.yaw_errors.append(abs(math.atan2(math.sin(yaw-ref[2]), math.cos(yaw-ref[2]))))
        if self.path:
            self.errors.append(min(math.hypot(p.x-x, p.y-y) for x, y in self.path))

    def on_spline(self, msg):
        knots = np.asarray(msg.knots, dtype=float)
        points = np.asarray([(p.x, p.y, p.z) for p in msg.pos_pts], dtype=float)
        if len(points) > msg.order and len(knots) == len(points)+msg.order+1:
            self.active_spline = (Spline(knots, points, msg.order), msg.start_time.sec + msg.start_time.nanosec * 1e-9, float(knots[len(points)]-knots[msg.order]))
        self.splines.append({'id': msg.traj_id, 'count': len(msg.pos_pts),
                             'order': msg.order, 'knots': list(msg.knots), 'start_time': msg.start_time.sec + msg.start_time.nanosec * 1e-9,
                             'control_points': [[p.x, p.y, p.z] for p in msg.pos_pts]})


def main():
    rclpy.init()
    node = Probe(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    start = time.monotonic()
    sent = False
    while time.monotonic()-start < float(sys.argv[4]):
        rclpy.spin_once(node, timeout_sec=0.1)
        if not sent and time.monotonic()-start >= 8:
            node.send_goal()
            sent = True
    result = {'target': node.target, 'duration_seconds': time.monotonic()-start,
              'odom_samples': len(node.odom), 'splines': node.splines,
              'last_odom': node.odom[-1] if node.odom else None,
              'time_error_count': len(node.time_errors),
              'time_error_mean_m': sum(node.time_errors)/len(node.time_errors) if node.time_errors else None,
              'time_error_max_m': max(node.time_errors) if node.time_errors else None,
              'yaw_error_max_rad': max(node.yaw_errors) if node.yaw_errors else None,
              'max_surge_mps': max(abs(o[3]) for o in node.odom) if node.odom else None,
              'tracking_error_count': len(node.errors),
              'tracking_error_mean_m': sum(node.errors)/len(node.errors) if node.errors else None,
              'tracking_error_max_m': max(node.errors) if node.errors else None,
              'goal_distance_m': math.hypot(node.odom[-1][1]-node.target[0], node.odom[-1][2]-node.target[1]) if node.odom else None}
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
