#!/usr/bin/env python3
"""Adapters for the open-water simulation; no change to planner message contracts."""
import copy
import math
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from plan_manage.msg import Bspline
from std_msgs.msg import Header


class Bridge(Node):
    def __init__(self):
        super().__init__('fast_planner_bridge')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, '/planner/odom', 10)
        self.goal_pub = self.create_publisher(Path, '/waypoint_generator/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/fast_planner/path', latched)
        self.active_goal_pub = self.create_publisher(PoseStamped, '/fast_planner/goal', latched)
        self.points_pub = self.create_publisher(Marker, '/fast_planner/control_points', latched)
        self.create_subscription(Odometry, '/odom', self.odom, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal, 10)
        self.create_subscription(Bspline, '/planning/bspline', self.spline, 10)
        self.create_subscription(Header, '/planning/replan_failed', self.replan_failed, 10)
        self.path = None
        self.expiry = None
        self.path_start = None
        self.last_failure_ns = -1
        self.create_timer(0.5, self.publish_path)

    def odom(self, msg):
        # The simulation defines map and odom as coincident fixed frames.
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().warning('Unexpected simulation odometry frame; ignoring')
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        u, v = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        out = copy.deepcopy(msg)
        out.header.frame_id = 'map'
        out.twist.twist.linear.x = math.cos(yaw)*u - math.sin(yaw)*v
        out.twist.twist.linear.y = math.sin(yaw)*u + math.cos(yaw)*v
        self.odom_pub.publish(out)

    def goal(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning('Goal must be in map frame')
            return
        self.path = None
        self.withdraw_path()
        path = Path()
        path.header = msg.header
        path.poses = [msg]
        self.active_goal_pub.publish(msg)
        self.goal_pub.publish(path)

    def replan_failed(self, msg):
        failure_ns = Time.from_msg(msg.stamp).nanoseconds
        self.last_failure_ns = max(self.last_failure_ns, failure_ns)
        if self.path is not None and self.path_start.nanoseconds <= failure_ns:
            self.path = None
            self.withdraw_path()
            self.get_logger().warning('Planning failed; old path withdrawn for braking')

    def spline(self, msg):
        start_time = Time.from_msg(msg.start_time)
        if start_time.nanoseconds <= self.last_failure_ns:
            self.get_logger().warning('Ignoring an older trajectory after a planning failure')
            return
        self.path = None
        try:
            p = int(msg.order)
            knots = np.asarray(msg.knots, dtype=float)
            points = np.asarray([(x.x, x.y, x.z) for x in msg.pos_pts], dtype=float)
            if p < 1 or len(points) <= p or len(knots) != len(points)+p+1:
                raise ValueError('invalid spline dimensions')
            if not np.isfinite(points).all() or not np.isfinite(knots).all() or np.any(np.diff(knots) < 0):
                raise ValueError('nonfinite or decreasing spline data')
            start, end = knots[p], knots[len(points)]
            if end <= start:
                raise ValueError('empty spline time interval')
            samples = BSpline(knots, points, p)(np.linspace(start, end, max(2, math.ceil((end-start)/0.1)+1)))
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = msg.start_time
            for x, y, yaw in samples:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z = math.sin(yaw/2)
                pose.pose.orientation.w = math.cos(yaw/2)
                path.poses.append(pose)
            marker = Marker()
            marker.header = path.header
            marker.ns = 'control_points'
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r, marker.color.g, marker.color.a = 1.0, 0.25, 1.0
            marker.points = [Point(x=float(pt[0]), y=float(pt[1]), z=0.0) for pt in points]
            self.points_pub.publish(marker)
            self.path = path
            self.path_start = start_time
            self.expiry = Time.from_msg(msg.start_time) + Duration(seconds=float(end-start))
            self.get_logger().info(f'trajectory {msg.traj_id}: {len(points)} control points, {end-start:.2f}s')
            self.publish_path()
        except (ValueError, IndexError) as exc:
            self.get_logger().error(f'Invalid B-spline: {exc}')
            self.withdraw_path()

    def withdraw_path(self):
        empty = Path()
        empty.header.frame_id = 'map'
        empty.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty)
        marker = Marker()
        marker.header = empty.header
        marker.ns = 'control_points'
        marker.id = 0
        marker.action = Marker.DELETE
        self.points_pub.publish(marker)

    def publish_path(self):
        if self.path is not None:
            if self.get_clock().now() <= self.expiry:
                self.path_pub.publish(self.path)
            else:
                self.path = None
                self.withdraw_path()
                self.get_logger().warning('Trajectory expired; path withdrawn')


def main():
    rclpy.init()
    node = Bridge()
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
