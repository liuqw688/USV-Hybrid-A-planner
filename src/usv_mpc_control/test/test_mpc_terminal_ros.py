#!/usr/bin/env python3
"""真实MPC节点+独立六状态RK4参考船模，验证终点停车、航点交接和直接目标。

仅在隔离ROS域174发布，不影响用户仿真；日志和指标写入/tmp。
"""
import argparse
import json
import math
import os
import subprocess
import time

os.environ.setdefault('ROS_DOMAIN_ID', '174')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/usv_mpc_terminal_logs'
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Wrench
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy


def derivative(s, c):
    x, y, yaw, u, v, r = s
    thrust, torque = c
    return [u*math.cos(yaw)-v*math.sin(yaw), u*math.sin(yaw)+v*math.cos(yaw), r,
            -0.290681*u+0.295742*thrust-2.126739*r*r,
            -0.425383*v-1.149012*r, -1.342561*r+0.781688*torque]


def step(s, c, dt):
    def shifted(k, f):
        return [a+dt*f*b for a, b in zip(s, k)]
    k1 = derivative(s, c)
    k2 = derivative(shifted(k1, .5), c)
    k3 = derivative(shifted(k2, .5), c)
    k4 = derivative(shifted(k3, 1), c)
    result = [a+dt*(b+2*d+2*e+f)/6 for a, b, d, e, f in zip(s, k1, k2, k3, k4)]
    result[3] = max(0., min(4., result[3]))
    result[5] = max(-.45, min(.45, result[5]))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--direct-goal', action='store_true', help='最终目标直接输入，不发布全局航点')
    parser.add_argument('--duration', type=float, default=28.)
    parser.add_argument('--curve', action='store_true')
    parser.add_argument('--intermediate', action='store_true')
    parser.add_argument('--map-frame', action='store_true', help='测试map地图经TF接入odom控制')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mpc_terminal_fixture')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/mpc_test/map', qos)
    path_pub = node.create_publisher(Path, '/mpc_test/path', qos)
    route_pub = node.create_publisher(Path, '/mpc_test/route', qos)
    goal_pub = node.create_publisher(PoseStamped, '/mpc_test/goal', qos)
    odom_pub = node.create_publisher(Odometry, '/mpc_test/odom', 20)
    command = [0., 0.]
    commands, predictions = [], []
    state = [0., 1., .15, 2., 0., 0.]
    def on_command(m):
        command[:] = [m.force.x, m.torque.z]
        commands.append((state[0], m.torque.z))
    def on_prediction(m):
        if m.action == Marker.ADD and m.points and state[0] > 12.:
            predictions.append(m.points[-1].y)
    node.create_subscription(Wrench, '/mpc_test/wrench', on_command, 20)
    node.create_subscription(Marker, '/mpc/predicted_trajectory', on_prediction, 20)
    prefix = get_package_prefix('usv_mpc_control')
    binary = prefix+'/lib/usv_mpc_control/mpc_controller'
    label = 'current'
    if args.direct_goal: label += '_direct_goal'
    if args.curve: label += '_curve'
    if args.intermediate: label += '_intermediate'
    if args.map_frame: label += '_map_frame'
    log = open('/tmp/usv_mpc_terminal_'+label+'.log', 'w')
    proc = subprocess.Popen([binary, '--ros-args', '--params-file', prefix+'/share/usv_mpc_control/config/mpc_params.yaml',
        '-p', 'channel_enabled:=false', '-p', 'require_speed_profile:=false', '-p', 'path_topic:=/mpc_test/path',
        '-p', 'costmap_topic:=/mpc_test/map', '-p', 'odom_topic:=/mpc_test/odom',
        '-p', 'global_route_topic:='+('""' if args.direct_goal else '/mpc_test/route'), '-p', 'active_goal_topic:=/mpc_test/goal',
        '-p', 'effort_topic:=/mpc_test/wrench',
        '-p', 'weight.torque_slew:=0.6'], stdout=log, stderr=log)
    try:
        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        if args.map_frame:
            from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
            from geometry_msgs.msg import TransformStamped
            broadcaster = StaticTransformBroadcaster(node)
            transform = TransformStamped()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'map'
            transform.transform.translation.x = 5.
            transform.transform.rotation.z = math.sin(.15)
            transform.transform.rotation.w = math.cos(.15)
            broadcaster.sendTransform(transform)
            grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 90
        grid.info.resolution = 1.
        grid.info.origin.position.x = grid.info.origin.position.y = -40.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*8100
        map_pub.publish(grid)
        path = Path()
        path.header.frame_id = 'odom'
        extended = Path()
        extended.header = path.header
        for i in range(361 if args.intermediate else 181):
            p = PoseStamped()
            p.header.frame_id = 'odom'
            arc=i*.1
            if args.curve:
                angle=arc/20.
                p.pose.position.x=20.*math.sin(angle)
                p.pose.position.y=20.*(1.-math.cos(angle))
                p.pose.orientation.z=math.sin(angle/2.)
                p.pose.orientation.w=math.cos(angle/2.)
            else:
                p.pose.position.x=arc
                p.pose.orientation.w=1.
            extended.poses.append(p)
            if i <= 180: path.poses.append(p)
        route=Path()
        route.header=path.header
        route.poses=list(extended.poses)
        if not args.direct_goal: route_pub.publish(route)
        goal_pub.publish(path.poses[-1])
        path_pub.publish(path)
        begin = last = time.monotonic()
        republish = begin
        samples = []
        handoff=False
        while time.monotonic()-begin < args.duration:
            assert proc.poll() is None, "MPC exited; inspect " + log.name
            current = time.monotonic()
            elapsed = min(.06, current-last)
            last = current
            state = step(state, command, elapsed)
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.header.stamp = stamp
            odom.pose.pose.position.x, odom.pose.pose.position.y = state[:2]
            odom.pose.pose.orientation.z = math.sin(state[2]/2)
            odom.pose.pose.orientation.w = math.cos(state[2]/2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = state[3:5]
            odom.twist.twist.angular.z = state[5]
            odom_pub.publish(odom)
            if args.intermediate and not handoff:
                point=path.poses[-1].pose.position
                if math.hypot(state[0]-point.x,state[1]-point.y)<8.:
                    path=extended
                    path_pub.publish(path)
                    goal_pub.publish(path.poses[-1])
                    handoff=True
            if current-republish > .5:
                path.header.stamp = stamp
                path_pub.publish(path)
                republish = current
            samples.append([current-begin]+state)
            rclpy.spin_once(node, timeout_sec=.02)
        near = [n for x, n in commands if x > 12.]
        signs = [1 if n > .05 else -1 for n in near if abs(n) > .05]
        flips = sum(a != b for a, b in zip(signs, signs[1:]))
        end=path.poses[-1].pose.position
        cross_error=abs(math.hypot(state[0],state[1]-20.)-20.) if args.curve else abs(state[1])
        metrics = {'case': label, 'final_state': state, 'goal_distance': math.hypot(state[0]-end.x,state[1]-end.y),
                   'cross_track_error': cross_error,
                   'waypoint_handoff': handoff,
                   'terminal_torque_sign_changes': flips,
                   'max_torque_step': max([abs(b-a) for a,b in zip(near,near[1:])]+[0.]),
                   'max_prediction_lateral_step': max([abs(b-a) for a,b in zip(predictions,predictions[1:])]+[0.]),
                   'samples': len(samples)}
        with open('/tmp/usv_mpc_terminal_'+label+'.json', 'w') as output:
            json.dump({'metrics': metrics, 'trajectory': samples}, output)
        print(json.dumps(metrics))
        if args.intermediate:
            assert handoff, 'Intermediate waypoint handoff was not exercised'
            assert state[3] > 1., 'Intermediate waypoint incorrectly stops the vessel'
            assert cross_error < 1., 'Poor bend tracking'
        else:
            assert metrics['goal_distance'] < .5, 'Final goal not tracked'
            assert state[3] < .15, 'Vessel did not smoothly stop'
            assert metrics['max_prediction_lateral_step'] < .25, 'Terminal prediction jumps too much'
            assert flips <= 5, 'Too many terminal turn reversals'
        assert metrics['max_torque_step'] <= .201, 'Torque continuity violated'
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
