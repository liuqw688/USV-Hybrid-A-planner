#!/usr/bin/env python3
"""Real /odom and /cmd_wrench validation of the running open-water tracking launch.
No target vessels or avoidance scenarios. Uses the existing MPC and simulator.
"""
import argparse
import json
import math
import os
from pathlib import Path as FilePath
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float64MultiArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance_to_path(x, y, points):
    best = float('inf')
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = dx*dx+dy*dy
        if length < 1e-12:
            continue
        t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/length))
        best = min(best, math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=100.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('usv_closed_loop_verifier')
    latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'odom': None, 'path': None, 'profile': None, 'debug': {}, 'effort': (0., 0.), 'states': None}
    node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 20)
    node.create_subscription(Path, '/hybrid_a_star_trajectory', lambda m: state.update(path=m), latch)
    node.create_subscription(TwistStamped, '/hybrid_a_star/speed_limit', lambda m: state.update(profile=m), latch)
    node.create_subscription(String, '/usv_frontend/status', lambda m: state.update(debug=json.loads(m.data)), latch)
    node.create_subscription(Float64MultiArray, '/usv_frontend/state_samples', lambda m: state.update(states=m), latch)
    node.create_subscription(Wrench, '/cmd_wrench', lambda m: state.update(effort=(m.force.x, m.torque.z)), 20)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    report = {'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
              'scope': 'open_water_actual_mpc_force_simulation', 'cases': [], 'passed': False}
    try:
        deadline = time.monotonic()+15
        while time.monotonic() < deadline and (state['odom'] is None or not goal_pub.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=.05)
        if state['odom'] is None or not goal_pub.get_subscription_count():
            raise RuntimeError('tracking launch is not ready')
        for name, goal in [('straight_30m', (30., 0., 0.)), ('curve_to_60m_15m', (60., 15., .4))]:
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.header.stamp = node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = goal[:2]
            p.pose.orientation.z, p.pose.orientation.w = math.sin(goal[2]/2), math.cos(goal[2]/2)
            goal_pub.publish(p)
            initial = state['odom']
            case = {'name': name, 'goal': goal,
                    'initial_state': [initial.pose.pose.position.x, initial.pose.pose.position.y,
                                      yaw(initial.pose.pose.orientation), initial.twist.twist.linear.x,
                                      initial.twist.twist.linear.y, initial.twist.twist.angular.z],
                    'track': [], 'reference': [], 'arrived': False, 'matched_profile_samples': 0}
            report['cases'].append(case)
            begin = time.monotonic()
            last_record, last_print, held = -1., -10., None
            while time.monotonic()-begin < args.timeout:
                rclpy.spin_once(node, timeout_sec=.025)
                elapsed = time.monotonic()-begin
                odom, path, profile = state['odom'], state['path'], state['profile']
                debug = state['debug']
                matching = len(debug.get('goal', [])) == 6 and all(abs(a-b)<1e-7 for a, b in zip(debug['goal'][:3], goal))
                if matching and debug.get('status') in ('tracking', 'arrived'):
                    case['planning'] = dict(debug)
                if matching and debug.get('status') == 'blocked':
                    case['failure'] = 'frontend blocked'; break
                if path and path.poses and math.hypot(path.poses[-1].pose.position.x-goal[0],
                                                      path.poses[-1].pose.position.y-goal[1]) < .2:
                    if not case['reference']:
                        case['reference'] = [[a.pose.position.x, a.pose.position.y, yaw(a.pose.orientation)] for a in path.poses]
                        case['diagnostic_at_first_path'] = dict(debug)
                        if state['states'] is not None:
                            case['state_sample_layout'] = state['states'].layout.dim[-1].label
                            case['state_samples'] = list(state['states'].data)
                    if profile and profile.header == path.header:
                        case['matched_profile_samples'] += 1
                if odom is None or elapsed-last_record < .09:
                    continue
                last_record = elapsed
                x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
                heading = yaw(odom.pose.pose.orientation)
                u, v, r = odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z
                error = math.hypot(x-goal[0], y-goal[1])
                speed = math.hypot(u, v)
                heading_error = abs(angle(heading-goal[2]))
                cross_track = distance_to_path(x, y, case['reference']) if case['reference'] else None
                case['track'].append([elapsed, x, y, heading, u, v, r, *state['effort'], cross_track])
                if elapsed-last_print >= 5:
                    last_print = elapsed
                    print(f'{name}: t={elapsed:.1f}s, position=({x:.2f},{y:.2f}), distance={error:.2f}m, '
                          f'speed={speed:.2f}m/s, cross_track={cross_track}', flush=True)
                if case['reference'] and error <= 1.0 and speed <= .12 and heading_error <= math.pi/6:
                    held = elapsed if held is None else held
                    if elapsed-held >= 2.0:
                        case.update(arrived=True, arrival_seconds=elapsed, final_distance=error,
                                    final_speed=speed, final_heading_error=heading_error)
                        break
                else:
                    held = None
            samples = [a[9] for a in case['track'] if a[9] is not None]
            if samples:
                case['cross_track_rms'] = math.sqrt(sum(d*d for d in samples)/len(samples))
                case['cross_track_max'] = max(samples)
            if case['track']:
                last = case['track'][-1]
                case.setdefault('final_distance', math.hypot(last[1]-goal[0], last[2]-goal[1]))
                case.setdefault('final_speed', math.hypot(last[4], last[5]))
            case['positive_thrust_seen'] = any(a[7] > .1 for a in case['track'])
            case['passed'] = case['arrived'] and case['positive_thrust_seen'] and case['matched_profile_samples'] > 3 \
                and case.get('cross_track_max', float('inf')) < 2.0
            FilePath(args.output).write_text(json.dumps(report, indent=2)+'\n')
            print(f'{name} RESULT: arrived={case["arrived"]}, final_distance={case.get("final_distance")}, '
                  f'RMS={case.get("cross_track_rms")}, max={case.get("cross_track_max")}', flush=True)
            if not case['arrived']:
                break
        report['passed'] = len(report['cases']) == 2 and all(c['passed'] for c in report['cases'])
    finally:
        FilePath(args.output).write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    if not report['passed']:
        raise SystemExit('closed-loop arrival/tracking criteria not all met; full data retained')


if __name__ == '__main__':
    main()
