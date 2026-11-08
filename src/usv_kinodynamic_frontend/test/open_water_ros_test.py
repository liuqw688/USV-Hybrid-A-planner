#!/usr/bin/env python3
"""Test a running open_water_rviz launch using only open-water pose goals.
No obstacle/other-vessel, controller or actuator tests are included.
"""
import argparse
import json
import math
import pathlib
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float64MultiArray


def derivative(s, command):
    _, _, psi, u, v, r = s
    vc, wc = command
    return [u*math.cos(psi)-v*math.sin(psi), u*math.sin(psi)+v*math.cos(psi), r,
            -0.433264281*u+0.400421432*vc+0.032263988,
            -0.313324916*v-0.006741061*r-0.407958500*wc-0.000993619,
            0.166118168*v-0.544509453*r+0.837536662*wc+0.000626529]


def integrate(s, c, dt):
    k1 = derivative(s, c)
    k2 = derivative([v+dt*k/2 for v, k in zip(s, k1)], c)
    k3 = derivative([v+dt*k/2 for v, k in zip(s, k2)], c)
    k4 = derivative([v+dt*k for v, k in zip(s, k3)], c)
    return [v+dt*(a+2*b+2*d+e)/6 for v, a, b, d, e in zip(s, k1, k2, k3, k4)]


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('usv_open_water_interface_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses = []
    received = {'states': None, 'path': None}
    node.create_subscription(String, '/usv_frontend/status', lambda m: statuses.append(json.loads(m.data)), qos)
    node.create_subscription(Float64MultiArray, '/usv_frontend/state_samples',
                             lambda m: received.update(states=m), qos)
    node.create_subscription(Path, '/usv_frontend/path', lambda m: received.update(path=m), qos)
    goal_pub = node.create_publisher(PoseStamped, '/usv_frontend/goal', 10)
    start_pub = node.create_publisher(PoseWithCovarianceStamped, '/usv_frontend/initialpose', 10)
    report = {'scope': 'empty_open_water_frontend_only', 'cases': []}
    try:
        deadline = time.monotonic()+10
        while not goal_pub.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not goal_pub.get_subscription_count():
            raise RuntimeError('start open_water_rviz.launch.py before running this test')
        deadline = time.monotonic()+10
        while (node.count_subscribers('/usv_frontend/path') < 2 or
               node.count_subscribers('/usv_frontend/markers') < 1) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Start changes use exactly the RViz SetInitialPose message contract.
        scenarios = [
            ('straight_2m', (0., 0., 0.), (2., 0., 0.)),
            ('left_curve', (0., 0., 0.), (2., 1., 0.5)),
            ('right_curve', (0., 0., 0.), (2., -1., -0.5)),
            ('beyond_old_5m_horizon', (0., 0., 0.), (6., 0., 0.)),
            ('translated_rotated_start', (1., -1., 0.4), (3., 0.2, 0.6)),
            ('rviz_display_curve', (0., 0., 0.), (6., 2., 0.4)),
        ]
        for name, start, goal in scenarios:
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.header.stamp = node.get_clock().now().to_msg()
            initial.pose.pose.position.x, initial.pose.pose.position.y = start[:2]
            initial.pose.pose.orientation.z = math.sin(start[2]/2)
            initial.pose.pose.orientation.w = math.cos(start[2]/2)
            start_pub.publish(initial)
            target = PoseStamped()
            target.header = initial.header
            target.pose.position.x, target.pose.position.y = goal[:2]
            target.pose.orientation.z = math.sin(goal[2]/2)
            target.pose.orientation.w = math.cos(goal[2]/2)
            goal_pub.publish(target)
            cursor = len(statuses)
            deadline = time.monotonic()+12
            result = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                for status in statuses[cursor:]:
                    if status['status'] != 'planning' and all(abs(a-b)<1e-8 for a, b in zip(status['goal'][:3], goal)) \
                            and all(abs(a-b)<1e-8 for a, b in zip(status['start'][:3], start)):
                        result = status
                if result:
                    if result['status'] != 'reach_end':
                        break
                    states = received['states']
                    path = received['path']
                    if states and path and len(path.poses) == result['sample_count'] \
                            and len(states.data) == result['sample_count']*9 \
                            and max(abs(a-b) for a, b in zip(states.data[-8:-2], result['endpoint'])) < 1e-8:
                        break
            if result is None:
                raise RuntimeError(f'{name}: no planning result received')
            case = {'name': name, **result}
            report['cases'].append(case)
            if result['status'] != 'reach_end':
                case['passed'] = False
                print(f'{name}: {result["status"]}', flush=True)
                continue
            states = received['states']
            rows = [list(states.data[i:i+9]) for i in range(0, len(states.data), 9)]
            assert rows and len(rows) == result['sample_count']
            assert max(abs(a-b) for a, b in zip(rows[0][1:7], list(start)+[0., 0., 0.])) < 1e-8
            endpoint = rows[-1][1:7]
            position_error = math.hypot(endpoint[0]-goal[0], endpoint[1]-goal[1])
            heading_error = abs(angle(endpoint[2]-goal[2]))
            assert position_error <= 0.35 and heading_error <= 0.25
            assert abs(endpoint[3]) <= 0.15 and abs(endpoint[4]) <= 0.10 and abs(endpoint[5]) <= 0.12
            model_error = 0
            for first, second in zip(rows, rows[1:]):
                assert second[0] > first[0]
                assert all(math.isfinite(v) for v in first)
                expected = integrate(first[1:7], first[7:9], second[0]-first[0])
                model_error = max(model_error, max(abs(a-b) for a, b in zip(expected, second[1:7])))
            assert model_error < 1e-7
            case.update(passed=True, position_error=position_error, heading_error=heading_error,
                        max_rk4_step_error=model_error, samples=rows)
            print(f'{name}: reach_end, {len(rows)} samples, search={result["search_seconds"]:.3f}s, '
                  f'position error={position_error:.3f}m', flush=True)
        report['rviz_path_subscriber_present'] = node.count_subscribers('/usv_frontend/path') >= 2
        report['rviz_marker_subscriber_present'] = node.count_subscribers('/usv_frontend/markers') >= 1
        report['passed'] = all(c.get('passed', False) for c in report['cases']) \
            and report['rviz_path_subscriber_present'] and report['rviz_marker_subscriber_present']
    finally:
        pathlib.Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    if not report.get('passed'):
        raise SystemExit('some open-water checks failed; all results retained in report')


if __name__ == '__main__':
    main()
