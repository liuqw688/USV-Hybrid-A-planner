#!/usr/bin/env python3
"""实际河图整链路检查；独占一键启动，建议 ROS_DOMAIN_ID=74。

验证固定全局任务、持续局部规划、晚订阅及无解任务撤销；不是实船安全认证。
"""
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


# [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
def main():
    rclpy.init()
    node = rclpy.create_node('river_fixed_route_probe')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    routes, local, active, commands, late = [], [], [], [], []
    state = {}
    node.create_subscription(Path, '/astar_channel_waypoints', routes.append, qos)
    node.create_subscription(Path, '/hybrid_a_star_trajectory', local.append, 10)
    node.create_subscription(PoseStamped, '/goal_pose_from_astar', active.append, qos)
    node.create_subscription(Odometry, '/odom', lambda m: state.update(own=m), 10)
    node.create_subscription(OccupancyGrid, '/channel/lane_costmap',
                             lambda m: state.update(lane=m), qos)
    node.create_subscription(OccupancyGrid, '/local_costmap',
                             lambda m: state.update(safety=m), qos)
    node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    child = None

    # [功能与联系] 诊断脚本限时派发回调并检查条件；等待期间不改变正常规划频率，超时用于判定测试失败。
    def spin(seconds, condition=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert child.poll() is None, 'launch exited; inspect /dev/shm/river_fixed_route.log'
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition and condition():
                return
        if condition:
            assert condition(), 'timed out waiting for pipeline'

    try:
        with open('/dev/shm/river_fixed_route.log', 'w') as log:
            child = subprocess.Popen(['ros2', 'launch', 'channel_navigation_manager',
                'river_navigation.launch.py', 'use_rviz:=false', 'use_virtual_boat:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            spin(35, lambda: 'lane' in state and 'safety' in state and 'own' in state and
                 pub.get_subscription_count() >= 2)
            start = state['own'].pose.pose.position
            sx, sy = start.x, start.y
            goal = PoseStamped()
            goal.header.frame_id = 'map'  # 与RViz Fixed Frame=map的目标一致，经真实静态TF转换。
            goal.pose.position.x, goal.pose.position.y = -324.0, -101.0
            outside_goal = os.environ.get('RIVER_TEST_OUTSIDE_GOAL') == '1'
            if os.environ.get('RIVER_TEST_LEFT_GOAL') == '1' or outside_goal:
                # 在真实海图中选择标准目标附近的左航道栅格，核验末段跨航道入口。
                grid = state['lane']
                q = grid.info.origin.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                # [功能与联系] 按真实栅格原点姿态和分辨率求格中心世界坐标；用于选取左/航道外测试终点。
                def location(i):
                    x = (i % grid.info.width + 0.5)*grid.info.resolution
                    y = (i // grid.info.width + 0.5)*grid.info.resolution
                    return (grid.info.origin.position.x+math.cos(yaw)*x-math.sin(yaw)*y,
                            grid.info.origin.position.y+math.sin(yaw)*x+math.cos(yaw)*y)
                # [功能与联系] 计算候选格到标准测试终点的平方距离；仅选择测试场景，不参与生产A*代价。
                def score(i):
                    x, y = location(i)
                    return (x+324)**2+(y+101)**2
                index = min((i for i, value in enumerate(grid.data)
                             if value == (45 if outside_goal else 75) and
                             (not outside_goal or 0 <= state['safety'].data[i] <= 30)), key=score)
                goal.pose.position.x, goal.pose.position.y = location(index)
            goal.pose.orientation.z = math.sin(math.radians(110) / 2)
            goal.pose.orientation.w = math.cos(math.radians(110) / 2)
            pub.publish(goal)
            spin(35, lambda: any(p.poses for p in routes) and any(p.poses for p in local))
            route = next(p for p in routes if p.poses)
            assert len(route.poses) >= 3
            if os.environ.get('RIVER_TEST_LEFT_GOAL') == '1' or outside_goal:
                grid = state['lane']
                for point in route.poses[:-1]:
                    dx = point.pose.position.x-grid.info.origin.position.x
                    dy = point.pose.position.y-grid.info.origin.position.y
                    x = int(math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/grid.info.resolution))
                    y = int(math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/grid.info.resolution))
                    assert 0 <= grid.data[y*grid.info.width+x] <= 20, 'sparse intermediate point in left lane'
                assert abs(route.poses[-1].pose.position.x-goal.pose.position.x) < 1e-6
                assert abs(route.poses[-1].pose.position.y-goal.pose.position.y) < 1e-6
            before = len(local)
            node.create_subscription(Path, '/astar_channel_waypoints', late.append, qos)
            spin(25)
            local_count = len(local)-before
            assert len([p for p in routes if p.poses]) == 1, 'global route was replanned'
            assert late and late[-1] == route, 'late subscriber did not receive fixed route'
            assert len(local) - before >= 8, 'Hybrid stopped continuous planning'
            p = state['own'].pose.pose.position
            moved = math.hypot(p.x-sx, p.y-sy)
            assert moved > 25, 'vessel did not advance beyond old replan threshold'
            # 新目标无解必须撤销全局任务与局部轨迹，而不是沿旧任务继续行驶。
            goal.pose.position.x = 1e6
            pub.publish(goal)
            spin(5, lambda: routes and not routes[-1].poses and local and not local[-1].poses)
            # 跨话题回调无全局顺序；先排空撤销前已排队的cmd，再核验稳定停车。
            spin(0.5)
            commands.clear()
            spin(1)
            assert commands and all(abs(m.linear.x) < 1e-9 and abs(m.angular.z) < 1e-9
                                    for m in commands), 'old mission still commands movement'
            report = dict(passed=True, global_publications=1, waypoint_count=len(route.poses),
                          left_goal=os.environ.get('RIVER_TEST_LEFT_GOAL') == '1',
                          outside_goal=outside_goal,
                          goal_xy=[route.poses[-1].pose.position.x, route.poses[-1].pose.position.y],
                          local_publications_during_25s=local_count,
                          moved_m=round(moved, 2), active_goal_publications=len(active),
                          late_subscription=True, invalid_goal_stops=True)
            print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
