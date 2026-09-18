"""真实C++节点：一次任务、途经点、晚订阅、新目标、服务重规划、无解撤销。"""
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def test_fixed_mission(tmp_path):
    pkg = 'channel_astar_global_planner'
    command = [get_package_prefix(pkg) + '/lib/' + pkg + '/channel_astar_node',
        '--ros-args', '--params-file', get_package_share_directory(pkg) + '/config/channel_astar.yaml',
        '-p', 'via_points:=[150.5,30.5]']
    with (tmp_path / 'node.log').open('w') as log:
        rclpy.init()
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        node = rclpy.create_node('fixed_mission_probe')
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        paths, active, late = [], [], []
        node.create_subscription(Path, '/astar_channel_waypoints', paths.append, qos)
        node.create_subscription(PoseStamped, '/goal_pose_from_astar', active.append, qos)
        maps = node.create_publisher(OccupancyGrid, '/channel/lane_costmap', qos)
        safety_pub = node.create_publisher(OccupancyGrid, '/local_costmap', qos)
        odoms = node.create_publisher(Odometry, '/odom', 10)
        goals = node.create_publisher(PoseStamped, '/goal_pose', 10)
        client = node.create_client(Trigger, '/channel_astar/replan')
        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.info.resolution = 1.0
        grid.info.width, grid.info.height = 320, 120
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (320 * 120)
        import copy
        safety = copy.deepcopy(grid)
        own = Odometry()
        own.header.frame_id = 'odom'
        own.pose.pose.position.x, own.pose.pose.position.y = 1.5, 30.5
        own.pose.pose.orientation.w = 1.0
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = 200.5, 50.5
        goal.pose.orientation.w = 1.0
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'map'
        transform.transform.translation.x, transform.transform.translation.y = 100.0, -20.0
        transform.transform.rotation.w = 1.0

        # [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
        def spin_for(seconds, condition=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert child.poll() is None, (tmp_path / 'node.log').read_text()
                grid.header.stamp = node.get_clock().now().to_msg()
                own.header.stamp = grid.header.stamp
                maps.publish(grid)
                safety_pub.publish(safety)
                odoms.publish(own)
                rclpy.spin_once(node, timeout_sec=0.02)
                if condition and condition(): return
            if condition: assert condition(), (tmp_path / 'node.log').read_text()

        # [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
        def nonempty(): return [p for p in paths if p.poses]
        # [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
        def current_x(): return active[-1].pose.position.x if active else None

        try:
            spin_for(8, lambda: goals.get_subscription_count() > 0 and client.service_is_ready())
            goals.publish(goal)
            spin_for(0.9)
            assert not nonempty(), 'Missing TF must not be treated as identity'
            transform.header.stamp = node.get_clock().now().to_msg()
            broadcaster.sendTransform(transform)
            spin_for(8, lambda: len(nonempty()) == 1 and active)
            coords = [(p.pose.position.x, p.pose.position.y) for p in nonempty()[0].poses]
            assert coords == [(1.5, 30.5), (101.5, 30.5), (150.5, 30.5),
                (250.5, 30.5), (300.5, 30.5)]
            assert current_x() == 101.5
            # 普通航点9m外不切换，7m内切换（边界8m）。
            own.pose.pose.position.x = 92.5
            spin_for(0.4)
            assert current_x() == 101.5
            own.pose.pose.position.x = 94.5
            spin_for(3, lambda: current_x() == 150.5)
            # 移动超过旧25m阈值、不断收到图甚至角色代价改变，路线仍只生成一次。
            own.pose.pose.position.x = 41.5
            grid.data = [1] * len(grid.data)
            spin_for(1.5)
            assert len(nonempty()) == 1
            node.create_subscription(Path, '/astar_channel_waypoints', late.append, qos)
            spin_for(3, lambda: bool(late))
            assert [(p.pose.position.x, p.pose.position.y) for p in late[-1].poses] == coords
            assert len(nonempty()) == 1
            # 途经点距离8m时不能使用普通8m阈值跳过，须进入1.5m内。
            own.pose.pose.position.x = 142.5
            spin_for(0.6)
            assert current_x() == 150.5
            own.pose.pose.position.x = 149.5
            spin_for(3, lambda: current_x() == 250.5)
            # 已完成途经点不会在显式重规划时重新要求本船返回。
            future = client.call_async(Trigger.Request())
            spin_for(3, lambda: future.done())
            assert future.result().success
            spin_for(8, lambda: len(nonempty()) == 2)
            assert all(abs(p.pose.position.x - 150.5) > 0.1 for p in nonempty()[-1].poses[1:])
            # 新目标只再生成一次，最后目标坐标精确保留。
            goal.pose.position.y = 80.5
            goals.publish(goal)
            spin_for(8, lambda: len(nonempty()) == 3)
            assert nonempty()[-1].poses[-1].pose.position.y == 60.5
            spin_for(0.7)
            assert len(nonempty()) == 3
            # 再次点击相同目标也仅再规划一次。
            goals.publish(goal)
            spin_for(8, lambda: len(nonempty()) == 4)
            spin_for(0.7)
            assert len(nonempty()) == 4
            # 无解目标撤销旧路线，不留下旧任务继续驱动。
            goal.pose.position.x = -1000.0
            goals.publish(goal)
            spin_for(2)
            assert not paths[-1].poses
            assert len(nonempty()) == 4
            # 搜索失败后改变地图也不能自动再次搜索。
            failures_before = (tmp_path / 'node.log').read_text().count('A* final goal is outside')
            assert failures_before == 1
            grid.data = [2] * len(grid.data)
            spin_for(0.7)
            assert len(nonempty()) == 4
            assert (tmp_path / 'node.log').read_text().count('A* final goal is outside') == failures_before
            # 平行双航道：主体仍在右侧，仅末段连接左侧目标。
            grid.data = [2 if y < 60 else 75 for y in range(120) for x in range(320)]
            goal.pose.position.x, goal.pose.position.y = 200.5, 90.5
            goals.publish(goal)
            spin_for(8, lambda: len(nonempty()) == 5)
            left_route = nonempty()[-1]
            assert left_route.poses[-1].pose.position.y == 70.5
            assert all(p.pose.position.y < 60 for p in left_route.poses[:-1])
            spin_for(0.7)
            assert len(nonempty()) == 5
            # 左航道连接被不可通行语义隔断时，不允许越过障碍或从航道外绕行。
            for x in range(320):
                grid.data[60*320+x] = 100
            spin_for(0.2)
            goals.publish(goal)
            spin_for(2)
            assert not paths[-1].poses and len(nonempty()) == 5
            # 航道外安全水域可作为终点，主体仍留在主航道。
            grid.data = [2 if y < 60 else 45 for y in range(120) for x in range(320)]
            goals.publish(goal)
            spin_for(8, lambda: len(nonempty()) == 6)
            assert all(p.pose.position.y < 60 for p in nonempty()[-1].poses[:-1])
            # 同一终点在安全图变为高代价危险区，重新点击必须拒绝。
            safety.data[70*320+300] = 88
            spin_for(0.2)
            goals.publish(goal)
            spin_for(2)
            assert not paths[-1].poses and len(nonempty()) == 6
            # 船已到航道外：新目标允许先返回主航道，不能把合法起点直接拒绝。
            safety.data[70*320+300] = 0
            own.pose.pose.position.x, own.pose.pose.position.y = 300.5, 70.5
            goal.pose.position.x, goal.pose.position.y = -98.5, 50.5
            goals.publish(goal)
            spin_for(8, lambda: len(nonempty()) == 7)
            assert all(p.pose.position.y < 60 for p in nonempty()[-1].poses[1:])
            # 误识别：整张主航道变为对向航道，取消旧任务并仅尝试一次。
            grid.data = [75] * len(grid.data)
            spin_for(2)
            assert not paths[-1].poses and len(nonempty()) == 7
            # 航道角色恢复，不点击目标也自动纠偏一次，重复图不继续规划。
            grid.data = [2] * len(grid.data)
            spin_for(8, lambda: len(nonempty()) == 8)
            spin_for(1)
            assert len(nonempty()) == 8
            # 0..20内部梯度不算角色变化。
            grid.data = [12] * len(grid.data)
            spin_for(1)
            assert len(nonempty()) == 8
            # 完成途经点后发生角色纠偏，不要求返回已完成点。
            for point in nonempty()[-1].poses[1:]:
                own.pose.pose.position.x = point.pose.position.x
                own.pose.pose.position.y = point.pose.position.y
                spin_for(0.15)
                if point.pose.position.x == 150.5 and point.pose.position.y == 30.5:
                    break
            grid.data[319] = 75  # 远离路线的小块角色变化，同样只触发一次。
            spin_for(8, lambda: len(nonempty()) == 9)
            assert all(abs(p.pose.position.x-150.5) > 0.1 for p in nonempty()[-1].poses[1:])
            # 快速翻转再恢复，只搜索稳定后的最新快照。
            grid.data[318] = 75
            spin_for(0.15)
            grid.data = [13] * len(grid.data)
            spin_for(8, lambda: len(nonempty()) == 10)
            spin_for(1)
            assert len(nonempty()) == 10
        finally:
            child.terminate()
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
