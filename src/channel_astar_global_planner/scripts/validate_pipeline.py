#!/usr/bin/env python3
"""Validate the live Channel A* -> Hybrid A* contract after a goal is sent."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PipelineValidator(Node):
    # [功能与联系] 读取本节点参数并创建ROS输入输出/调度；后续回调与主循环关系见包内功能说明。
    def __init__(self) -> None:
        super().__init__("channel_astar_pipeline_validator")
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.lane = None
        self.final_goal = None
        self.astar_path = None
        self.active_goal = None
        self.hybrid_path = None
        self.create_subscription(OccupancyGrid, "/channel/lane_costmap", self._lane, latched)
        self.create_subscription(Path, "/astar_channel_waypoints", self._astar, latched)
        self.create_subscription(PoseStamped, "/goal_pose_from_astar", self._active, latched)
        self.create_subscription(Path, "/hybrid_a_star_trajectory", self._hybrid, latched)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, 10)

    # [功能与联系] 缓存最新航道图供诊断校验，未执行生产航道分类。
    def _lane(self, message: OccupancyGrid) -> None:
        self.lane = message

    # [功能与联系] 缓存请求终点供诊断对比；本历史校验器不做跨frame转换，map非单位TF场景可能误报。
    def _goal(self, message: PoseStamped) -> None:
        self.final_goal = message

    # [功能与联系] 缓存全局稀疏Path供诊断检查；生产航点切换由全局C++节点完成。
    def _astar(self, message: Path) -> None:
        self.astar_path = message

    # [功能与联系] 缓存当前Hybrid活动目标供接口存在性核验。
    def _active(self, message: PoseStamped) -> None:
        self.active_goal = message

    # [功能与联系] 缓存最近非空Hybrid路径供诊断；此脚本不能独立证明控制安全或最终到达。
    def _hybrid(self, message: Path) -> None:
        if message.poses:
            self.hybrid_path = message

    # [功能与联系] 把世界点变换到语义格并读取类别，用于历史右航道校验；不是碰撞验证。
    def lane_value(self, x: float, y: float) -> int | None:
        if self.lane is None:
            return None
        origin = self.lane.info.origin
        quaternion = origin.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        dx, dy = x - origin.position.x, y - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ix = math.floor(local_x / self.lane.info.resolution)
        iy = math.floor(local_y / self.lane.info.resolution)
        if not (0 <= ix < self.lane.info.width and 0 <= iy < self.lane.info.height):
            return None
        return int(self.lane.data[iy * self.lane.info.width + ix])

    # [功能与联系] 检查话题存在、全局点在右航道及末点相等；旧规则不支持新左/外终点与跨frame比较，可能误报，详见文档边界。
    def validate(self) -> list[str]:
        failures = []
        if self.lane is None:
            failures.append("missing /channel/lane_costmap")
        if self.astar_path is None or not self.astar_path.poses:
            failures.append("missing non-empty /astar_channel_waypoints")
        elif self.lane is not None:
            invalid = []
            for index, stamped in enumerate(self.astar_path.poses):
                value = self.lane_value(stamped.pose.position.x, stamped.pose.position.y)
                if value is None or not 0 <= value <= 20:
                    invalid.append((index, value))
            if invalid:
                failures.append(f"A* waypoints outside right lane: {invalid[:10]}")
        if self.active_goal is None:
            failures.append("missing /goal_pose_from_astar PoseStamped")
        if self.hybrid_path is None:
            failures.append("missing non-empty /hybrid_a_star_trajectory")
        if self.final_goal is not None and self.astar_path and self.astar_path.poses:
            last = self.astar_path.poses[-1].pose.position
            requested = self.final_goal.pose.position
            if math.hypot(last.x - requested.x, last.y - requested.y) > 1.0e-3:
                failures.append("A* final waypoint does not equal /goal_pose")
        return failures


# [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
def main() -> int:
    rclpy.init()
    node = PipelineValidator()
    deadline = time.monotonic() + 30.0
    print("Waiting up to 30 s; publish a valid RViz 2D Goal Pose in the right-hand lane.")
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.lane is not None and node.astar_path and node.astar_path.poses and \
                node.active_goal is not None and node.hybrid_path is not None:
            break
    failures = node.validate()
    if failures:
        print("PIPELINE_VALIDATION=FAIL")
        for failure in failures:
            print(f"- {failure}")
    else:
        print(f"PIPELINE_VALIDATION=PASS A*_waypoints={len(node.astar_path.poses)} "
              f"Hybrid_poses={len(node.hybrid_path.poses)}")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
