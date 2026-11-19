#!/usr/bin/env python3
"""发布往复式 B 样条测试航线及对应的 RViz2 路径."""

import bisect
import math
import os

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt64
from usv_interfaces.msg import UsvBspline, UsvBsplineControlPoint


def quaternion_yaw(q):
    """读取 RFU 本体系四元数的 ENU 船首航向."""
    yaw_of_rfu_x = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return math.atan2(math.sin(yaw_of_rfu_x + math.pi / 2.0),
                      math.cos(yaw_of_rfu_x + math.pi / 2.0))


class BsplineReferencePublisher(Node):
    """在 /odom 坐标系下生成一条平滑的测试航线并发布参考."""

    ODOM_TOPIC = "/odom"
    BSPLINE_TOPIC = "/planning/bspline"
    REFERENCE_PATH_TOPIC = "/planning/bspline_path"
    FRAME_ID = "odom"
    SPEED_MPS = 4.0
    PATH_LENGTH_M = 300.0
    LANE_COUNT = 5
    LANE_SPACING_M = 60.0
    TURN_BULGE_M = 30.0
    CURVE_AMPLITUDE_M = 90.0
    CURVE_HALF_WAVE_M = 120.0
    CONTROL_INTERVAL_S = 2.0
    PUBLISH_PERIOD_S = 1.0
    # 为 ROS 发现和订阅建立留出时间，避免首次接收时参考已越过船位接受范围。
    START_DELAY_S = 3.0

    def __init__(self):
        """读取环境中的测试路线配置，并创建里程计订阅、发布器及定时器."""
        super().__init__("bspline_reference_publisher")
        self.frame_id = self.FRAME_ID
        self.speed = float(os.environ.get("USV_NMPC_ROUTE_SPEED_MPS", self.SPEED_MPS))
        self.lane_length = float(os.environ.get("USV_NMPC_ROUTE_LENGTH_M", self.PATH_LENGTH_M))
        self.lane_count = int(os.environ.get("USV_NMPC_ROUTE_LANES", self.LANE_COUNT))
        self.lane_spacing = float(os.environ.get("USV_NMPC_ROUTE_SPACING_M", self.LANE_SPACING_M))
        self.turn_bulge = float(os.environ.get("USV_NMPC_ROUTE_BULGE_M", self.TURN_BULGE_M))
        self.curve_amplitude = float(os.environ.get(
            "USV_NMPC_ROUTE_CURVE_AMPLITUDE_M", self.CURVE_AMPLITUDE_M))
        self.curve_half_wave = float(os.environ.get(
            "USV_NMPC_ROUTE_CURVE_HALF_WAVE_M", self.CURVE_HALF_WAVE_M))
        self.start_offset = float(os.environ.get(
            "USV_NMPC_ROUTE_START_OFFSET_M", "0"))
        self.route_mode = os.environ.get("USV_NMPC_ROUTE_MODE", "mowing").strip().lower()
        self.interval = self.CONTROL_INTERVAL_S
        self.start_delay = self.START_DELAY_S
        if not (0.0 < self.speed <= 5.0):
            raise ValueError("speed_mps 必须在 (0, 5.0] 内")
        if self.route_mode not in {
                "straight", "reverse_straight", "arc_left", "arc_right",
                "s_curve", "mowing"}:
            raise ValueError(
                "route_mode 必须是 straight、reverse_straight、arc_left、"
                "arc_right、s_curve 或 mowing")
        if (self.lane_length <= 0.0 or self.lane_count < 1 or
                (self.route_mode == "mowing" and self.lane_count < 2) or
                self.lane_spacing <= 0.0 or self.turn_bulge <= 0.0 or
                self.interval <= 0.0):
            raise ValueError(
                "路线长度、间距、掉头外扩量和控制点间隔必须为正；"
                "mowing 路线至少需要 2 条航段")
        if (self.curve_amplitude <= 0.0 or self.curve_half_wave <= 0.0 or
                self.start_offset < 0.0):
            raise ValueError("S 弯幅值和半波长必须为正")
        if self.PUBLISH_PERIOD_S <= 0.0 or self.start_delay < 0.0:
            raise ValueError("publish_period_s 必须为正，start_delay_s 不能为负")

        self.odom_topic = self.ODOM_TOPIC
        self.bspline_topic = self.BSPLINE_TOPIC
        self.reference_path_topic = self.REFERENCE_PATH_TOPIC
        self.publisher = self.create_publisher(
            UsvBspline,
            self.bspline_topic,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # RViz2 直接显示 B 样条求值后的曲线，不显示离散控制点。
        self.path_publisher = self.create_publisher(
            Path,
            self.reference_path_topic,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.odom = None
        self.trajectory = None
        self.traj_id = 1
        self.subscription = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10)
        self.reset_subscription = self.create_subscription(
            UInt64, "/planning/mission_reset", self._on_reset, 10)
        self.timer = self.create_timer(self.PUBLISH_PERIOD_S, self._publish)
        self.get_logger().info(
            f"等待 {self.odom_topic}；收到后发布 {self.bspline_topic} 和 "
            f"{self.reference_path_topic} 测试轨迹")

    def _on_odom(self, message):
        """缓存匹配坐标系的里程计，仅在尚无路线时构造并显示测试路线."""
        if message.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"忽略坐标系为 {message.header.frame_id!r} 的里程计，"
                f"期望 {self.frame_id!r}")
            return
        self.odom = message
        if self.trajectory is None:
            self.trajectory = self._make_trajectory(message)
            self._publish_reference_path()
            self.get_logger().info(
                f"测试 B 样条已生成：{len(self.trajectory.control_points)} 个控制点，"
                f"时长 {(len(self.trajectory.control_points) - 3) * self.interval:.1f} s")

    def _on_reset(self, _message):
        """任务重置后用最新里程计生成新起点，避免重发旧 start_time."""
        self.traj_id += 1
        self.trajectory = None
        self.get_logger().info("收到任务重置，将从最新 /odom 重新生成测试样条")

    def _make_trajectory(self, odom):
        """在船舶初始局部坐标系中生成平滑测试路线."""
        degree = 3
        spacing = self.speed * self.interval
        start = odom.pose.pose.position
        heading = quaternion_yaw(odom.pose.pose.orientation)
        route_heading_deg = os.environ.get("USV_NMPC_REFERENCE_HEADING_DEG")
        if route_heading_deg is not None:
            heading = math.radians(90.0 - float(route_heading_deg))
        forward = (math.cos(heading), math.sin(heading))
        left = (-math.sin(heading), math.cos(heading))

        def world_point(x, y):
            return (
                start.x + self.start_offset * forward[0] + x * forward[0] + y * left[0],
                start.y + self.start_offset * forward[1] + x * forward[1] + y * left[1],
            )

        # 先建立连续的几何路线及其弧长查找表，再按固定弧长间隔取控制点。
        # 这样直线和掉头处的样条控制点间距一致，降低参考速度起伏。
        local_samples = [(0.0, 0.0)]
        cumulative_distance = [0.0]

        def append_local_point(point):
            previous = local_samples[-1]
            distance = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if distance <= 1e-9:
                return
            local_samples.append(point)
            cumulative_distance.append(cumulative_distance[-1] + distance)

        if self.route_mode == "straight":
            append_local_point((self.lane_length, 0.0))
        elif self.route_mode == "reverse_straight":
            append_local_point((-self.lane_length, 0.0))
        elif self.route_mode == "s_curve":
            sample_count = max(100, int(math.ceil(self.lane_length / 0.25)))
            for index in range(1, sample_count + 1):
                x = self.lane_length * index / sample_count
                y = 0.5 * self.curve_amplitude * (
                    1.0 - math.cos(math.pi * x / self.curve_half_wave))
                append_local_point((x, y))
        elif self.route_mode in {"arc_left", "arc_right"}:
            direction = 1.0 if self.route_mode == "arc_left" else -1.0
            radius = self.curve_amplitude
            sample_count = max(100, int(math.ceil(self.lane_length / 0.25)))
            for index in range(1, sample_count + 1):
                angle = self.lane_length * index / sample_count / radius
                append_local_point((
                    radius * math.sin(angle),
                    direction * radius * (1.0 - math.cos(angle)),
                ))
        else:
            direction = 1.0
            current_x = 0.0
            current_y = 0.0

            for lane in range(self.lane_count):
                target_x = self.lane_length if direction > 0.0 else 0.0
                append_local_point((target_x, current_y))
                current_x = target_x
                if lane + 1 >= self.lane_count:
                    break

                # 正弦/半余弦组合得到平滑半圆掉头，两端与直线切向相接。
                turn_steps = max(200, int(math.ceil(
                    (math.pi * self.turn_bulge + self.lane_spacing) / 0.1)))
                for index in range(1, turn_steps + 1):
                    fraction = index / turn_steps
                    append_local_point((
                        current_x + direction * self.turn_bulge *
                        math.sin(math.pi * fraction),
                        current_y + self.lane_spacing * 0.5 *
                        (1.0 - math.cos(math.pi * fraction)),
                    ))
                current_y += self.lane_spacing
                direction *= -1.0

        route_length = cumulative_distance[-1]
        segments = max(2, int(math.ceil(route_length / spacing)))

        def local_point_at(distance):
            """按弧长插值，并沿两端切向延伸路线."""
            if distance < 0.0:
                return (distance, 0.0)
            if distance > route_length:
                x0, y0 = local_samples[-2]
                x1, y1 = local_samples[-1]
                length = math.hypot(x1 - x0, y1 - y0)
                extra = distance - route_length
                return (x1 + extra * (x1 - x0) / length,
                        y1 + extra * (y1 - y0) / length)
            index = bisect.bisect_left(cumulative_distance, distance)
            if index <= 0:
                return local_samples[0]
            if index >= len(local_samples):
                return local_samples[-1]
            low_distance = cumulative_distance[index - 1]
            high_distance = cumulative_distance[index]
            ratio = (distance - low_distance) / (high_distance - low_distance)
            low = local_samples[index - 1]
            high = local_samples[index]
            return (low[0] + ratio * (high[0] - low[0]),
                    low[1] + ratio * (high[1] - low[1]))

        waypoints = [
            world_point(*local_point_at(index * spacing))
            for index in range(segments + 1)
        ]
        # 非夹持均匀三次 B 样条在有效区间两端各需要一个切向点。
        first = waypoints[0]
        second = waypoints[1]
        last = waypoints[-1]
        penultimate = waypoints[-2]
        controls = [
            tuple(2.0 * first[axis] - second[axis] for axis in range(2)),
            *waypoints,
            tuple(2.0 * last[axis] - penultimate[axis] for axis in range(2)),
        ]

        def tangent_yaw(index):
            if index == 0:
                a, b = waypoints[0], waypoints[1]
            elif index >= len(waypoints) - 1:
                a, b = waypoints[-2], waypoints[-1]
            else:
                a, b = waypoints[index - 1], waypoints[index + 1]
            # 内部几何切线是东向为 0 的数学角；消息发送导航角 phi，北为 0、
            # 东为 -pi/2，因此 phi = psi - pi/2。
            return math.atan2(b[1] - a[1], b[0] - a[0]) - math.pi / 2.0

        yaw_offset = math.pi if self.route_mode == "reverse_straight" else 0.0
        phi_waypoints = [
            tangent_yaw(index) + yaw_offset for index in range(len(waypoints))]
        for index in range(1, len(phi_waypoints)):
            delta = phi_waypoints[index] - phi_waypoints[index - 1]
            phi_waypoints[index] = phi_waypoints[index - 1] + math.atan2(
                math.sin(delta), math.cos(delta))
        phi_waypoints = [phi_waypoints[0], *phi_waypoints,
                         phi_waypoints[-1]]
        # 重复末端控制点，使有限测试路线在终点的变化率归零。
        controls.extend([controls[-1]] * degree)
        phi_waypoints.extend([phi_waypoints[-1]] * degree)
        # 节点向量必须覆盖追加的停车控制点，长度为控制点数 + 次数 + 1。
        count = len(controls)

        trajectory = UsvBspline()
        trajectory.order = degree
        trajectory.traj_id = self.traj_id
        start = self.get_clock().now() + Duration(seconds=self.start_delay)
        trajectory.start_time = start.to_msg()
        trajectory.knots = [
            (index - degree) * self.interval
            for index in range(count + degree + 1)
        ]
        trajectory.control_points = [
            UsvBsplineControlPoint(x=float(x), y=float(y), phi=float(phi))
            for (x, y), phi in zip(controls, phi_waypoints)
        ]
        self.get_logger().info(
            f"已生成 {self.route_mode} 轨迹：{self.lane_count} 条直线航段，"
            f"航段长度 {self.lane_length:.1f} m，横向间距 {self.lane_spacing:.1f} m，"
            f"样条起点前移 {self.start_offset:.1f} m，控制点 {count} 个，"
            f"预计覆盖长度约 {segments * spacing:.1f} m")
        return trajectory

    @staticmethod
    def _deboor(controls, knots, degree, time):
        """采用 de Boor 算法求值，使 RViz 显示与发布的样条一致."""
        last = len(controls) - 1
        start = knots[degree]
        end = knots[len(controls)]
        time = min(max(time, start), end)
        span = degree if time < end else last
        while span < last and time >= knots[span + 1]:
            span += 1
        work = [list(controls[span - degree + j]) for j in range(degree + 1)]
        for level in range(1, degree + 1):
            for j in range(degree, level - 1, -1):
                left = span - degree + j
                right = span + j - level + 1
                denominator = knots[right] - knots[left]
                alpha = ((time - knots[left]) / denominator
                         if denominator > 1e-12 else 0.0)
                work[j] = [
                    (1.0 - alpha) * work[j - 1][axis] + alpha * work[j][axis]
                    for axis in range(3)
                ]
        return work[-1]

    def _publish_reference_path(self):
        """发布采样路径，供 RViz 转换到固定坐标系后显示."""
        if self.trajectory is None:
            return
        degree = self.trajectory.order
        controls = [(point.x, point.y, point.phi)
                    for point in self.trajectory.control_points]
        knots = list(self.trajectory.knots)
        start = knots[degree]
        end = knots[len(controls)]
        sample_interval = min(0.5, max(0.1, self.interval / 2.0))
        sample_count = max(2, int(math.ceil((end - start) / sample_interval)))
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        for index in range(sample_count + 1):
            time = start + (end - start) * index / sample_count
            x, y, phi = self._deboor(controls, knots, degree, time)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            # Path 的标准姿态以本体 +x 为前；RFU 船首沿 +y，故转成 ENU yaw。
            map_yaw = phi + math.pi / 2.0
            pose.pose.orientation.z = math.sin(0.5 * map_yaw)
            pose.pose.orientation.w = math.cos(0.5 * map_yaw)
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish(self):
        """首次获得里程计后构造测试路线，后续周期重复发布同一条样条."""
        if self.trajectory is None and self.odom is not None:
            self.trajectory = self._make_trajectory(self.odom)
            self._publish_reference_path()
        if self.trajectory is None:
            return
        self.publisher.publish(self.trajectory)


def main(args=None):
    """初始化 ROS，运行测试参考发布器，并在退出时释放节点."""
    rclpy.init(args=args)
    node = BsplineReferencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
