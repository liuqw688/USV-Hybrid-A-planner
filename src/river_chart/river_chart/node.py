from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

from geometry_msgs.msg import Point, PoseArray
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .parser import Chart, ChartFeature, Geometry, ChartParseError, load_chart


# [功能与联系] 定位安装包海图资源，供启动参数默认路径使用，避免依赖源码绝对路径。
def _default_input_path() -> str:
    """@brief 获取安装无关的默认海图路径。

    @return 已安装包的 data/ecdis_data.xml 路径；源码测试时返回包内资源路径。
    """
    try:
        return str(Path(get_package_share_directory("river_chart")) / "data" / "ecdis_data.xml")
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "resource" / "ecdis_data.xml")
_CHART_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_GROUP_COLORS: dict[str, tuple[float, float, float]] = {
    "DEPARE": (0.16, 0.47, 0.90),
    "LNDARE": (0.72, 0.55, 0.24),
    "LNDRGN": (0.78, 0.62, 0.28),
    "DEPCNT": (0.25, 0.72, 0.90),
    "TSSBND": (0.10, 1.00, 0.35),
    "TSELNE": (0.10, 1.00, 0.35),
    "TSSLPT": (0.10, 1.00, 0.35),
    "ACHARE": (1.00, 0.55, 0.05),
    "BERTHS": (1.00, 0.10, 0.75),
    "BRIDGE": (1.00, 0.10, 0.10),
    "BOYSPP": (1.00, 0.90, 0.05),
    "BUAARE": (0.85, 0.35, 0.22),
    "ROADWY": (0.90, 0.85, 0.22),
    "RESARE": (0.65, 0.40, 0.85),
    "SLOTOP": (0.96, 0.40, 0.70),
    "SOUNDG": (0.25, 0.86, 0.65),
    "LIGHTS": (1.00, 0.95, 0.35),
}

_HIDDEN_VISUAL_GROUPS = {"M_NPUB", "M_NSYS", "M_QUAL", "GENERIC", "ACHARE"}
_LAND_BOUNDARY_GROUPS = {"LNDARE", "LNDRGN"}
_NAVIGATION_FOCUS_GROUPS = {
    "TSSBND",
    "TSELNE",
    "TSSLPT",
    "BOYSPP",
    "BRIDGE",
    "BERTHS",
    "DEPCNT",
}
_STATIC_OBSTACLE_GROUPS = {"LNDARE"}
_CHANNEL_GROUPS = {"TSSBND", "TSELNE", "TSSLPT"}


# [功能与联系] 将海图类别映射为显示颜色；只影响Marker，不改变航道语义或安全栅格。
def _color_for_group(group: str) -> tuple[float, float, float]:
    color = _GROUP_COLORS.get(group.upper())
    if color is not None:
        return color
    digest = hashlib.sha1(group.encode("utf-8")).digest()
    return (
        0.30 + digest[0] / 255.0 * 0.65,
        0.30 + digest[1] / 255.0 * 0.65,
        0.30 + digest[2] / 255.0 * 0.65,
    )


# [功能与联系] 构造Marker几何点；绘图辅助函数。
def _point(x: float, y: float, z: float = 0.0) -> Point:
    result = Point()
    result.x = float(x)
    result.y = float(y)
    result.z = float(z)
    return result


# [功能与联系] 遍历海图所有几何坐标，供规划/显示网格边界估计。
def _all_points(chart: Chart) -> Iterable[tuple[float, float]]:
    for feature in chart.features:
        for geometry in feature.geometry:
            for part in geometry.parts:
                yield from part


# [功能与联系] 求要素显示中心，用于标签/点显示；不是导航目标或航道中心线。
def _centroid(feature: ChartFeature) -> tuple[float, float] | None:
    points = list(
        point
        for geometry in feature.geometry
        for part in geometry.parts
        for point in part
    )
    if not points:
        return None
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


# [功能与联系] 用扇形三角片生成填充显示；不是复杂凹多边形的严格剖分或碰撞模型。
def _fan_triangles(ring: tuple[tuple[float, float], ...]) -> list[Point]:
    if len(ring) < 3:
        return []
    points = list(ring)
    if points[0] == points[-1]:
        points.pop()
    if len(points) < 3:
        return []
    result: list[Point] = []
    anchor = _point(*points[0])
    for index in range(1, len(points) - 1):
        result.extend((anchor, _point(*points[index]), _point(*points[index + 1])))
    return result


class RiverChartNode(Node):
    """Decode the chart and publish visualization messages."""

    # [功能与联系] 读取本节点参数并创建ROS输入输出/调度；后续回调与主循环关系见包内功能说明。
    def __init__(self) -> None:
        super().__init__("river_chart_node")
        self.declare_parameter("input_path", _default_input_path())
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("origin_longitude", float("nan"))
        self.declare_parameter("origin_latitude", float("nan"))
        
        self.declare_parameter("scale", 15.0)
        self.declare_parameter("line_width", 0.2)
        self.declare_parameter("point_size", 0.15)
        
        self.declare_parameter("fill_polygons", False)
        self.declare_parameter("show_land_boundaries", False)
        self.declare_parameter("focus_navigation_features", True)
        self.declare_parameter("publish_labels", False)
        self.declare_parameter("publish_occupancy_grid", False)
        
        # 【核心修改】分辨率提高到 0.2m
        self.declare_parameter("grid_resolution", 0.2)
        # 【核心修改】大幅提高最大栅格数限制，以容纳 0.2m 分辨率下的庞大网格 (约1500万个)
        self.declare_parameter("max_grid_cells", 20_000_000)
        
        self.declare_parameter("reload_period", 0.0)
        self.declare_parameter("marker_topic", "chart_markers")
        self.declare_parameter("metadata_topic", "chart_metadata")
        self.declare_parameter("occupancy_topic", "chart_occupancy")
        self.declare_parameter("publish_planning_layers", True)
        
        # 【核心修改】规划分辨率提高到 0.2m
        self.declare_parameter("planning_resolution", 0.2)
        # 【核心修改】大幅提高规划最大栅格数限制
        self.declare_parameter("planning_max_cells", 20_000_000)
        
        # 物理尺寸参数保持不变（因为坐标系已经是 1单位=1米）
        self.declare_parameter("obstacle_inflation", 1.0)
        self.declare_parameter("channel_corridor_width", 20.0)
        self.declare_parameter("publish_channel_direction", False)
        self.declare_parameter("channel_outside_cost", 60)
        self.declare_parameter("future_obstacle_topic", "future_obstacles")
        self.declare_parameter("dynamic_obstacle_radius", 1.0)

        marker_topic = str(self.get_parameter("marker_topic").value)
        metadata_topic = str(self.get_parameter("metadata_topic").value)
        occupancy_topic = str(self.get_parameter("occupancy_topic").value)
        self._markers_pub = self.create_publisher(MarkerArray, marker_topic, _CHART_QOS)
        self._metadata_pub = self.create_publisher(String, metadata_topic, _CHART_QOS)
        self._occupancy_pub = self.create_publisher(OccupancyGrid, occupancy_topic, _CHART_QOS)
        self._static_pub = self.create_publisher(OccupancyGrid, "chart_static_obstacles", _CHART_QOS)
        self._costmap_pub = self.create_publisher(OccupancyGrid, "chart_costmap", _CHART_QOS)
        self._direction_pub = self.create_publisher(MarkerArray, "chart_channel_direction", _CHART_QOS)
        # 规划节点不应从 RViz Marker 反推 S-57 属性。这里只发布一次轻量级的
        # TSSLPT 多边形与 ORIENT，动态的“本船航道/对向航道”由独立功能包判定。
        self._lane_data_pub = self.create_publisher(String, "chart_lane_data", _CHART_QOS)
        self._future_obstacles: list[tuple[float, float]] = []
        self._future_sub = self.create_subscription(
            PoseArray, str(self.get_parameter("future_obstacle_topic").value),
            self._future_obstacles_callback, _CHART_QOS,
        )
        self._reload_timer = None
        self._chart: Chart | None = None
        self._publish_chart()
        reload_period = float(self.get_parameter("reload_period").value)
        if math.isfinite(reload_period) and reload_period > 0.0:
            self._reload_timer = self.create_timer(reload_period, self._publish_chart)

    # [功能与联系] 将非有限经纬度参数转为None，允许parser自动选择原点。
    def _float_parameter(self, name: str) -> float | None:
        value = float(self.get_parameter(name).value)
        return value if math.isfinite(value) else None

    # [功能与联系] 读取输入文件并传入原点/scale参数给load_chart；由_publish_chart调用。
    def _load(self) -> Chart:
        return load_chart(
            Path(str(self.get_parameter("input_path").value)).expanduser(),
            origin_longitude=self._float_parameter("origin_longitude"),
            origin_latitude=self._float_parameter("origin_latitude"),
            scale=float(self.get_parameter("scale").value),
        )

    # [功能与联系] 加载海图、缓存Chart并发布Marker/静态航道JSON/规划层/元数据；reload_period=0时启动执行一次。
    def _publish_chart(self) -> None:
        try:
            chart = self._load()
        except (ChartParseError, OSError, ValueError) as exc:
            self.get_logger().error(f"Unable to load S-57 chart: {exc}")
            return
        self._chart = chart
        self._markers_pub.publish(self._build_markers(chart))
        lane_data = String()
        lane_data.data = self._lane_data_json(chart)
        self._lane_data_pub.publish(lane_data)
        if bool(self.get_parameter("publish_planning_layers").value):
            self._publish_planning_layers(chart)
        metadata = String()
        metadata.data = chart.metadata_json()
        self._metadata_pub.publish(metadata)
        if bool(self.get_parameter("publish_occupancy_grid").value):
            self._occupancy_pub.publish(self._build_occupancy_grid(chart))
        self.get_logger().info(
            f"Published {chart.feature_count} features from {chart.map_count} map(s); "
            f"origin=({chart.origin_longitude:.6f}, {chart.origin_latitude:.6f})"
        )

    # [功能与联系] 提取TSSLPT多边形和ORIENT形成轻量静态JSON，供channel_navigation_manager分类；不从绿色Marker颜色反推属性。
    def _lane_data_json(self, chart: Chart) -> str:
        """返回体积很小的静态分道几何，避免下游重复解析整份海图。

        ORIENT 是航道自身的静态属性；目标改变时无需重发海图，下游只需
        根据无人船当前实际航行方向重新给这些 lane_id 分配角色。
        """
        lanes: list[dict[str, object]] = []
        lane_id = 0
        for feature in chart.features:
            if feature.group.upper() != "TSSLPT":
                continue
            try:
                orient = float(feature.attributes.get("ORIENT", ""))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(orient):
                continue
            for geometry in feature.geometry:
                if geometry.geometry_type not in {"POLYGON", "MULTIPOLYGON"}:
                    continue
                for ring in geometry.parts:
                    if len(ring) < 4:
                        continue
                    lanes.append({
                        "lane_id": lane_id,
                        "orient_deg": orient % 360.0,
                        "points": [[float(x), float(y)] for x, y in ring],
                    })
                    lane_id += 1
        return json.dumps({
            "frame_id": str(self.get_parameter("frame_id").value),
            "lanes": lanes,
        }, ensure_ascii=False, separators=(",", ":"))

    # [功能与联系] 缓存PoseArray障碍位置并重建规划层；当前写入chart_costmap，不是直接发布独立/channel/obstacle_costmap。
    def _future_obstacles_callback(self, message: PoseArray) -> None:
        self._future_obstacles = [
            (float(p.position.x), float(p.position.y)) for p in message.poses
        ]
        if self._chart is not None and bool(self.get_parameter("publish_planning_layers").value):
            self._publish_planning_layers(self._chart)

    # [功能与联系] 由全部要素坐标求规划地图外接范围，供_new_planning_grid确定网格。
    def _planning_bounds(self, chart: Chart) -> tuple[float, float, float, float]:
        points = list(_all_points(chart))
        if not points:
            return (-100.0, -100.0, 100.0, 100.0)
        return (min(x for x, _ in points), min(y for _, y in points),
                max(x for x, _ in points), max(y for _, y in points))

    # [功能与联系] 创建规划网格并按max_cells必要时降分辨率；data与坐标元信息供所有规划层共享。
    def _new_planning_grid(self, chart: Chart) -> tuple[OccupancyGrid, list[int], float, float, int, int, float]:
        min_x, min_y, max_x, max_y = self._planning_bounds(chart)
        resolution = float(self.get_parameter("planning_resolution").value)
        if not math.isfinite(resolution) or resolution <= 0.0:
            resolution = 1.0  # Fallback 改为 0.2
        width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
        height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
        max_cells = max(1, int(self.get_parameter("planning_max_cells").value))
        cells = width * height
        if cells > max_cells:
            resolution *= math.sqrt(cells / max_cells)
            width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
            height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
            self.get_logger().warn(
                f"Planning grid exceeded planning_max_cells; using {resolution:.2f} m "
                f"resolution ({width}x{height})"
            )
        grid = OccupancyGrid()
        grid.header.frame_id = str(self.get_parameter("frame_id").value)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.orientation.w = 1.0
        return grid, [-1] * (width * height), min_x, min_y, width, height, resolution

    @staticmethod
    # [功能与联系] 沿线段密采样并刷写圆形邻域，生成物理障碍边缘或航道附近底图低代价区域。
    def _raster_segment(data: list[int], width: int, height: int, min_x: float, min_y: float,
                        resolution: float, start: tuple[float, float], end: tuple[float, float],
                        value: int, radius: float = 0.0) -> None:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        # 分辨率变小后，步长自动变密，轮廓信息会更精细
        steps = max(1, int(math.ceil(distance / max(resolution * 0.5, 1e-6))))
        cell_radius = int(math.ceil(radius / resolution))
        for step in range(steps + 1):
            ratio = step / steps
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            ix, iy = int((x - min_x) / resolution), int((y - min_y) / resolution)
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    if dx * dx + dy * dy > cell_radius * cell_radius:
                        continue
                    cx, cy = ix + dx, iy + dy
                    if 0 <= cx < width and 0 <= cy < height:
                        data[cy * width + cx] = value

    # [功能与联系] 分别绘制原始陆地核心和chart_costmap，叠加预留障碍点；此栅格与主/对向角色分类是两回事。
    def _planning_grid(self, chart: Chart) -> tuple[OccupancyGrid, OccupancyGrid]:
        static, static_data, min_x, min_y, width, height, resolution = self._new_planning_grid(chart)
        for feature in chart.features:
            if feature.group.upper() not in _STATIC_OBSTACLE_GROUPS:
                continue
            for geometry in feature.geometry:
                for part in geometry.parts:
                    if len(part) < 3:
                        continue
                    self._fill_polygon(
                        static_data, width, height, min_x, min_y, resolution, part, 100
                    )
                    for start, end in zip(part, part[1:] or part[:1]):
                        self._raster_segment(
                            static_data, width, height, min_x, min_y, resolution,
                            start, end, 100,
                            float(self.get_parameter("obstacle_inflation").value),
                        )
        static.data = static_data
        cost = OccupancyGrid()
        cost.header = static.header
        cost.info = static.info
        outside_cost = max(-1, min(99, int(self.get_parameter("channel_outside_cost").value)))
        cost_data = [outside_cost] * (width * height)
        channel_radius = max(0.0, float(self.get_parameter("channel_corridor_width").value))
        for feature in chart.features:
            if feature.group.upper() not in _CHANNEL_GROUPS:
                continue
            for geometry in feature.geometry:
                for part in geometry.parts:
                    for start, end in zip(part, part[1:] or part):
                        self._raster_segment(cost_data, width, height, min_x, min_y, resolution,
                                             start, end, 0, channel_radius)
        for index, value in enumerate(static_data):
            if value == 100:
                cost_data[index] = 100
        dynamic_radius = float(self.get_parameter("dynamic_obstacle_radius").value)
        for point in self._future_obstacles:
            self._raster_segment(cost_data, width, height, min_x, min_y, resolution,
                                 point, point, 100, dynamic_radius)
        cost.data = cost_data
        return static, cost

    @staticmethod
    # [功能与联系] 扫描线填充多边形占用；构建物理静态层，不执行右行策略判定。
    def _fill_polygon(data: list[int], width: int, height: int, min_x: float,
                      min_y: float, resolution: float,
                      ring: tuple[tuple[float, float], ...], value: int) -> None:
        points = list(ring)
        if points[0] == points[-1]:
            points.pop()
        if len(points) < 3:
            return
        low_y = max(0, int(math.floor((min(y for _, y in points) - min_y) / resolution)))
        high_y = min(height - 1, int(math.ceil((max(y for _, y in points) - min_y) / resolution)))
        for iy in range(low_y, high_y + 1):
            y = min_y + (iy + 0.5) * resolution
            intersections: list[float] = []
            for first, second in zip(points, points[1:] + points[:1]):
                x1, y1 = first
                x2, y2 = second
                if (y1 > y) != (y2 > y):
                    intersections.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
            intersections.sort()
            for left, right in zip(intersections[::2], intersections[1::2]):
                low_x = max(0, int(math.floor((left - min_x) / resolution)))
                high_x = min(width - 1, int(math.ceil((right - min_x) / resolution)))
                for ix in range(low_x, high_x + 1):
                    data[iy * width + ix] = value

    # [功能与联系] 按几何线段生成方向显示箭头；不等同于TSSLPT ORIENT主航道判定。
    def _build_direction_markers(self, chart: Chart) -> MarkerArray:
        result = MarkerArray()
        delete = Marker(); delete.action = Marker.DELETEALL; result.markers.append(delete)
        marker_id = 1
        for feature in chart.features:
            if feature.group.upper() not in _CHANNEL_GROUPS:
                continue
            for geometry in feature.geometry:
                for part in geometry.parts:
                    for start, end in zip(part, part[1:]):
                        length = math.hypot(end[0] - start[0], end[1] - start[1])
                        if length < 30.0:
                            continue
                        marker = self._base_marker(chart, "channel_direction", marker_id, Marker.ARROW)
                        marker.pose.position.x = (start[0] + end[0]) / 2.0
                        marker.pose.position.y = (start[1] + end[1]) / 2.0
                        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
                        marker.pose.orientation.z = math.sin(yaw / 2.0)
                        marker.pose.orientation.w = math.cos(yaw / 2.0)
                        marker.scale.x = min(250.0, length * 0.7)
                        marker.scale.y = 3.0; marker.scale.z = 3.0
                        self._set_color(marker, "TSELNE")
                        result.markers.append(marker); marker_id += 1
        return result

    # [功能与联系] 发布静态核心图和chart_costmap，后者由local_costmap_generator膨胀；可选方向Marker用于显示。
    def _publish_planning_layers(self, chart: Chart) -> None:
        static, cost = self._planning_grid(chart)
        self._static_pub.publish(static)
        self._costmap_pub.publish(cost)
        if bool(self.get_parameter("publish_channel_direction").value):
            self._direction_pub.publish(self._build_direction_markers(chart))

    # [功能与联系] 填入Marker头、id、namespace、类型及基础姿态，供各海图显示要素复用。
    def _base_marker(
        self, chart: Chart, namespace: str, marker_id: int, marker_type: int
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = str(self.get_parameter("frame_id").value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.9
        marker.scale.x = max(0.001, float(self.get_parameter("line_width").value))
        return marker

    # [功能与联系] 按类别和透明度设置Marker颜色，供_build_markers共用。
    def _set_color(self, marker: Marker, group: str, alpha: float | None = None) -> None:
        marker.color.r, marker.color.g, marker.color.b = _color_for_group(group)
        if alpha is not None:
            marker.color.a = alpha

    # [功能与联系] 将线/面/点/标签转换为矢量海图MarkerArray供RViz；不是可通行栅格生成函数。
    def _build_markers(self, chart: Chart) -> MarkerArray:
        result = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        result.markers.append(delete)
        marker_id = 1
        fill_polygons = bool(self.get_parameter("fill_polygons").value)
        show_land_boundaries = bool(self.get_parameter("show_land_boundaries").value)
        focus_navigation_features = bool(
            self.get_parameter("focus_navigation_features").value
        )
        publish_labels = bool(self.get_parameter("publish_labels").value)
        point_size = max(0.001, float(self.get_parameter("point_size").value))
        for feature in chart.features:
            group_upper = feature.group.upper()
            if group_upper in _HIDDEN_VISUAL_GROUPS:
                continue
            if not show_land_boundaries and group_upper in _LAND_BOUNDARY_GROUPS:
                continue
            if focus_navigation_features and group_upper not in _NAVIGATION_FOCUS_GROUPS:
                continue
            namespace = f"s57/{feature.group}"
            for geometry in feature.geometry:
                if feature.feature_type == "P" or geometry.geometry_type in {
                    "POINT",
                    "MULTIPOINT",
                }:
                    marker = self._base_marker(chart, namespace, marker_id, Marker.POINTS)
                    marker.scale.x = point_size
                    marker.scale.y = point_size
                    self._set_color(marker, feature.group)
                    marker.points = [_point(x, y) for part in geometry.parts for x, y in part]
                    if marker.points:
                        result.markers.append(marker)
                        marker_id += 1
                    continue
                for ring in geometry.parts:
                    if feature.feature_type == "P" or len(ring) == 1:
                        marker = self._base_marker(chart, namespace, marker_id, Marker.POINTS)
                        marker.scale.x = point_size
                        marker.scale.y = point_size
                        self._set_color(marker, feature.group)
                        marker.points = [_point(*ring[0])]
                        result.markers.append(marker)
                        marker_id += 1
                        continue
                    if len(ring) < 2:
                        continue
                    drawable_points = ring
                    if feature.feature_type == "L" and ring[0] == ring[-1]:
                        drawable_points = ring[:-1]
                    if len(drawable_points) < 2:
                        continue
                    marker = self._base_marker(chart, namespace, marker_id, Marker.LINE_STRIP)
                    self._set_color(marker, feature.group)
                    marker.points = [_point(x, y) for x, y in drawable_points]
                    if (
                        feature.feature_type not in {"L", "S", "O"}
                        and geometry.geometry_type in {"POLYGON", "MULTIPOLYGON"}
                        and ring[0] != ring[-1]
                    ):
                        marker.points.append(marker.points[0])
                    result.markers.append(marker)
                    marker_id += 1
                    if (
                        fill_polygons
                        and feature.feature_type == "A"
                        and geometry.geometry_type in {"POLYGON", "MULTIPOLYGON"}
                    ):
                        triangles = _fan_triangles(ring)
                        if triangles:
                            fill = self._base_marker(
                                chart, namespace, marker_id, Marker.TRIANGLE_LIST
                            )
                            self._set_color(fill, feature.group, alpha=0.20)
                            fill.points = triangles
                            result.markers.append(fill)
                            marker_id += 1
            if publish_labels:
                center = _centroid(feature)
                if center is not None and feature.name:
                    text = self._base_marker(
                        chart,
                        f"labels/{feature.group}",
                        marker_id,
                        Marker.TEXT_VIEW_FACING,
                    )
                    self._set_color(text, feature.group)
                    text.pose.position.x, text.pose.position.y = center
                    text.scale.z = max(0.5, point_size * 1.5)
                    text.text = feature.name
                    result.markers.append(text)
                    marker_id += 1
        return result

    # [功能与联系] 创建可选显示占用图chart_occupancy；默认关闭，与chart_costmap规划输入分开。
    def _build_occupancy_grid(self, chart: Chart) -> OccupancyGrid:
        points = list(_all_points(chart))
        grid = OccupancyGrid()
        grid.header.frame_id = str(self.get_parameter("frame_id").value)
        grid.header.stamp = self.get_clock().now().to_msg()
        if not points:
            return grid
        resolution = float(self.get_parameter("grid_resolution").value)
        if not math.isfinite(resolution) or resolution <= 0.0:
            resolution = 0.2  # Fallback 改为 0.2
        min_x = min(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_x = max(point[0] for point in points)
        max_y = max(point[1] for point in points)
        width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
        height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
        max_cells = max(1, int(self.get_parameter("max_grid_cells").value))
        cells = width * height
        if cells > max_cells:
            factor = math.sqrt(cells / max_cells)
            resolution *= factor
            width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
            height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
            self.get_logger().warn(
                f"Occupancy grid exceeded max_grid_cells; using {resolution:.2f} m "
                f"resolution ({width}x{height})"
            )
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.orientation.w = 1.0
        data = [-1] * (width * height)

        # [功能与联系] 在可选显示占用图中刷写一个格子；与用于实际规划的_planning_grid区分。
        def mark(x: float, y: float) -> None:
            ix = int((x - min_x) / resolution)
            iy = int((y - min_y) / resolution)
            if 0 <= ix < width and 0 <= iy < height:
                data[iy * width + ix] = 100

        for feature in chart.features:
            for geometry in feature.geometry:
                for part in geometry.parts:
                    if not part:
                        continue
                    mark(*part[0])
                    for start, end in zip(part, part[1:]):
                        distance = math.hypot(end[0] - start[0], end[1] - start[1])
                        steps = max(1, int(math.ceil(distance / max(resolution * 0.5, 1e-6))))
                        for step in range(1, steps + 1):
                            ratio = step / steps
                            mark(
                                start[0] + (end[0] - start[0]) * ratio,
                                start[1] + (end[1] - start[1]) * ratio,
                            )
        grid.data = data
        return grid


# [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RiverChartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["RiverChartNode", "main"]
