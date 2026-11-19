"""启动编辑工具共用的海图、GNSS 航线和 RViz2."""

import atexit
import os
import tempfile
from pathlib import Path
from urllib.parse import quote

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def workspace_root():
    """从启动目录向上查找源工作空间，支持在子目录启动."""
    for location in (Path.cwd(), Path(__file__).resolve().parent):
        for candidate in (location, *location.parents):
            if (
                (candidate / "src" / "usv_map" / "package.xml").is_file()
                and (candidate / "src" / "usv_map_tools" / "package.xml").is_file()
            ):
                return candidate
    return Path.cwd()


def resolve_tile_cache(value, package_share):
    """
    解析瓦片缓存路径，优先使用源工作空间中已存在的目录.

    :param value: 缓存路径，支持用户目录及相对路径。
    :param package_share: 安装包共享目录，也是未找到目录时的回退基准。

    :return: 首个已存在的候选目录；都不存在时返回包共享目录下的候选路径。
    """
    cache_path = Path(value).expanduser()
    if cache_path.is_absolute():
        return cache_path
    candidates = []
    root = workspace_root()
    if root is not None:
        candidates.extend((root / "src" / "usv_map" / cache_path, root / cache_path))
    candidates.extend((Path.cwd() / cache_path, Path.cwd() / "src" / "usv_map" / cache_path))
    candidates.append(package_share / cache_path)
    candidates = list(dict.fromkeys(candidates))
    return next((path for path in candidates if path.is_dir()), candidates[-1])


def configure_satellite(display, config, package_share):
    """配置 RViz 卫星图层，支持本地缓存或在线瓦片."""
    mode = str(config.get("rviz_satellite_mode", "local")).strip().lower()
    if mode == "disabled":
        display["Enabled"] = False
        return

    if mode == "local":
        cache_value = str(config.get("tile_cache", "cache/tianditu"))
        cache_dir = resolve_tile_cache(cache_value, package_share)
        layer = str(config.get("rviz_satellite_layer", "img_w")).strip()
        zoom = int(config.get("rviz_satellite_zoom", config.get("zoom", 16)))
        tile = next(cache_dir.glob(f"{layer}_*.tile"), None) if cache_dir.is_dir() else None
        display["Enabled"] = tile is not None
        display["Zoom"] = zoom
        display["Object URI"] = (
            cache_dir.as_uri() + f"/{layer}_{{z}}_{{x}}_{{y}}.tile"
        )
        if tile is None:
            print(f"未找到本地天地图瓦片：{cache_dir}（卫星图层已关闭）")
        return

    if mode != "online":
        raise RuntimeError(
            "water_map.yaml 的 rviz_satellite_mode 必须是 local、online 或 disabled"
        )

    token = str(config.get("tianditu_api_key") or "").strip()
    if not token:
        token = os.environ.get("TIANDITU_TK", "").strip()
    display["Enabled"] = bool(token)
    display["Object URI"] = display["Object URI"].replace(
        "{TIANDITU_TK}", quote(token, safe="")
    )


def resolve_path(value, root):
    """
    展开用户目录，并以工作空间根目录解析相对路径.

    :param value: 文件或目录路径。
    :param root: 相对路径基准目录。

    :return: 解析后的绝对路径。
    """
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def launch_nodes(context):
    """
    根据编辑模式和输入输出路径创建编辑器、海图、航线及显示节点.

    :param context: ROS launch 上下文，提供编辑模式和路径覆盖参数。

    :return: 待执行的启动动作列表。
    """
    river_share = Path(get_package_share_directory("usv_map"))
    tools_share = Path(get_package_share_directory("usv_map_tools"))

    root = workspace_root()
    editor = LaunchConfiguration("editor").perform(context)
    berth_mode, reef_mode = editor == "berth", editor == "reef"
    editor_name, config_name = {
        "no_go": ("no_go_area_editor", "area_editor.yaml"),
        "berth": ("berth_designer", "berth_designer.yaml"),
        "reef": ("reef_designer", "reef_designer.yaml"),
    }[editor]
    editor_config = tools_share / "config" / config_name
    parameters = yaml.safe_load(editor_config.read_text(encoding="utf-8"))[
        editor_name]["ros__parameters"]

    output_chart = LaunchConfiguration("output_chart").perform(context).strip()
    output_chart = resolve_path(output_chart or parameters["output_chart"], root)
    input_chart = LaunchConfiguration("input_chart").perform(context).strip()
    input_chart = input_chart or parameters.get("input_chart", "")
    generated = root / "src" / "usv_map" / "resource" / "generated"
    candidates = []
    if input_chart:
        candidates.append(resolve_path(input_chart, root))
    else:
        if reef_mode:
            candidates.append(generated / "reservoir_test_berths.xml")
        if berth_mode or reef_mode:
            candidates.append(generated / "reservoir_test_blocked.xml")
        candidates.extend([
            generated / "reservoir_test_vec.xml",
            river_share / "data/reservoir_test_vec.xml",
        ])
    base_chart = next((path for path in candidates if path.is_file()), None)
    active_chart = output_chart if output_chart.is_file() else base_chart
    if active_chart is None or (berth_mode and base_chart is None):
        raise RuntimeError(f"找不到输入海图 XML：{input_chart or candidates[0]}")
    chart_path = output_chart if reef_mode else (base_chart if berth_mode else active_chart)
    editor_parameters = {"input_chart": str(active_chart), "output_chart": str(output_chart)}
    if berth_mode:
        editor_parameters["base_chart"] = str(base_chart)
    if reef_mode:
        render_parameters = yaml.safe_load(
            (river_share / "config" / "usv_map.yaml").read_text(encoding="utf-8")
        )["usv_map_node"]["ros__parameters"]
        for key in (
            "line_width", "point_size", "planning_resolution", "planning_max_cells",
            "obstacle_inflation", "channel_corridor_width", "channel_outside_cost",
        ):
            editor_parameters[key] = render_parameters[key]

    chart_node = Node(
        package="usv_map",
        executable="usv_map_node",
        name="usv_map_node",
        output="screen",
        parameters=[
            str(river_share / "config" / "usv_map.yaml"),
            {"input_path": str(chart_path), "focus_navigation_features": False},
        ],
    )
    editor_node = Node(
        package="usv_map_tools",
        executable=editor_name,
        name=editor_name,
        output="screen",
        parameters=[
            str(editor_config),
            editor_parameters,
        ],
    )

    gnss_config = tools_share / "config" / "gnss_path.yaml"
    gnss_parameters = yaml.safe_load(gnss_config.read_text(encoding="utf-8"))[
        "gnss_path_publisher"]["ros__parameters"]
    fairway_chart = resolve_path(gnss_parameters["output_chart"], root)
    path_node = Node(
        package="usv_map_tools",
        executable="gnss_path_publisher",
        name="gnss_path_publisher",
        output="screen",
        parameters=[
            str(gnss_config),
            {"chart_file": str(chart_path), "output_chart": str(fairway_chart),
             "preserve_fairway": reef_mode},
        ],
    )

    # 暗礁编辑器在每次点击后发布完整地图，代价地图随之更新。
    actions = [editor_node, path_node] if reef_mode else [chart_node, editor_node, path_node]
    if LaunchConfiguration("rviz").perform(context).lower() not in {"false", "0"}:
        map_config = yaml.safe_load(
            (river_share / "config" / "water_map.yaml").read_text(encoding="utf-8")
        ) or {}
        rviz_path = tools_share / "rviz" / "area_editor.rviz"
        config = yaml.safe_load(rviz_path.read_text(encoding="utf-8"))
        manager = config["Visualization Manager"]
        displays = manager["Displays"]
        if berth_mode:
            selected = next(display for display in displays if display["Name"] == "Selected areas")
            selected["Name"] = "Berths"
            selected["Topic"]["Value"] = "/test_tools/berths"
            chart = next(display for display in displays if display["Name"] == "Segmented chart")
            chart["Namespaces"]["s57/BERTHS"] = True
            manager["Tools"] = [tool for tool in manager["Tools"] if tool["Class"] not in {
                "rviz_default_plugins/SetInitialPose", "rviz_default_plugins/PublishPoint"}]
            goal = next(tool for tool in manager["Tools"] if tool["Class"] == "rviz_default_plugins/SetGoal")
            goal["Topic"]["Value"] = "/test_tools/berth_pose"
            expanded = config["Panels"][0]["Property Tree Widget"]["Expanded"]
            expanded[expanded.index("/Selected areas1")] = "/Berths1"
        if reef_mode:
            selected = next(display for display in displays if display["Name"] == "Selected areas")
            selected["Name"] = "Reefs"
            selected["Topic"]["Value"] = "/test_tools/reefs"
            manager["Tools"] = [tool for tool in manager["Tools"] if tool["Class"] not in {
                "rviz_default_plugins/SetInitialPose", "rviz_default_plugins/SetGoal"}]
            publish_point = next(tool for tool in manager["Tools"] if
                                 tool["Class"] == "rviz_default_plugins/PublishPoint")
            publish_point["Topic"] = "/test_tools/reef_point"
            publish_point["Single click"] = True
            expanded = config["Panels"][0]["Property Tree Widget"]["Expanded"]
            expanded[expanded.index("/Selected areas1")] = "/Reefs1"
        satellite = next(display for display in displays if display["Name"] == "Tianditu satellite")
        configure_satellite(satellite, map_config, river_share)

        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", suffix=".rviz", prefix="usv_map_test_", delete=False
        ) as temporary_config:
            yaml.safe_dump(config, temporary_config, sort_keys=False, allow_unicode=True)
            temporary_path = Path(temporary_config.name)
        atexit.register(lambda path=temporary_path: path.unlink(missing_ok=True))

        actions.extend([
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_origin_frame",
                arguments=["--frame-id", "map", "--child-frame-id", "map_origin"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gps_sensor_frame",
                arguments=["--frame-id", "map", "--child-frame-id", "gps_sensor"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name=f"{editor_name}_rviz",
                arguments=["-d", str(temporary_path)],
                output="screen",
            ),
        ])
    return actions


def generate_launch_description():
    """返回海图编辑器及其地图、航线和 RViz 配套节点的启动描述."""
    return LaunchDescription([
        DeclareLaunchArgument("input_chart", default_value=""),
        DeclareLaunchArgument(
            "output_chart", default_value="",
        ),
        DeclareLaunchArgument("editor", default_value="no_go", choices=["no_go", "berth", "reef"]),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_nodes),
    ])
