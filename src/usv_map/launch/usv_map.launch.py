"""加载海图配置并启动地图节点，可选启用天地图图层."""

import atexit
import os
import tempfile
from pathlib import Path
from urllib.parse import quote

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def workspace_root():
    """从启动目录向上查找源工作空间，支持在子目录启动."""
    for location in (Path.cwd(), Path(__file__).resolve().parent):
        for candidate in (location, *location.parents):
            if (candidate / "src" / "usv_map" / "package.xml").is_file():
                return candidate
    return None


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


def launch_chart_nodes(
    context, config_path, rviz_path, water_map_path, channel_navigation_launch
):
    """
    按启动参数创建海图及显示节点，并生成临时 RViz 配置.

    :param context: ROS launch 上下文。
    :param config_path: 地图节点参数文件的路径替换表达式。
    :param rviz_path: RViz 配置路径替换表达式。
    :param water_map_path: 水域及卫星图层配置路径替换表达式。
    :param channel_navigation_launch: 可选的航道管理启动动作。

    :return: 待执行的节点和启动动作列表；临时 RViz 配置在进程退出时删除。
    """
    parameters = [config_path]
    input_path = LaunchConfiguration("input_path").perform(context)
    if input_path:
        parameters.append({"input_path": input_path})

    chart_node = Node(
        package="usv_map",
        executable="usv_map_node",
        name="usv_map_node",
        output=LaunchConfiguration("output"),
        parameters=parameters,
    )
    odometry_node = Node(
        package="usv_map",
        executable="usv_odometry_node",
        name="usv_odometry_node",
        output=LaunchConfiguration("output"),
        parameters=[config_path],
    )
    # 海图以 map 为根坐标系；持续发布单位变换，使关闭 RViz 时也可查询坐标树。
    map_origin_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_origin_frame",
        arguments=["--frame-id", "map", "--child-frame-id", "map_origin"],
        output=LaunchConfiguration("output"),
    )
    gps_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_gps_sensor_frame",
        arguments=["--frame-id", "map", "--child-frame-id", "gps_sensor"],
        output=LaunchConfiguration("output"),
    )
    model_urdf = PathJoinSubstitution([
        FindPackageShare("usv_map"), "urdf", "wam_v_display.urdf"
    ])
    vessel_display = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="usv_robot_state_publisher",
        output=LaunchConfiguration("output"),
        parameters=[{
            "robot_description": ParameterValue(Command(["cat ", model_urdf]), value_type=str)
        }],
    )
    actions = [chart_node, odometry_node, map_origin_tf, gps_sensor_tf, vessel_display]
    if LaunchConfiguration("channel_navigation").perform(context).lower() not in {"false", "0"}:
        actions.append(channel_navigation_launch)
    if LaunchConfiguration("rviz").perform(context).lower() in {"false", "0"}:
        return actions

    rviz_config = yaml.safe_load(Path(rviz_path.perform(context)).read_text(encoding="utf-8"))
    water_config = yaml.safe_load(
        Path(water_map_path.perform(context)).read_text(encoding="utf-8")
    ) or {}
    satellite = next(
        display for display in rviz_config["Visualization Manager"]["Displays"]
        if display["Name"] == "Tianditu satellite"
    )
    package_share = Path(water_map_path.perform(context)).parent.parent
    configure_satellite(satellite, water_config, package_share)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".rviz", prefix="usv_map_", delete=False
    ) as temporary:
        yaml.safe_dump(rviz_config, temporary, sort_keys=False, allow_unicode=True)
        temporary_path = Path(temporary.name)
    atexit.register(lambda path=temporary_path: path.unlink(missing_ok=True))
    actions.append(Node(
        package="rviz2",
        executable="rviz2",
        name="usv_map_rviz",
        output=LaunchConfiguration("output"),
        arguments=["-d", str(temporary_path)],
    ))
    return actions


def generate_launch_description() -> LaunchDescription:
    """返回海图、里程计、船模及可选 RViz 的启动描述."""
    config_path = PathJoinSubstitution([
        FindPackageShare("usv_map"),
        "config",
        "usv_map.yaml",
    ])
    rviz_path = PathJoinSubstitution([
        FindPackageShare("usv_map"),
        "rviz",
        "usv_map.rviz",
    ])
    water_map_path = PathJoinSubstitution([
        FindPackageShare("usv_map"),
        "config",
        "water_map.yaml",
    ])
    channel_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("channel_navigation_manager"),
            "launch",
            "river_navigation.launch.py",
        ])),
        launch_arguments={
            "simulate_perception": LaunchConfiguration("simulate_perception"),
            "output": LaunchConfiguration("output"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_path",
            default_value="",
            description="海图 XML 路径；非空覆盖 usv_map.yaml，空值沿用 YAML 或包内默认海图",
        ),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="启动 RViz 窗口；false 时地图、里程计和 TF 仍运行"),
        DeclareLaunchArgument("output", default_value="screen",
                              description="节点输出：screen 显示到终端，log 写入 ROS 日志文件"),
        DeclareLaunchArgument("simulate_perception", default_value="true",
                              description="启用模拟感知；仅在 channel_navigation=true 时有效，false 不关闭航道节点"),
        DeclareLaunchArgument(
            "channel_navigation",
            default_value="true",
            description="启用动态航道代价及可选模拟感知；false 关闭整个航道模块，保留静态海图",
        ),
        OpaqueFunction(
            function=launch_chart_nodes,
            args=[
                config_path,
                rviz_path,
                water_map_path,
                channel_navigation_launch,
            ],
        ),
    ])
