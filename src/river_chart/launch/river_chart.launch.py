"""@brief 启动海图发布节点和 RViz2。

@author susheng
@date 2026-08-28
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description() -> LaunchDescription:
    # 👇 使用安装后的资源路径，而非源码绝对路径
    default_input_path = PathJoinSubstitution([
        FindPackageShare("river_chart"),
        "data",
        "ecdis_data.xml",
    ])

    input_path = DeclareLaunchArgument(
        "input_path",
        default_value=default_input_path,
        description="Path to the GDAL/S-57 XML export",
    )
    frame_id = DeclareLaunchArgument("frame_id", default_value="map")
    origin_longitude = DeclareLaunchArgument("origin_longitude", default_value=".nan")
    origin_latitude = DeclareLaunchArgument("origin_latitude", default_value=".nan")
    # scale = DeclareLaunchArgument("scale", default_value="1.0")
    # line_width = DeclareLaunchArgument("line_width", default_value="3.0")
    # point_size = DeclareLaunchArgument("point_size", default_value="2.0")
    fill_polygons = DeclareLaunchArgument("fill_polygons", default_value="false")
    show_land_boundaries = DeclareLaunchArgument("show_land_boundaries", default_value="false")
    focus_navigation_features = DeclareLaunchArgument(
        "focus_navigation_features",
        default_value="true",
        description="Show only channel, buoy, bridge, and berth features",
    )
    scale = DeclareLaunchArgument("scale", default_value="15.0")
    line_width = DeclareLaunchArgument("line_width", default_value="0.2")
    point_size = DeclareLaunchArgument("point_size", default_value="0.15")
    obstacle_inflation = DeclareLaunchArgument("obstacle_inflation", default_value="1.0")
    channel_corridor_width = DeclareLaunchArgument("channel_corridor_width", default_value="20.0")



    # resolution and cell
    grid_resolution = DeclareLaunchArgument("grid_resolution", default_value="1.0")  # occupancy_costmap resolution
    planning_resolution = DeclareLaunchArgument("planning_resolution", default_value="1.0") # obstacle_map resolution
    planning_max_cells = DeclareLaunchArgument("planning_max_cells", default_value="650_000") # obstacle_map cell
    max_grid_cells = DeclareLaunchArgument("max_grid_cells", default_value="1000_000") # occupancy_costmap cell



    publish_labels = DeclareLaunchArgument("publish_labels", default_value="false")
    publish_occupancy_grid = DeclareLaunchArgument("publish_occupancy_grid", default_value="false") #93
    publish_channel_direction = DeclareLaunchArgument("publish_channel_direction", default_value="false") #93
    channel_outside_cost = DeclareLaunchArgument("channel_outside_cost", default_value="60")
    reload_period = DeclareLaunchArgument("reload_period", default_value="0.0")
    rviz = DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2")

    chart_node = Node(
        package="river_chart",
        executable="river_chart_node",
        name="river_chart_node",
        output="screen",
        parameters=[
            {
                "input_path": LaunchConfiguration("input_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "origin_longitude": LaunchConfiguration("origin_longitude"),
                "origin_latitude": LaunchConfiguration("origin_latitude"),
                "scale": LaunchConfiguration("scale"),
                "line_width": LaunchConfiguration("line_width"),
                "point_size": LaunchConfiguration("point_size"),
                "fill_polygons": LaunchConfiguration("fill_polygons"),
                "show_land_boundaries": LaunchConfiguration("show_land_boundaries"),
                "focus_navigation_features": LaunchConfiguration("focus_navigation_features"),
                "publish_labels": LaunchConfiguration("publish_labels"),
                "publish_occupancy_grid": LaunchConfiguration("publish_occupancy_grid"),
                "grid_resolution": LaunchConfiguration("grid_resolution"),
                "max_grid_cells": LaunchConfiguration("max_grid_cells"),
                "planning_resolution": LaunchConfiguration("planning_resolution"),
                "planning_max_cells": LaunchConfiguration("planning_max_cells"),
                "obstacle_inflation": LaunchConfiguration("obstacle_inflation"),
                "channel_corridor_width": LaunchConfiguration("channel_corridor_width"),
                "publish_channel_direction": LaunchConfiguration("publish_channel_direction"),
                "channel_outside_cost": LaunchConfiguration("channel_outside_cost"),
                "reload_period": LaunchConfiguration("reload_period"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="river_chart_rviz",
        output="screen",
        arguments=[
            "-d",
            # 👇 去掉 "src"，与 setup.py 安装目标 share/river_chart/rviz/ 一致
            PathJoinSubstitution([
                FindPackageShare("river_chart"),
                "rviz",
                "river_chart.rviz",
            ]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
        # ---------------- static tf (map -> odom) ----------------
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )
    # ----------------------------------------------------------


    return LaunchDescription(
        [
            input_path,
            frame_id,
            origin_longitude,
            origin_latitude,
            scale,
            line_width,
            point_size,
            fill_polygons,
            show_land_boundaries,
            focus_navigation_features,
            publish_labels,
            publish_occupancy_grid,
            grid_resolution,
            max_grid_cells,
            planning_resolution,
            planning_max_cells,
            obstacle_inflation,
            channel_corridor_width,
            publish_channel_direction,
            channel_outside_cost,
            reload_period,
            static_tf_map_to_odom,
            rviz,
            chart_node,
            rviz_node,
        ]
    )
