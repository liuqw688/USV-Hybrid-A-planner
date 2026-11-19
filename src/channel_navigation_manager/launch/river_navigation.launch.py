"""启动航道代价节点及可替换的仿真感知节点."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """返回航道代价管理及可选仿真感知节点的启动描述."""
    config = os.path.join(get_package_share_directory("channel_navigation_manager"),
                          "config", "channel_navigation.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="screen",
                              description="Node output: screen or log"),
        DeclareLaunchArgument("simulate_perception", default_value="true",
                              description="启动仿真感知；接真实感知时设为 false"),
        Node(package="channel_navigation_manager", executable="channel_navigation_node",
             output=LaunchConfiguration("output"), parameters=[config]),
        Node(package="channel_navigation_manager", executable="obstacle_perception_sim_node",
             output=LaunchConfiguration("output"), parameters=[config],
             condition=IfCondition(LaunchConfiguration("simulate_perception"))),
    ])
