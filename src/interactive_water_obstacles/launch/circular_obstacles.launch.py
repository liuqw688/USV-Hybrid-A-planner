"""Overlay RViz 2D Pose Estimate obstacle circles on a base OccupancyGrid."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='2.0', description='Obstacle radius in meters'),
        DeclareLaunchArgument('base_map_topic', default_value='/free_water/base_map'),
        DeclareLaunchArgument('output_map_topic', default_value='/map'),
        Node(package='interactive_water_obstacles', executable='circular_obstacles.py', output='screen',
             parameters=[{'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
                          'base_map_topic': LaunchConfiguration('base_map_topic'),
                          'output_map_topic': LaunchConfiguration('output_map_topic')}]),
    ])
