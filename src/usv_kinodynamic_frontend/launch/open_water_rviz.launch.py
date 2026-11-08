"""Only the migrated frontend and RViz in empty open water; no controllers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('usv_kinodynamic_frontend')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(package='usv_kinodynamic_frontend', executable='usv_open_water_node',
             name='usv_open_water_frontend', output='screen',
             parameters=[os.path.join(share, 'config', 'open_water.yaml')]),
        Node(package='rviz2', executable='rviz2', name='usv_frontend_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'open_water.rviz')]),
    ])
