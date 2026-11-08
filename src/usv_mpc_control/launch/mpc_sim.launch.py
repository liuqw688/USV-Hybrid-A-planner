"""MPC与本船仿真独立入口；需另行提供规划轨迹、地图与坐标变换。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('usv_mpc_control')) / 'config'
    return LaunchDescription([
        Node(package='usv_mpc_control', executable='boat_simulator',
             parameters=[str(config / 'sim_params.yaml')], output='screen'),
        Node(package='usv_mpc_control', executable='mpc_controller',
             parameters=[str(config / 'mpc_params.yaml')], output='screen'),
    ])
