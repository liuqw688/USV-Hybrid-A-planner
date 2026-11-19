"""Open-water integration: Fast-Planner, MPC and six-state boat simulation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_config = Path(get_package_share_directory('free_water_map')) / 'rviz' / 'fast_planner_open_water.rviz'
    planner = Path(get_package_share_directory('plan_manage')) / 'launch' / 'kino_replan.launch.py'
    mpc_config = Path(get_package_share_directory('usv_mpc_control')) / 'config' / 'mpc_params.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('trajectory_metrics_file', default_value='~/.ros/usv_trajectory_metrics.jsonl'),
        DeclareLaunchArgument('obstacle_radius', default_value='2.0', description='Interactive obstacle radius in meters'),
        DeclareLaunchArgument('shot_trigger_distance', default_value='25.0'),
        DeclareLaunchArgument('search_heuristic_weight', default_value='2.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(planner)),
                                 launch_arguments={'odom_topic': '/planner/odom',
                                                   'shot_trigger_distance': LaunchConfiguration('shot_trigger_distance'),
                                                   'search_heuristic_weight': LaunchConfiguration('search_heuristic_weight')}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='free_water_map', executable='free_water_costmap.py',
             parameters=[{'topic': '/free_water/base_map', 'frame_id': 'map', 'resolution': 1.0,
                          'width': 290, 'height': 180, 'origin_x': -120.0, 'origin_y': -90.0}]),
        Node(package='interactive_water_obstacles', executable='circular_obstacles.py', output='screen',
             parameters=[{'radius': ParameterValue(LaunchConfiguration('obstacle_radius'), value_type=float)}]),
        Node(package='free_water_map', executable='fast_planner_bridge.py', output='screen'),
        Node(package='free_water_map', executable='trajectory_quality_monitor.py', output='screen',
             parameters=[{'output_file': LaunchConfiguration('trajectory_metrics_file')}]),
        Node(package='usv_mpc_control', executable='boat_simulator', output='screen',
             parameters=[{'start_x': 0.0, 'start_y': 0.0, 'start_yaw': 0.0,
                          'initial_speed': 0.0, 'max_linear_speed': 2.0, 'max_angular_speed': 0.3}]),
        Node(package='usv_mpc_control', executable='mpc_controller', output='screen',
             parameters=[str(mpc_config), {'path_topic': '/fast_planner/path',
                 'costmap_topic': '/map', 'control_frame': 'odom', 'channel_enabled': False,
                 'global_route_topic': '', 'active_goal_topic': '/fast_planner/goal',
                 'max_speed': 2.0, 'max_yaw_rate': 0.3, 'require_speed_profile': False,
                 'stop_at_path_end': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz_config)], condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
