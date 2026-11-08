"""Identified-model frontend/MPC/plant on open water or the existing S-57 channel map."""
import os
import fcntl
import atexit
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

_lock = None


def start(context):
    # One shared lock per ROS domain prevents duplicate instances of this launch.
    global _lock
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    _lock = open('/tmp/usv_frontend_tracking_domain_' + domain + '.lock', 'a+')
    try:
        fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('An open-water tracking launch already owns this ROS domain') from exc
    _lock.seek(0)
    _lock.truncate()
    _lock.write(str(os.getpid()) + '\n')
    _lock.flush()
    atexit.register(_lock.close)
    own = get_package_share_directory('usv_kinodynamic_frontend')
    control = get_package_share_directory('usv_mpc_control')
    environment = LaunchConfiguration('environment').perform(context)
    if environment not in ('open_water', 'channel'):
        raise ValueError('environment must be open_water or channel')
    channel = environment == 'channel'
    def number(name, default):
        raw = LaunchConfiguration(name).perform(context)
        return float(raw) if raw else default
    start_x = number('start_x', -800.0 if channel else 0.0)
    start_y = number('start_y', -840.0 if channel else 0.0)
    start_yaw = number('start_yaw', 2.14675498 if channel else 0.0)
    goal_x = number('goal_x', -835.0 if channel else 40.0)
    goal_y = number('goal_y', -785.0 if channel else 10.0)
    goal_yaw = number('goal_yaw', 2.14675498 if channel else 0.25)
    with open(os.path.join(control, 'config', 'mpc_params.yaml'), encoding='utf-8') as stream:
        runtime_parameters = yaml.safe_load(stream)['mpc_controller']['ros__parameters']
    shared_model = {name: value for name, value in runtime_parameters.items() if name.startswith('model.')}
    mode = LaunchConfiguration('dynamics_model').perform(context)
    if mode not in ('force', 'identified_velocity_commands'):
        raise ValueError('unknown dynamics_model')
    # All three actors select identical state equations and command bounds.
    shared_model.update({'dynamics_model': mode, 'command_topic': '/cmd_vel'})
    if mode == 'identified_velocity_commands':
        shared_model.update({'identified.min_v_cmd': -0.4, 'identified.max_v_cmd': 1.5,
                             'identified.max_w_cmd': 0.7})
    speed = float(LaunchConfiguration('tracking_speed').perform(context))
    auto_goal = LaunchConfiguration('auto_goal').perform(context).lower() == 'true'
    if channel:
        chart = get_package_share_directory('river_chart')
        local = get_package_share_directory('local_costmap_generator')
        map_actions = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(chart, 'launch', 'river_chart.launch.py')),
                launch_arguments={'rviz': 'false', 'frame_id': 'odom',
                                  'show_land_boundaries': 'true',
                                  'focus_navigation_features': 'false'}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(local, 'launch', 'local_costmap.launch.py'))),
        ]
    else:
        water = get_package_share_directory('free_water_map')
        map_actions = [Node(package='free_water_map', executable='free_water_costmap.py',
            name='free_water_costmap', output='screen',
            parameters=[os.path.join(water, 'config', 'free_water.yaml')])]
    return map_actions + [
        Node(package='usv_mpc_control', executable='boat_simulator', name='boat_simulator', output='screen',
             parameters=[os.path.join(control, 'config', 'sim_params.yaml'), {
                 'start_x': start_x, 'start_y': start_y, 'start_yaw': start_yaw, 'initial_speed': 0.0,
                 'max_linear_speed': 1.6 if mode == 'identified_velocity_commands' else 4.0}, shared_model]),
        Node(package='usv_mpc_control', executable='mpc_controller', name='mpc_controller', output='screen',
             parameters=[os.path.join(control, 'config', 'mpc_params.yaml'), {
                 'channel_enabled': False, 'global_route_topic': '',
                 'active_goal_topic': '/usv_frontend/active_goal',
                 'intermediate_goal_speed': 0.5 if mode == 'identified_velocity_commands' else 0.7,
                 'max_speed': 1.6 if mode == 'identified_velocity_commands' else 4.0,
                 'max_yaw_rate': 0.8 if mode == 'identified_velocity_commands' else 0.45,
             }, shared_model]),
        Node(package='usv_kinodynamic_frontend', executable='usv_tracking_node', name='usv_tracking_frontend',
             output='screen', parameters=[{'goal.x': goal_x, 'goal.y': goal_y, 'goal.yaw': goal_yaw,
                                          'auto_goal': auto_goal, 'tracking_speed': speed,
                                          'search.max_search_time': float(LaunchConfiguration('max_search_time').perform(context)),
                                          'search.horizon': float(LaunchConfiguration('search_horizon').perform(context)),
                                          'search.max_nodes': int(LaunchConfiguration('max_nodes').perform(context)),
                                          'search.output_step': float(LaunchConfiguration('output_step').perform(context)),
                                          # This chart is latched/static; odometry freshness remains enforced.
                                          'map_timeout': 0.0 if channel else 3.0,
                                          'search.collision_radius': 1.5 if channel else 0.7,
                                          'search.shore_cost_weight': 1.0 if channel else 0.0,
                                          'display.water_plane': not channel,
                                          'display.origin_x': start_x if channel else 0.0,
                                          'display.origin_y': start_y if channel else 0.0,
                                          'display.text_height': 1.8 if channel else 1.0}, shared_model]),
        Node(package='rviz2', executable='rviz2', name='usv_tracking_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', os.path.join(own, 'rviz', 'channel_tracking.rviz' if channel else 'open_water_tracking.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('environment', default_value='open_water'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument('dynamics_model', default_value='identified_velocity_commands'),
        DeclareLaunchArgument('tracking_speed', default_value='1.0'),
        DeclareLaunchArgument('max_search_time', default_value='8.0'),
        DeclareLaunchArgument('search_horizon', default_value='500.0'),
        DeclareLaunchArgument('max_nodes', default_value='200000'),
        DeclareLaunchArgument('output_step', default_value='0.1'),
        # Empty coordinates select environment-specific defaults.
        DeclareLaunchArgument('start_x', default_value=''),
        DeclareLaunchArgument('start_y', default_value=''),
        DeclareLaunchArgument('start_yaw', default_value=''),
        DeclareLaunchArgument('goal_x', default_value=''),
        DeclareLaunchArgument('goal_y', default_value=''),
        DeclareLaunchArgument('goal_yaw', default_value=''),
        OpaqueFunction(function=start),
    ])
