"""Recorded A* compatible goals → unchanged Hybrid A* → unchanged MPC/ship model."""
import fcntl
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ins_path_replay.route import load_route

_lock=None


def setup(context):
    global _lock
    lock_path=f'/tmp/ins_path_replay_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID","0")}.lock'
    _lock=open(lock_path,'w')
    try: fcntl.flock(_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError as error: raise RuntimeError('本 DDS 域的数据回放已运行，请先结束旧实例') from error
    def arg(name): return LaunchConfiguration(name).perform(context)
    max_speed=float(arg('max_speed'))
    if not math.isfinite(max_speed) or max_speed<0.9:
        raise ValueError('max_speed must be finite and at least 0.9 m/s for the existing shooting fallback')
    route=load_route(arg('csv_path'),int(arg('point_count')),angle_unit=arg('angle_unit'))
    share=Path(get_package_share_directory('ins_path_replay'))
    control=Path(get_package_share_directory('usv_mpc_control'))/'config'
    hybrid=Path(get_package_share_directory('hybrid_a_star_planner'))/'config/hybrid_a_star_params.yaml'
    return [
        Node(package='ins_path_replay',executable='ins_replay_node.py',name='ins_path_replay',output='screen',
             parameters=[{'csv_path':arg('csv_path'),'point_count':int(arg('point_count')),
                          'angle_unit':arg('angle_unit'),'output_dir':arg('output_dir'),
                          'advance_radius':float(arg('advance_radius')),
                          'final_radius':float(arg('final_radius'))}]),
        Node(package='usv_mpc_control',executable='boat_simulator',name='boat_simulator',output='screen',
             parameters=[str(control/'sim_params.yaml'),{'start_x':0.,'start_y':0.,
                 'start_yaw':route.headings[0],'initial_speed':0.,
                 'max_linear_speed':max_speed}]),
        Node(package='hybrid_a_star_planner',executable='hybrid_a_star_node',name='hybrid_a_star_node',output='screen',
             parameters=[str(hybrid),{'channel.enabled':False,'require_target_states':False,
                 # Apply speed overrides only to this experiment; keep package YAML intact.
                 'cruise_speed':max_speed,'large_turn.speed':min(2.0,max_speed),
                 'close_turn.speed':min(1.2,max_speed),
                 'target_odom_topics':['/ins_replay/no_target/odom'],
                 'global_route_topic':'/astar_channel_waypoints'}]),
        Node(package='usv_mpc_control',executable='mpc_controller',name='mpc_controller',output='screen',
             parameters=[str(control/'mpc_params.yaml'),{'channel_enabled':False,
                 'max_speed':max_speed,'intermediate_goal_speed':min(2.0,max_speed)}]),
        Node(package='rviz2',executable='rviz2',name='ins_replay_rviz',output='screen',
             arguments=['-d',str(share/'rviz/ins_replay.rviz')],condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share=Path(get_package_share_directory('ins_path_replay'))
    return LaunchDescription([
        DeclareLaunchArgument('csv_path',default_value=str(share/'data/data_ins_20261019_153413.csv')),
        DeclareLaunchArgument('point_count',default_value='30'),
        DeclareLaunchArgument('angle_unit',default_value='radians'),
        DeclareLaunchArgument('advance_radius',default_value='3.0'),
        DeclareLaunchArgument('final_radius',default_value='0.6'),
        DeclareLaunchArgument('max_speed',default_value='1.2',
                              description='Hybrid cruise / MPC maximum / simulator surge speed, m/s'),
        DeclareLaunchArgument('output_dir',default_value='/tmp/ins_path_replay'),
        DeclareLaunchArgument('use_rviz',default_value='true'),
        OpaqueFunction(function=setup),
    ])
