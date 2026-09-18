import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='head_on',
                              description='COLREGs test scenario name'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz with risk/DWA layers'),
        OpaqueFunction(function=launch_scenario),
    ])


# [功能与联系] 给选定演示会遇配置初始船位/目标等启动条件；演示入口绕过全局航点包，Hybrid直接接收/goal_pose。
def launch_scenario(context):
    scenario=LaunchConfiguration('scenario').perform(context)
    scenarios={
        # 本船4m/s、目标2m/s：交叉场景约10秒后在(0,0)发生CPA。
        'head_on': (60.0,0.0,3.141592653589793,2.0),
        'starboard': (0.0,-20.0,1.5707963267948966,2.0),
        # 直航船需要先观察让路船行动，给20秒预测提前量。
        # 原10秒近距突现测试另见测试报告，不能保证保持7.5m冗余。
        'port_cooperative': (0.0,40.0,-1.5707963267948966,2.0),
        'port_takeover': (0.0,40.0,-1.5707963267948966,2.0),
        'overtaking': (-10.0,0.0,0.0,2.0),
        'open_water': (160.0,75.0,0.0,2.0),
    }
    x,y,yaw,speed=scenarios[scenario]
    own_start_x = -80.0 if scenario.startswith('port_') else -40.0
    map_config = os.path.join(
        get_package_share_directory('free_water_map'), 'config', 'free_water.yaml')
    planner_config = os.path.join(
        get_package_share_directory('hybrid_a_star_planner'),
        'config', 'hybrid_a_star_params.yaml')
    dwa_config = os.path.join(
        get_package_share_directory('dwa_deep'), 'config', 'dwa_params.yaml')
    simulator_config = os.path.join(
        get_package_share_directory('dwa_deep'), 'config', 'sim_params.yaml')
    target_config = os.path.join(
        get_package_share_directory('virtual_boat_simulator'),
        'config', 'virtual_boat.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('free_water_map'), 'rviz', 'colregs_v4.rviz')

    return [
        Node(
            package='free_water_map', executable='free_water_costmap.py',
            name='free_water_costmap', output='screen', parameters=[map_config]),
        Node(
            package='dwa_deep', executable='boat_simulator',
            name='boat_simulator', output='screen', parameters=[simulator_config,
                {'static_obstacles_enabled':False,'initial_speed':4.0,
                 'start_x':own_start_x,'start_y':0.0,'start_yaw':0.0}]),
        Node(
            package='virtual_boat_simulator', executable='virtual_boat_node',
            name='virtual_boat_node', output='screen', parameters=[target_config,
                {'initial_x':x,'initial_y':y,'initial_yaw':yaw,'initial_speed':speed}]),
        Node(
            package='hybrid_a_star_planner', executable='hybrid_a_star_node',
            name='hybrid_a_star_node', output='screen', parameters=[planner_config,
                {'channel.enabled':False,'require_target_states':True,
                 'goal_topic':'/goal_pose','global_route_topic':''}]),
        Node(
            package='dwa_deep', executable='dwa_planner',
            name='dwa_planner', output='screen', parameters=[dwa_config,
                {'channel.enabled':False}]),
        Node(
            package='free_water_map', executable='demo_goal_publisher.py',
            name='demo_goal_publisher', output='screen'),
        # A ready-made view exposes risk layers and both DWA trajectories.  The
        # 2D Goal Pose tool publishes /goal_pose and can replace the one-shot goal.
        Node(package='rviz2',executable='rviz2',name='colregs_rviz',output='screen',
             arguments=['-d',rviz_config],condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
