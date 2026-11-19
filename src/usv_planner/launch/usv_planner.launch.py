"""启动无人船局部规划节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """使用包内 YAML 参数启动规划器，并开放常用接口参数。"""
    params_file = LaunchConfiguration('params_file')
    grid_map_topic = LaunchConfiguration('grid_map_topic')
    local_grid_topic = LaunchConfiguration('local_grid_topic')
    planning_frame = LaunchConfiguration('planning_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    goal_input_mode = LaunchConfiguration('goal_input_mode')
    goal_pose_topic = LaunchConfiguration('goal_pose_topic')
    goal_path_topic = LaunchConfiguration('goal_path_topic')
    bspline_topic = LaunchConfiguration('bspline_topic')
    path_speed_mps = LaunchConfiguration('path_speed_mps')
    shot_trigger_distance = LaunchConfiguration('shot_trigger_distance')
    search_time_limit = LaunchConfiguration('search_time_limit')
    search_horizon = LaunchConfiguration('search_horizon')
    search_resolution = LaunchConfiguration('search_resolution')
    search_heuristic_weight = LaunchConfiguration('search_heuristic_weight')
    replan_distance = LaunchConfiguration('replan_distance')
    safety_replan_distance = LaunchConfiguration('safety_replan_distance')
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')

    planner = Node(
        package='usv_planner',
        executable='usv_planner_node',
        name='usv_planner_node',
        output='screen',
        remappings=[('/sdf_map/odom', odom_topic)],
        parameters=[
            params_file,
            {
                'sdf_map/grid_topic': grid_map_topic,
                'sdf_map/grid_local_topic': local_grid_topic,
                'sdf_map/frame_id': planning_frame,
                'visualization/frame_id': planning_frame,
                'topics/odom': odom_topic,
                'goal/input_mode': goal_input_mode,
                'goal/pose_topic': goal_pose_topic,
                'goal/path_topic': goal_path_topic,
                'goal/path_speed_mps': path_speed_mps,
                'topics/bspline': bspline_topic,
                'search/shot_trigger_distance': shot_trigger_distance,
                'search/time_limit': search_time_limit,
                'search/horizon': search_horizon,
                'sdf_map/grid_local_range': search_horizon,
                'search/resolution_astar': search_resolution,
                'search/lambda_heu': search_heuristic_weight,
                'fsm/thresh_replan': replan_distance,
                'manager/safety_replan_distance': safety_replan_distance,
                'manager/max_vel': max_vel,
                'manager/max_acc': max_acc,
                'search/max_vel': max_vel,
                'search/max_acc': max_acc,
                'optimization/max_vel': max_vel,
                'optimization/max_acc': max_acc,
                'bspline/limit_vel': max_vel,
                'bspline/limit_acc': max_acc,
            },
        ],
    )

    package_config = PathJoinSubstitution([
        FindPackageShare('usv_planner'), 'config', 'usv_planner.yaml'])
    arguments = [
        DeclareLaunchArgument('params_file', default_value=package_config,
                              description='无人船规划器 ROS 参数 YAML 文件'),
        DeclareLaunchArgument('grid_map_topic', default_value='/channel_costmap',
                              description='航道方向代价地图 OccupancyGrid 话题'),
        DeclareLaunchArgument('local_grid_topic', default_value='/local_map',
                              description='局部障碍 OccupancyGrid 话题'),
        DeclareLaunchArgument('planning_frame', default_value='map',
                              description='地图、里程计及目标必须使用的坐标系'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='无人船 nav_msgs/Odometry 话题'),
        DeclareLaunchArgument('goal_input_mode', default_value='1',
                              description='目标输入模式：1 为 PoseStamped，2 为 Path'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose',
                              description='RViz 单个位姿目标话题'),
        DeclareLaunchArgument('goal_path_topic', default_value='/path',
                              description='Path 航点序列目标话题'),
        DeclareLaunchArgument('bspline_topic', default_value='/planning/bspline',
                              description='输出 UsvBspline 轨迹话题'),
        DeclareLaunchArgument('path_speed_mps', default_value='3.0',
                              description='Path 滚动局部目标通过速度，单位 m/s；任务末点为零速'),
        DeclareLaunchArgument('shot_trigger_distance', default_value='30.0',
                              description='终端动力学连接的最大触发距离，单位 m'),
        DeclareLaunchArgument('search_time_limit', default_value='0.4',
                              description='前端搜索时间预算，单位 s'),
        DeclareLaunchArgument('search_horizon', default_value='60.0',
                              description='Path 前视弧长、局部搜索半径及局部栅格裁切半宽；应与感知半径相同，单位 m'),
        DeclareLaunchArgument('search_resolution', default_value='1.0',
                              description='动力学搜索位置离散分辨率，单位 m'),
        DeclareLaunchArgument('search_heuristic_weight', default_value='5.0',
                              description='加权 A* 启发式系数'),
        DeclareLaunchArgument('replan_distance', default_value='8.0',
                              description='常规滚动重规划距离阈值，单位 m'),
        DeclareLaunchArgument('safety_replan_distance', default_value='0.5',
                              description='膨胀障碍边界外的附加安全距离，单位 m'),
        DeclareLaunchArgument('max_vel', default_value='3.0',
                              description='样条各分量速度上限，单位 m/s'),
        DeclareLaunchArgument('max_acc', default_value='2.0',
                              description='样条各分量加速度上限，单位 m/s^2'),
    ]
    return LaunchDescription(arguments + [planner])
