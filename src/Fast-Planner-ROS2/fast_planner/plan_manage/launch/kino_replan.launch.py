from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (mirror ROS1 kino_replan.launch)
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')

    odom_topic = LaunchConfiguration('odom_topic')
    grid_map_topic = LaunchConfiguration('grid_map_topic')
    local_grid_topic = LaunchConfiguration('local_grid_topic')
    planning_frame = LaunchConfiguration('planning_frame')
    shot_trigger_distance = LaunchConfiguration('shot_trigger_distance')
    search_heuristic_weight = LaunchConfiguration('search_heuristic_weight')

    # Camera / depth / cloud topics
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')

    # Intrinsics
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')

    # Dynamics
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')

    # Waypoints / flight mode
    flight_type = LaunchConfiguration('flight_type')
    point_num = LaunchConfiguration('point_num')
    p0x = LaunchConfiguration('point0_x')
    p0y = LaunchConfiguration('point0_y')
    p0z = LaunchConfiguration('point0_z')
    p1x = LaunchConfiguration('point1_x')
    p1y = LaunchConfiguration('point1_y')
    p1z = LaunchConfiguration('point1_z')
    p2x = LaunchConfiguration('point2_x')
    p2y = LaunchConfiguration('point2_y')
    p2z = LaunchConfiguration('point2_z')

    fast_planner = Node(
        package='plan_manage',
        executable='fast_planner_node',
        name='fast_planner_node',
        output='screen',
        remappings=[
            ('/odom_world', odom_topic),
            ('/sdf_map/odom', odom_topic),
            ('/sdf_map/cloud', cloud_topic),
            ('/sdf_map/pose', camera_pose_topic),
            ('/sdf_map/depth', depth_topic),
        ],
        parameters=[{
            # replanning method: kino
            'planner_node/planner': 1,

            # planning fsm
            'fsm/flight_type': flight_type,
            'fsm/thresh_replan': 4.0,  # 距轨迹起点的平面距离，m。
            'fsm/thresh_no_replan': 5.0,  # 距目标的平面距离，m。
            'fsm/thresh_no_replan_yaw': 0.5235987755982988,  # 30°，rad。
            'fsm/arrival_pos_tolerance': 4.0,  # 实际船位到目标，m。
            'fsm/arrival_yaw_tolerance': 0.6981317007977318,  # 实际航向误差，40°。

            'fsm/waypoint_num': point_num,
            'fsm/waypoint0_x': p0x,
            'fsm/waypoint0_y': p0y,
            'fsm/waypoint0_z': p0z,
            'fsm/waypoint1_x': p1x,
            'fsm/waypoint1_y': p1y,
            'fsm/waypoint1_z': p1z,
            'fsm/waypoint2_x': p2x,
            'fsm/waypoint2_y': p2y,
            'fsm/waypoint2_z': p2z,

            # USV global/local grids: metadata comes from OccupancyGrid.
            'sdf_map/use_grid_map': True,
            'sdf_map/grid_topic': grid_map_topic,
            'sdf_map/grid_local_topic': local_grid_topic,
            'sdf_map/grid_local_range': 50.0,  # x/y各 +/-50m，即100m x 100m。
            'sdf_map/grid_distance_cap': 10.0,  # 用户确认的带符号饱和距离。
            'sdf_map/grid_snapshot_period': 5.0,  # 有订阅者时完整快照周期，不是ESDF重算周期。
            'sdf_map/grid_occupied_threshold': 75,
            'sdf_map/grid_unknown_is_occupied': True,
            'sdf_map/grid_publish_esdf': False,
            'sdf_map/grid_visualization_stride': 10,
            'visualization/frame_id': planning_frame,
            # Legacy 3D parameters below are ignored by the static grid backend.
            'sdf_map/resolution': 0.1,
            'sdf_map/map_size_x': map_size_x,
            'sdf_map/map_size_y': map_size_y,
            'sdf_map/map_size_z': map_size_z,
            'sdf_map/local_update_range_x': 5.5,
            'sdf_map/local_update_range_y': 5.5,
            'sdf_map/local_update_range_z': 4.5,
            'sdf_map/obstacles_inflation': 3.0,
            'sdf_map/local_bound_inflate': 0.0,
            'sdf_map/local_map_margin': 50,
            'sdf_map/ground_height': -1.0,
            'sdf_map/cx': cx,
            'sdf_map/cy': cy,
            'sdf_map/fx': fx,
            'sdf_map/fy': fy,
            'sdf_map/use_depth_filter': True,
            'sdf_map/depth_filter_tolerance': 0.15,
            'sdf_map/depth_filter_maxdist': 5.0,
            'sdf_map/depth_filter_mindist': 0.2,
            'sdf_map/depth_filter_margin': 2,
            'sdf_map/k_depth_scaling_factor': 1000.0,
            'sdf_map/skip_pixel': 2,
            'sdf_map/p_hit': 0.65,
            'sdf_map/p_miss': 0.35,
            'sdf_map/p_min': 0.12,
            'sdf_map/p_max': 0.90,
            'sdf_map/p_occ': 0.80,
            'sdf_map/min_ray_length': 0.5,
            'sdf_map/max_ray_length': 4.5,
            'sdf_map/esdf_slice_height': 0.3,
            'sdf_map/visualization_truncate_height': 2.49,
            'sdf_map/virtual_ceil_height': 2.5,
            'sdf_map/show_occ_time': False,
            'sdf_map/show_esdf_time': False,
            'sdf_map/pose_type': 1,
            'sdf_map/frame_id': planning_frame,

            # manager
            'manager/max_vel': max_vel,
            'manager/max_acc': max_acc,
            # Limits for the global pose curve [x, y, phi].
            'manager/max_global_vel': 2.0,  # |vx|, |vy|, m/s
            'manager/max_yaw_rate': 0.3,    # |r|, rad/s
            'manager/max_global_acc': 0.5,  # |ax|, |ay|, m/s^2
            'manager/max_yaw_acc': 0.1,     # |dot_r|, rad/s^2
            'manager/max_jerk': 4.0,
            'manager/dynamic_environment': 0,
            'manager/local_segment_length': 6.0,
            'manager/clearance_threshold': 0.2,
            'manager/control_points_distance': 0.5,
            'manager/use_geometric_path': False,
            'manager/use_kinodynamic_path': True,
            'manager/use_topo_path': False,
            'manager/use_optimization': True,

            # kinodynamic search
            'search/max_tau': 0.6,
            'search/init_max_tau': 2.0,
            'search/rk4_steps': 10,
            'search/max_vel': max_vel,
            'search/max_acc': max_acc,
            'search/max_u': 2.0,  # body-frame surge, m/s
            'search/max_v': 0.5,  # body-frame sway, m/s
            'search/max_r': 0.3,  # yaw rate, rad/s
            # Commands for ordinary USV search expansion.
            'search/min_v_cmd': 0.0,
            'search/max_v_cmd': 2.0,
            'search/max_w_cmd': 0.3,
            # Actual USV speed bounds specified by the user.
            'search/max_ground_speed': 2.0,  # m/s, includes sway contribution
            'search/max_yaw_rate': 0.3,       # rad/s, actual r, not w_cmd
            'search/w_time': 10.0,
            'search/horizon': 50.0,              # local planning range in meters
            'search/goal_pos_tolerance': shot_trigger_distance,  # 射击尝试触发距离，m。
            'search/goal_yaw_tolerance': 0.5235987755982988,  # 射击尝试触发距角度radians: 30 degrees
            # Four-piece USV terminal shooting. Command limits are trial values.
            'search/shot_durations': [24.0, 36.0, 48.0, 64.0, 80.0, 100.0],
            'search/shot_cruise_speed': 1.1,
            'search/shot_integration_step': 0.1,  # seconds, max outer RK4 interval
            'search/shot_collision_step': 0.04,   # seconds
            'search/shot_pos_tolerance': 2.5,    # m, shot 末态位置误差
            'search/shot_yaw_tolerance': 0.40,    # rad，约 22.92°
            'search/shot_u_tolerance': 0.08,      # m/s
            'search/shot_v_tolerance': 0.06,      # m/s
            'search/shot_r_tolerance': 0.04,      # rad/s
            'search/shot_min_v_cmd': 0.0,
            'search/shot_max_v_cmd': 2.0,
            'search/shot_max_w_cmd': 0.3,
            'search/shot_command_scale': 0.85,
            'search/shot_max_u': 2.0,
            'search/shot_max_v': 0.5,
            'search/shot_max_r': 0.3,
            'search/shot_map_height': 0.0,       # temporary map z slice, m
            'search/shot_fd_epsilon': 0.001,
            'search/shot_damping': 0.02,
            'search/shot_time_limit': 4.0,        # seconds per connection attempt
            'search/shot_max_iterations': 16,
            'search/lambda_heu': search_heuristic_weight,
            'search/resolution_astar': 0.25,  # x/y grid resolution in meters
            'search/yaw_bins': 72,             # 360 degrees / 72 = 5 degrees
            'search/time_resolution': 0.8,    # dynamic-search time bin in seconds
            'search/margin': 0.2,
            'search/allocate_num': 100000,
            'search/check_num': 5,

            # optimization
            'optimization/yaw_bound': 0.2617993877991494,  # +/-15 deg relative to initial heading control point
            'optimization/lambda1': 10.0,
            'optimization/yaw_smoothness_weight': 4.0,
            'optimization/lambda2': 5.0,
            'optimization/lambda3': 1e-5,
            'optimization/yaw_rate_weight': 1975.3,
            'optimization/yaw_acc_weight': 625.0,
            'optimization/lambda4': 0.01,
            'optimization/yaw_endpoint_weight': 3.6475626111241605,  # 1 m equals 30 deg 
            'optimization/yaw_guide_weight': 3.6475626111241605,  # 1 m equals 30 deg
            'optimization/yaw_waypoint_weight': 3.6475626111241605,  # 1 m equals 30 deg
            'optimization/lambda7': 100.0,
            'optimization/dist0': 2.0,  # obstacle clearance threshold, m
            'optimization/max_vel': max_vel,
            'optimization/max_acc': max_acc,
            'optimization/algorithm1': 15,
            'optimization/algorithm2': 11,
            'optimization/max_iteration_num1': 2,
            'optimization/max_iteration_num2': 600,
            'optimization/max_iteration_num3': 200,
            'optimization/max_iteration_num4': 200,
            'optimization/max_iteration_time1': 0.0001,
            'optimization/max_iteration_time2': 0.06,
            'optimization/max_iteration_time3': 0.003,
            'optimization/max_iteration_time4': 0.003,
            'optimization/order': 3,

            'bspline/limit_vel': max_vel,
            'bspline/limit_acc': max_acc,
            'bspline/limit_ratio': 1.1,
        }]
    )

    traj_server = Node(
        package='plan_manage',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[
            ('/position_cmd', 'planning/pos_cmd'),
            ('/odom_world', odom_topic),
        ],
        parameters=[{
            'traj_server/time_forward': 1.5,
            'traj_server/frame_id': planning_frame,
        }]
    )

    # Waypoint generator (enable only if package exists in your workspace)
    #waypoint_generator = Node(
    #    package='waypoint_generator',
    #    executable='waypoint_generator',
    #    name='waypoint_generator',
    #    output='screen',
    #    remappings=[
    #        ('odom', odom_topic),
    #        ('goal', '/move_base_simple/goal'),
    #        ('traj_start_trigger', '/traj_start_trigger'),
    #    ],
    #    parameters=[{
    #        'waypoint_type': 'manual-lonely-waypoint',
    #    }],
    #    emulate_tty=True,
    #)

    return LaunchDescription([
        # Declarations with defaults from ROS1 kino_replan.launch
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),

        # Static grid size, resolution and origin are read from the received map.
        DeclareLaunchArgument('grid_map_topic', default_value='/map'),
        DeclareLaunchArgument('local_grid_topic', default_value='/local_map'),
        DeclareLaunchArgument('planning_frame', default_value='map'),
        DeclareLaunchArgument('shot_trigger_distance', default_value='25.0',
                              description='Position distance to try terminal shooting in meters'),
        DeclareLaunchArgument('search_heuristic_weight', default_value='5.0',
                              description='Weighted A-star heuristic factor'),
        DeclareLaunchArgument('odom_topic', default_value='/state_ukf/odom'),

        DeclareLaunchArgument('camera_pose_topic', default_value='/pcl_render_node/camera_pose'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth'),
        DeclareLaunchArgument('cloud_topic', default_value='/pcl_render_node/cloud'),

        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),

        DeclareLaunchArgument('max_vel', default_value='3.0'),
        DeclareLaunchArgument('max_acc', default_value='2.0'),

        DeclareLaunchArgument('flight_type', default_value='1'),
        DeclareLaunchArgument('point_num', default_value='2'),

        DeclareLaunchArgument('point0_x', default_value='19.0'),
        DeclareLaunchArgument('point0_y', default_value='0.0'),
        DeclareLaunchArgument('point0_z', default_value='1.0'),

        DeclareLaunchArgument('point1_x', default_value='-19.0'),
        DeclareLaunchArgument('point1_y', default_value='0.0'),
        DeclareLaunchArgument('point1_z', default_value='1.0'),

        DeclareLaunchArgument('point2_x', default_value='0.0'),
        DeclareLaunchArgument('point2_y', default_value='19.0'),
        DeclareLaunchArgument('point2_z', default_value='1.0'),

        fast_planner,
        traj_server,
        #waypoint_generator,
    ])
