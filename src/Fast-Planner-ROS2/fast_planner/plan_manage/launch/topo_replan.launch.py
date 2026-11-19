from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (mirror ROS1 topo_replan.launch)
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')

    odom_topic = LaunchConfiguration('odom_topic')

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
            # replanning method: topology
            'planner_node/planner': 2,

            # planning fsm
            'fsm/flight_type': flight_type,
            'fsm/thresh_replan': 0.5,
            'fsm/thresh_no_replan': 2.0,

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

            # sdf map
            'sdf_map/resolution': 0.1,
            'sdf_map/map_size_x': map_size_x,
            'sdf_map/map_size_y': map_size_y,
            'sdf_map/map_size_z': map_size_z,
            'sdf_map/local_update_range_x': 5.5,
            'sdf_map/local_update_range_y': 5.5,
            'sdf_map/local_update_range_z': 4.5,
            'sdf_map/obstacles_inflation': 0.099,
            'sdf_map/local_bound_inflate': 0.5,
            'sdf_map/local_map_margin': 50,
            'sdf_map/ground_height': -1.0,
            'sdf_map/cx': cx,
            'sdf_map/cy': cy,
            'sdf_map/fx': fx,
            'sdf_map/fy': fy,
            'sdf_map/use_depth_filter': True,
            'sdf_map/depth_filter_tolerance': 0.15,
            'sdf_map/depth_filter_maxdist': 4.5,
            'sdf_map/depth_filter_mindist': 0.2,
            'sdf_map/depth_filter_margin': 2,
            'sdf_map/k_depth_scaling_factor': 1000.0,
            'sdf_map/skip_pixel': 3,
            'sdf_map/p_hit': 0.65,
            'sdf_map/p_miss': 0.35,
            'sdf_map/p_min': 0.12,
            'sdf_map/p_max': 0.90,
            'sdf_map/p_occ': 0.80,
            'sdf_map/min_ray_length': 0.5,
            'sdf_map/max_ray_length': 4.5,
            'sdf_map/esdf_slice_height': 0.3,
            'sdf_map/visualization_truncate_height': 2.5,
            'sdf_map/virtual_ceil_height': 3.0,
            'sdf_map/show_occ_time': False,
            'sdf_map/show_esdf_time': False,
            'sdf_map/pose_type': 1,
            'sdf_map/frame_id': 'world',

            # manager
            'manager/max_vel': max_vel,
            'manager/max_acc': max_acc,
            'manager/max_jerk': 4.0,
            'manager/dynamic_environment': 0,
            'manager/local_segment_length': 7.0,
            'manager/clearance_threshold': 0.2,
            'manager/control_points_distance': 0.3,
            'manager/use_geometric_path': False,
            'manager/use_kinodynamic_path': False,
            'manager/use_topo_path': True,
            'manager/use_optimization': True,

            # topology path finding
            'topo_prm/sample_inflate_x': 1.0,
            'topo_prm/sample_inflate_y': 3.5,
            'topo_prm/sample_inflate_z': 1.0,
            'topo_prm/clearance': 0.3,
            'topo_prm/max_sample_time': 0.005,
            'topo_prm/max_sample_num': 2000,
            'topo_prm/max_raw_path': 300,
            'topo_prm/max_raw_path2': 25,
            'topo_prm/short_cut_num': 1,
            'topo_prm/reserve_num': 6,
            'topo_prm/ratio_to_short': 5.5,
            'topo_prm/parallel_shortcut': True,

            # optimization
            'optimization/lambda1': 10.0,
            'optimization/lambda2': 5.0,
            'optimization/lambda3': 0.0,
            'optimization/lambda4': 0.001,
            'optimization/lambda5': 1.5,
            'optimization/lambda6': 10.0,
            'optimization/lambda7': 20.0,
            'optimization/dist0': 0.4,
            'optimization/dist1': 0.0,
            'optimization/max_vel': max_vel,
            'optimization/max_acc': max_acc,
            'optimization/algorithm1': 15,
            'optimization/algorithm2': 11,
            'optimization/max_iteration_num1': 2,
            'optimization/max_iteration_num2': 300,
            'optimization/max_iteration_time1': 0.0001,
            'optimization/max_iteration_time2': 0.005,
            'optimization/order': 3,

            'bspline/limit_vel': max_vel,
            'bspline/limit_acc': max_acc,
            'bspline/limit_ratio': 1.1,

            'heading_planner/yaw_diff': 5 * 3.1415926 / 180.0,
            'heading_planner/half_vert_num': 3,
            'heading_planner/lambda1': 2.0,
            'heading_planner/lambda2': 1.0,
            'heading_planner/max_yaw_rate': 60 * 3.1415926 / 180.0,
            'heading_planner/w': 10.0,
            'fsm/act_map': False,
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
        # Declarations with defaults from ROS1 topo_replan.launch
        DeclareLaunchArgument('map_size_x', default_value='40.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),

        DeclareLaunchArgument('odom_topic', default_value='/state_ukf/odom'),

        DeclareLaunchArgument('camera_pose_topic', default_value='/pcl_render_node/camera_pose'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth'),
        DeclareLaunchArgument('cloud_topic', default_value='/pcl_render_node/cloud'),

        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),

        DeclareLaunchArgument('max_vel', default_value='3.0'),
        DeclareLaunchArgument('max_acc', default_value='2.5'),

        DeclareLaunchArgument('flight_type', default_value='2'),
        DeclareLaunchArgument('point_num', default_value='3'),

        DeclareLaunchArgument('point0_x', default_value='19.0'),
        DeclareLaunchArgument('point0_y', default_value='0.0'),
        DeclareLaunchArgument('point0_z', default_value='1.0'),

        DeclareLaunchArgument('point1_x', default_value='19.0'),
        DeclareLaunchArgument('point1_y', default_value='10.0'),
        DeclareLaunchArgument('point1_z', default_value='1.0'),

        DeclareLaunchArgument('point2_x', default_value='-19.0'),
        DeclareLaunchArgument('point2_y', default_value='0.0'),
        DeclareLaunchArgument('point2_z', default_value='1.0'),

        fast_planner,
        traj_server,
        #waypoint_generator,
    ])
