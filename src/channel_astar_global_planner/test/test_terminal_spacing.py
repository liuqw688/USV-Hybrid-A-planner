"""真实全局A*节点：末端距离边界、目标切换和必经点保护。"""
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import QoSProfile, DurabilityPolicy


def _run(tmp_path, via=False):
    package='channel_astar_global_planner'
    command=[get_package_prefix(package)+'/lib/'+package+'/channel_astar_node','--ros-args',
        '--params-file',get_package_share_directory(package)+'/config/channel_astar.yaml']
    if via:command+=['-p','via_points:=[150.5,10.5]']
    rclpy.init();node=rclpy.create_node('terminal_spacing_probe')
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    routes=[];active=[]
    node.create_subscription(Path,'/astar_channel_waypoints',routes.append,qos)
    node.create_subscription(PoseStamped,'/goal_pose_from_astar',active.append,qos)
    maps=node.create_publisher(OccupancyGrid,'/channel/lane_costmap',qos)
    safety=node.create_publisher(OccupancyGrid,'/local_costmap',qos)
    odoms=node.create_publisher(Odometry,'/odom',10)
    goals=node.create_publisher(PoseStamped,'/goal_pose',10)
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.width=400;grid.info.height=60
    grid.info.resolution=1.;grid.info.origin.orientation.w=1.;grid.data=[0]*24000
    own=Odometry();own.header.frame_id='odom';own.pose.pose.orientation.w=1.
    own.pose.pose.position.x=.5;own.pose.pose.position.y=10.5
    with open(tmp_path/'node.log','w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=log)
        def spin(seconds,predicate=None):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                assert child.poll() is None,(tmp_path/'node.log').read_text()
                grid.header.stamp=node.get_clock().now().to_msg();own.header.stamp=grid.header.stamp
                maps.publish(grid);safety.publish(grid);odoms.publish(own)
                rclpy.spin_once(node,timeout_sec=.02)
                if predicate and predicate():return
            if predicate:assert predicate(),(tmp_path/'node.log').read_text()
        try:
            spin(6,lambda:goals.get_subscription_count()>0)
            cases=[(160.5,[.5,100.5,150.5,160.5])] if via else [
                (110.5,[.5,110.5]),(119.5,[.5,119.5]),
                (120.5,[.5,100.5,120.5]),(121.5,[.5,100.5,121.5]),
                (210.5,[.5,100.5,210.5])]
            for x,expected in cases:
                routes.clear();active.clear()
                goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=node.get_clock().now().to_msg()
                goal.pose.position.x=x;goal.pose.position.y=10.5;goal.pose.orientation.w=1.
                goals.publish(goal)
                spin(6,lambda:any(p.poses and abs(p.poses[-1].pose.position.x-x)<1e-6 for p in routes))
                route=next(p for p in routes if p.poses and abs(p.poses[-1].pose.position.x-x)<1e-6)
                assert [p.pose.position.x for p in route.poses]==expected
                assert route.poses[-1].pose.orientation.w==1,'Final direction altered'
            if not via:
                spin(2,lambda:active and active[-1].pose.position.x==100.5)
                own.pose.pose.position.x=100.5
                spin(3,lambda:active[-1].pose.position.x==210.5)
                assert all(p.pose.position.x!=200.5 for p in active),'Deleted waypoint dispatched'
        finally:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            node.destroy_node();rclpy.shutdown()


def test_terminal_spacing_and_dispatch(tmp_path):_run(tmp_path)
def test_mandatory_via_is_not_removed(tmp_path):_run(tmp_path,via=True)
