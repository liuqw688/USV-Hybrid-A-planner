#!/usr/bin/env python3
"""Read-only contract checks against a running replay in the selected DDS domain."""
import json
import math
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String


def main():
    rclpy.init(); node=rclpy.create_node('ins_replay_interface_check')
    state={}; route_epochs=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def route(message):
        state['route']=message
        route_epochs.append((message.header.stamp.sec,message.header.stamp.nanosec))
    node.create_subscription(Path,'/astar_channel_waypoints',route,qos)
    node.create_subscription(OccupancyGrid,'/local_costmap',lambda m:state.update(map=m),qos)
    node.create_subscription(PoseStamped,'/goal_pose_from_astar',lambda m:state.update(goal=m),qos)
    node.create_subscription(Path,'/hybrid_a_star_trajectory',lambda m:state.update(path=m),qos)
    node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',lambda m:state.update(cap=m),qos)
    node.create_subscription(String,'/ins_replay/status',lambda m:state.update(status=json.loads(m.data)),qos)
    deadline=time.monotonic()+8
    paired=False
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if all(k in state for k in ('route','map','goal','path','cap','status')):
                paired=paired or state['path'].header==state['cap'].header
        assert set(state)=={'route','map','goal','path','cap','status'},state.keys()
        route=state['route']; goal=state['goal']; grid=state['map']
        assert len(route.poses)==30
        assert route.poses[0].pose.position.x==route.poses[0].pose.position.y==0.
        assert len(set(route_epochs))==1 and len(route_epochs)>=2,'Map heartbeat must not change route epoch'
        assert goal.header==route.header
        assert all(p.header==route.header for p in route.poses)
        assert all(getattr(grid.header,'frame_id')==p.header.frame_id=='odom' for p in route.poses)
        assert grid.data and set(grid.data)=={0},'Open-water map must contain only free cells'
        assert any(math.dist((goal.pose.position.x,goal.pose.position.y),
                            (p.pose.position.x,p.pose.position.y))<1e-8 for p in route.poses)
        for a,b in zip(route.poses,route.poses[1:]):
            expected=math.atan2(b.pose.position.y-a.pose.position.y,b.pose.position.x-a.pose.position.x)
            yaw=2*math.atan2(a.pose.orientation.z,a.pose.orientation.w)
            assert abs(math.atan2(math.sin(yaw-expected),math.cos(yaw-expected)))<1e-8
        assert paired,'Hybrid Path and speed-limit timestamps must pair'
        assert state['path'].poses and state['path'].header.frame_id=='odom'
        print(json.dumps({'passed':True,'route_points':30,'stable_route_epoch_messages':len(route_epochs),
                          'paired_path_and_speed':paired,'all_free_grid_cells':len(grid.data),
                          'active_index':state['status']['active_index']},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
