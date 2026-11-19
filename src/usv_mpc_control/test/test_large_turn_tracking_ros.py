#!/usr/bin/env python3
"""隔离域178：真实Hybrid+MPC节点，180度目标，六状态船模闭环。"""
import os
os.environ.setdefault('ROS_DOMAIN_ID','178')
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_large_turn_logs'
import json
import argparse
import math
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped,Wrench,TwistStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from rclpy.qos import QoSProfile,DurabilityPolicy
from test_mpc_terminal_ros import step

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--narrow',action='store_true')
    parser.add_argument('--close',action='store_true',help='2m前方目标、180度目标艏向、4m/s初始速度')
    parser.add_argument('--close-distance',type=float,default=2.,help='--close的前方目标距离(m)')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('large_turn_fixture')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub=node.create_publisher(OccupancyGrid,'/large_turn/map',qos)
    route_pub=node.create_publisher(Path,'/large_turn/route',qos)
    goal_pub=node.create_publisher(PoseStamped,'/large_turn/goal',qos)
    odom_pub=node.create_publisher(Odometry,'/odom',20)
    state=[0.,0.,0.,4.,0.,0.];command=[0.,0.];paths=[];caps=[];errors=[]
    def on_command(m): command[:]=[m.force.x,m.torque.z]
    node.create_subscription(Wrench,'/large_turn/wrench',on_command,20)
    node.create_subscription(Path,'/large_turn/path',lambda m: paths.append(m),qos)
    node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',lambda m: caps.append(m.twist.linear.x),qos)
    hp=get_package_prefix('hybrid_a_star_planner');mp=get_package_prefix('usv_mpc_control')
    label=('_close' if args.close_distance==2. else '_close_'+str(args.close_distance)) if args.close else ('_narrow' if args.narrow else '')
    logs=[open('/tmp/usv_large_turn_hybrid'+label+'.log','w'),open('/tmp/usv_large_turn_mpc'+label+'.log','w')]
    processes=[]
    try:
        processes.append(subprocess.Popen([hp+'/lib/hybrid_a_star_planner/hybrid_a_star_node','--ros-args',
          '--params-file',hp+'/share/hybrid_a_star_planner/config/hybrid_a_star_params.yaml',
          '-p','channel.enabled:=false','-p','costmap_topic:=/large_turn/map',
          '-p','trajectory_topic:=/large_turn/path','-p','goal_topic:=/large_turn/goal',
          '-p','global_route_topic:=/large_turn/route'],stdout=logs[0],stderr=logs[0]))
        processes.append(subprocess.Popen([mp+'/lib/usv_mpc_control/mpc_controller','--ros-args',
          '--params-file',mp+'/share/usv_mpc_control/config/mpc_params.yaml',
          '-p','channel_enabled:=false','-p','path_topic:=/large_turn/path',
          '-p','costmap_topic:=/large_turn/map','-p','global_route_topic:=/large_turn/route',
          '-p','active_goal_topic:=/large_turn/goal','-p','effort_topic:=/large_turn/wrench'],
          stdout=logs[1],stderr=logs[1]))
        grid=OccupancyGrid();grid.header.frame_id='odom';grid.header.stamp=node.get_clock().now().to_msg()
        grid.info.width=140;grid.info.height=120;grid.info.resolution=1.
        grid.info.origin.position.x=-70.;grid.info.origin.position.y=-60.;grid.info.origin.orientation.w=1.
        if args.narrow: grid.info.height=28;grid.info.origin.position.y=-14.
        grid.data=[0]*(grid.info.width*grid.info.height);grid_pub.publish(grid)
        goal=PoseStamped();goal.header=grid.header
        gx,gy=(args.close_distance,0.) if args.close else (-30.,10.)
        goal.pose.position.x=gx;goal.pose.position.y=gy
        # ROS2 Quaternion默认w=1；只写z=1实际表示90度，180度必须显式w=0。
        goal.pose.orientation.z=1.;goal.pose.orientation.w=0.
        route=Path();route.header=grid.header;route.poses=[goal]
        route_pub.publish(route);goal_pub.publish(goal)
        start=last=time.monotonic();tracking=None;next_report=10.;settling_until=None
        while time.monotonic()-start<75.:
            now=time.monotonic();dt=min(.06,now-last);last=now
            if any(len(p.poses)>1 for p in paths):
                if tracking is None: tracking=now
                state=step(state,command,dt)
                latest=next((p for p in reversed(paths) if len(p.poses)>1),None)
                errors.append(min(math.hypot(state[0]-p.pose.position.x,state[1]-p.pose.position.y)
                    for p in latest.poses))
            odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.x,odom.pose.pose.position.y=state[:2]
            odom.pose.pose.orientation.z=math.sin(state[2]/2);odom.pose.pose.orientation.w=math.cos(state[2]/2)
            odom.twist.twist.linear.x,odom.twist.twist.linear.y=state[3:5];odom.twist.twist.angular.z=state[5]
            odom_pub.publish(odom);rclpy.spin_once(node,timeout_sec=.02)
            elapsed=now-start
            if elapsed>next_report:
                print('progress',round(elapsed,1),'state',[round(x,2) for x in state],flush=True);next_report+=10.
            if tracking and now-tracking>15 and math.hypot(state[0]-gx,state[1]-gy)<1.05 and math.hypot(state[3],state[4])<.1:
                if settling_until is None: settling_until=now+5.
                if now>=settling_until: break
            else: settling_until=None
        result={'first_path_delay':None if tracking is None else tracking-start,'state':state,
          'goal_distance':math.hypot(state[0]-gx,state[1]-gy),'max_cross_track':max(errors or [999]),
          'speed_limits':sorted(set(caps)),'duration':time.monotonic()-start}
        valid_paths=[p for p in paths if len(p.poses)>1]
        if valid_paths:
            end=valid_paths[0].poses[-1].pose
            result['planned_end_distance']=math.hypot(end.position.x-gx,end.position.y-gy)
            yaw=2.*math.atan2(end.orientation.z,end.orientation.w)
            result['planned_end_heading_error']=abs(math.atan2(math.sin(yaw-math.pi),math.cos(yaw-math.pi)))
        print(json.dumps(result),flush=True)
        with open('/tmp/usv_large_turn_tracking'+label+'.json','w') as report: json.dump(result,report)
        assert tracking is not None and tracking-start<5.,'No timely large-angle path'
        assert result['planned_end_distance']<1. and result['planned_end_heading_error']<.22,'Planner endpoint pose mismatch'
        assert caps and set(caps).issubset({1.2,2.,4.}),'Planning speed profile was not delivered to MPC'
        if args.close: assert any(c<=2. for c in caps),'Close goal did not exercise low-speed fallback'
        if args.narrow: assert 2. in caps,'Narrow water did not exercise low-speed fallback'
        assert result['goal_distance']<1.05,'Large-angle goal not reached within Hybrid 1m endpoint tolerance'
        assert result['max_cross_track']<3.,'Excessive tracking error'
        assert math.hypot(state[3],state[4])<.15,'Final goal did not stop'
    finally:
        for proc in processes:
            proc.terminate()
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill();proc.wait()
        for log in logs: log.close()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__': main()
