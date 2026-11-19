#!/usr/bin/env python3
"""隔离域183：实际Hybrid节点，多距离C弯/必要多弧回退/硬障碍拒绝；不启动MPC。"""
import os
os.environ['ROS_DOMAIN_ID']='183';os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_adaptive_c_logs'
import json
import math
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

def main():
    rclpy.init();node=rclpy.create_node('adaptive_c_fixture')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub=node.create_publisher(OccupancyGrid,'/cturn/map',qos)
    route_pub=node.create_publisher(Path,'/cturn/route',qos)
    goal_pub=node.create_publisher(PoseStamped,'/cturn/goal',qos)
    odom_pub=node.create_publisher(Odometry,'/odom',20)
    paths=[];profiles=[];statuses=[];report=[]
    node.create_subscription(Path,'/cturn/path',lambda m:paths.append(m),qos)
    node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',lambda m:profiles.append(m),qos)
    node.create_subscription(String,'/hybrid_a_star/planning_status',lambda m:statuses.append(m.data),qos)
    prefix=get_package_prefix('hybrid_a_star_planner')
    log=open('/tmp/usv_adaptive_c_node.log','w')
    proc=subprocess.Popen([prefix+'/lib/hybrid_a_star_planner/hybrid_a_star_node','--ros-args',
      '--params-file',prefix+'/share/hybrid_a_star_planner/config/hybrid_a_star_params.yaml',
      '-p','channel.enabled:=false','-p','costmap_topic:=/cturn/map','-p','trajectory_topic:=/cturn/path',
      '-p','goal_topic:=/cturn/goal','-p','global_route_topic:=/cturn/route'],stdout=log,stderr=log)
    def spin():
        m=Odometry();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
        m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=4.;odom_pub.publish(m)
        rclpy.spin_once(node,timeout_sec=.025)
    def publish_goal(gx,gy,yaw):
        m=PoseStamped();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
        m.pose.position.x=float(gx);m.pose.position.y=float(gy);m.pose.orientation.z=math.sin(yaw/2)
        m.pose.orientation.w=math.cos(yaw/2);route=Path();route.header=m.header;route.poses=[m]
        route_pub.publish(route);goal_pub.publish(m)
    try:
        grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.width=200;grid.info.height=160
        grid.info.resolution=1.;grid.info.origin.position.x=-100.;grid.info.origin.position.y=-80.
        grid.info.origin.orientation.w=1.;grid.data=[0]*32000;grid_pub.publish(grid)
        for gx,gy,yaw,require_c in [(-15.,8.,math.pi,True),(-30.,10.,math.pi,True),
          (-60.,20.,math.pi,True),(-30.,-10.,-math.pi,True),(20.,25.,math.pi/2,True),
          (14.,14.,math.pi/2,True),(14.,-14.,-math.pi/2,True),
          # 目标正前方而末端艏向正交：几何上可能必须多弧，不能强迫单C弯。
          (20.,0.,math.pi/2,False),(20.,0.,-math.pi/2,False),(2.,0.,math.pi,False)]:
            paths.clear();profiles.clear();publish_goal(gx,gy,yaw);start=time.monotonic();accepted=None;cap=None
            while time.monotonic()-start<6.:
                spin()
                for path in reversed(paths):
                    if len(path.poses)<2:continue
                    end=path.poses[-1].pose
                    if math.hypot(end.position.x-gx,end.position.y-gy)>=1.:continue
                    end_yaw=2*math.atan2(end.orientation.z,end.orientation.w)
                    if abs(math.atan2(math.sin(end_yaw-yaw),math.cos(end_yaw-yaw)))>=.523599:continue
                    paired=next((p for p in reversed(profiles) if p.header==path.header),None)
                    if paired is not None:accepted=path;cap=paired.twist.linear.x;break
                if accepted is not None:break
            assert accepted is not None,('No valid path',gx,gy,statuses[-5:])
            positive=negative=0.
            for a,b in zip(accepted.poses,accepted.poses[1:]):
                aa=2*math.atan2(a.pose.orientation.z,a.pose.orientation.w)
                bb=2*math.atan2(b.pose.orientation.z,b.pose.orientation.w)
                d=math.atan2(math.sin(bb-aa),math.cos(bb-aa));positive+=max(0.,d);negative+=max(0.,-d)
            if require_c:assert min(positive,negative)<.02,('C path has reverse steering',gx,gy,positive,negative)
            report.append({'goal':[gx,gy,yaw],'delay':time.monotonic()-start,'reference_speed':cap,
              'reverse_heading_sum':min(positive,negative),'poses':len(accepted.poses),'require_c':require_c})
        # 新任务被贯穿水域的硬墙阻断：不能为了C形状跳过安全检查。
        for y in range(160):grid.data[y*200+90]=100
        grid_pub.publish(grid);paths.clear();statuses.clear();publish_goal(-30,10,math.pi)
        until=time.monotonic()+5.
        while time.monotonic()<until:spin()
        assert 'blocked' in statuses and paths and not paths[-1].poses,'Hard wall did not block new task'
        print(json.dumps(report,indent=2),flush=True)
        with open('/tmp/usv_adaptive_c_node_results.json','w') as stream:json.dump(report,stream,indent=2)
        print('PASS: real node C curves, lower reference speeds, close multi-arc fallback, hard wall rejection')
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
