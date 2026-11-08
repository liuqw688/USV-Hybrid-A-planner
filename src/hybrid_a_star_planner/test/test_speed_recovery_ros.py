#!/usr/bin/env python3
"""Isolated real-node test: low-speed U-turn followed by maximum-speed recovery."""
import os
os.environ['ROS_DOMAIN_ID']='184'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_speed_recovery_logs'
import math,time,json,subprocess
import rclpy
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_prefix

def main():
 rclpy.init();node=rclpy.create_node('speed_recovery_fixture')
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 mp=node.create_publisher(OccupancyGrid,'/recovery/map',qos)
 rp=node.create_publisher(Path,'/recovery/route',qos)
 gp=node.create_publisher(PoseStamped,'/recovery/goal',qos)
 op=node.create_publisher(Odometry,'/odom',20)
 paths=[];caps=[]
 node.create_subscription(Path,'/recovery/path',paths.append,qos)
 node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',caps.append,qos)
 prefix=get_package_prefix('hybrid_a_star_planner');log=open('/tmp/usv_speed_recovery_node.log','w')
 proc=subprocess.Popen([prefix+'/lib/hybrid_a_star_planner/hybrid_a_star_node','--ros-args','--params-file',
  prefix+'/share/hybrid_a_star_planner/config/hybrid_a_star_params.yaml','-p','channel.enabled:=false',
  '-p','costmap_topic:=/recovery/map','-p','goal_topic:=/recovery/goal','-p','trajectory_topic:=/recovery/path',
  '-p','global_route_topic:=/recovery/route','-p','speed_sampling.recovery_interval:=1.0'],stdout=log,stderr=log)
 x=y=yaw=0.;u=4.
 def spin():
  m=Odometry();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
  m.pose.pose.position.x=x;m.pose.pose.position.y=y
  m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
  m.twist.twist.linear.x=u;op.publish(m);rclpy.spin_once(node,timeout_sec=.02)
 def receive(timeout,predicate):
  start=time.monotonic()
  while time.monotonic()-start<timeout:
   spin()
   for p in reversed(paths):
    if len(p.poses)<2:continue
    cap=next((c.twist.linear.x for c in reversed(caps) if c.header==p.header),None)
    if cap is not None and predicate(p,cap):return p,cap,time.monotonic()-start
  raise AssertionError('No matching safe trajectory')
 try:
  grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.width=240;grid.info.height=160
  grid.info.resolution=1.;grid.info.origin.position.x=-120.;grid.info.origin.position.y=-80.
  grid.info.origin.orientation.w=1.;grid.data=[0]*38400;mp.publish(grid)
  goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=node.get_clock().now().to_msg()
  goal.pose.position.x=-60.;goal.pose.position.y=10.;goal.pose.orientation.z=1.;goal.pose.orientation.w=0.
  route=Path();route.header=goal.header;route.poses=[goal];rp.publish(route);gp.publish(goal)
  p,cap,delay=receive(8,lambda p,c:c<3.99)
  candidates=[q.pose for q in p.poses if abs(math.atan2(math.sin(2*math.atan2(q.pose.orientation.z,q.pose.orientation.w)-math.pi),
   math.cos(2*math.atan2(q.pose.orientation.z,q.pose.orientation.w)-math.pi)))<.03 and math.hypot(q.pose.position.x+60,q.pose.position.y-10)>25]
  assert candidates,'No settled turn suffix'
  # Simulated measured state after the controller settles on the outbound leg.
  # Keep >25m to the goal so this is cruise recovery rather than terminal braking.
  x=-20.;y=10.;yaw=math.pi;u=cap
  paths.clear();caps.clear()
  upgraded,high,recovery_delay=receive(8,lambda p,c:c>=3.99)
  assert math.hypot(upgraded.poses[0].pose.position.x-x,upgraded.poses[0].pose.position.y-y)<1.5
  import csv
  for name,trajectory in [('recovery_turn',p),('recovery_fast',upgraded)]:
   with open('artifacts/hybrid_speed_update/after/'+name+'.csv','w') as stream:
    writer=csv.writer(stream);writer.writerow(['x','y','yaw'])
    for q in trajectory.poses:
     writer.writerow([q.pose.position.x,q.pose.position.y,2*math.atan2(q.pose.orientation.z,q.pose.orientation.w)])
  result={'turn_reference_speed':cap,'recovered_reference_speed':high,'initial_delivery_seconds':delay,
   'recovery_delivery_seconds':recovery_delay,'recovery_start':[x,y,yaw],'passed':True}
  print(json.dumps(result,indent=2));json.dump(result,open('artifacts/hybrid_speed_update/recovery_ros.json','w'),indent=2)
 finally:
  proc.terminate()
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
