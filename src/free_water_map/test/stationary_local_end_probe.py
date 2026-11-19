#!/usr/bin/env python3
"""Hold odometry fixed outside the arrival radius while a short reference expires."""
import json,time,sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from geometry_msgs.msg import PoseStamped
from plan_manage.msg import Bspline
class Probe(Node):
 def __init__(self, goal_x):
  self.goal_x=goal_x
  super().__init__('stationary_local_end_probe'); self.initial=time.monotonic();self.ids=[]
  qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.map=self.create_publisher(OccupancyGrid,'/map',qos);self.odom=self.create_publisher(Odometry,'/planner/odom',10);self.goal=self.create_publisher(Path,'/waypoint_generator/waypoints',10)
  self.create_subscription(Bspline,'/planning/bspline',lambda m:self.ids.append({'id':m.traj_id,'elapsed':time.monotonic()-self.initial}),10)
 def update(self):
  now=self.get_clock().now().to_msg();m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=now;m.info.resolution=1.;m.info.width=100;m.info.height=100;m.info.origin.position.x=-50.;m.info.origin.position.y=-50.;m.info.origin.orientation.w=1.;m.data=[0]*10000;self.map.publish(m)
  o=Odometry();o.header.frame_id='map';o.header.stamp=now;o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;self.odom.publish(o)
 def send_goal(self):
  m=Path();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();p=PoseStamped();p.header=m.header;p.pose.position.x=self.goal_x;p.pose.orientation.w=1.;m.poses=[p];self.goal.publish(m)
def main():
 rclpy.init();n=Probe(float(sys.argv[1]) if len(sys.argv)>1 else 4.);sent=False;last=0;duration=float(sys.argv[2]) if len(sys.argv)>2 else 45.
 while time.monotonic()-n.initial<duration:
  rclpy.spin_once(n,timeout_sec=.05);t=time.monotonic()-n.initial
  if t-last>.1:n.update();last=t
  if t>8 and not sent and n.count_subscribers('/waypoint_generator/waypoints')>0:n.send_goal();sent=True
 print(json.dumps({'goal':[n.goal_x,0,0],'fixed_odom':[0,0,0],'splines':n.ids},indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
