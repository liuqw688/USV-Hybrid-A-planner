#!/usr/bin/env python3
"""Exercise the same pose and goal topics as RViz in an isolated ROS domain."""
import json, math, time
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from plan_manage.msg import Bspline

class Probe(Node):
    def __init__(self):
        super().__init__('obstacle_integration_probe')
        self.start = time.monotonic(); self.maps = []; self.inflated = None; self.splines = []
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.maps.append, q)
        self.create_subscription(OccupancyGrid, '/sdf_map/grid_inflate', self.on_inflated, q)
        self.create_subscription(Bspline, '/planning/bspline', self.splines.append, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
    def on_inflated(self, msg): self.inflated=msg
    def cell(self, msg, x,y):
        if msg is None: return None
        q=msg.info.origin.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        dx=x-msg.info.origin.position.x;dy=y-msg.info.origin.position.y
        ix=math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/msg.info.resolution)
        iy=math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/msg.info.resolution)
        if not (0<=ix<msg.info.width and 0<=iy<msg.info.height): return None
        return msg.data[iy*msg.info.width+ix]
    def obstacle(self,x,y):
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.pose.position.x=x;m.pose.pose.position.y=y;m.pose.pose.orientation.w=1.;self.pose_pub.publish(m)
    def goal(self,x,y):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.w=1.;self.goal_pub.publish(m)

def main():
    rclpy.init(); n=Probe(); sent=False;goal=False;checks={};baseline=0;deadline=time.monotonic()+32
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.05);elapsed=time.monotonic()-n.start
        if elapsed>4 and not sent and n.maps and n.pose_pub.get_subscription_count()>0:
            n.obstacle(25.,0.);n.obstacle(25.,20.);sent=True;baseline=len(n.maps)
        if sent and len(n.maps)>baseline and not goal:
            latest=n.maps[-1]
            checks['first_obstacle_map_100']=n.cell(latest,25,0)==100
            checks['second_obstacle_map_100']=n.cell(latest,25,20)==100
            checks['outside_obstacle_free']=n.cell(latest,25,10)==0
            if elapsed>7 and n.cell(n.inflated,25,0)==100:
                checks['inflated_map_center_100']=True
                checks['map_updates_received']=len(n.maps)-baseline
                n.goal(60.,0.);goal=True
        if goal and n.splines:break
    out={'checks':checks,'sent_obstacles':sent,'sent_goal':goal,'spline':None}
    if n.splines:
        m=n.splines[0];k=np.asarray(m.knots);p=np.asarray([[a.x,a.y,a.z] for a in m.pos_pts]);f=BSpline(k,p,m.order)
        t=np.linspace(k[m.order],k[len(p)],2001);v=f(t);speed=f(t,1);acc=f(t,2);sp=np.linalg.norm(speed[:,:2],axis=1);mask=sp>=.3
        curvature=np.abs(speed[:,0]*acc[:,1]-speed[:,1]*acc[:,0])/np.maximum(sp,.3)**3
        out['spline']={'start':v[0].tolist(),'end':v[-1].tolist(),'length_m':float(np.linalg.norm(np.diff(v[:,:2],axis=0),axis=1).sum()),
          'min_distance_obstacle_center_m':float(np.linalg.norm(v[:,:2]-[25,0],axis=1).min()),
          'raw_occupied_samples':sum(n.cell(n.maps[-1],a[0],a[1])==100 for a in v),
          'inflated_occupied_samples':sum(n.cell(n.inflated,a[0],a[1])==100 for a in v),
          'curvature_max_speed_above_0_3':float(curvature[mask].max()) if mask.any() else None}
    print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
if __name__ == '__main__':main()
