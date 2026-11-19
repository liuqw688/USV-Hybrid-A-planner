#!/usr/bin/env python3
"""Sample each published B-spline and record comparable diagnostic metrics."""
import json
import math
import os
import time
import uuid

import numpy as np
from scipy.interpolate import BSpline
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from plan_manage.msg import Bspline


class TrajectoryQualityMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_quality_monitor')
        self.declare_parameter('output_file', '~/.ros/usv_trajectory_metrics.jsonl')
        self.declare_parameter('samples', 1001)
        self.declare_parameter('speed_floor', 0.3)
        self.declare_parameter('clearance_threshold', 3.0)
        self.output_file = os.path.expanduser(self.get_parameter('output_file').value)
        self.samples = max(10, int(self.get_parameter('samples').value))
        self.speed_floor = float(self.get_parameter('speed_floor').value)
        self.clearance_threshold = float(self.get_parameter('clearance_threshold').value)
        if not math.isfinite(self.speed_floor) or self.speed_floor <= 0 or not math.isfinite(self.clearance_threshold) or self.clearance_threshold <= 0:
            raise ValueError('speed_floor and clearance_threshold must be positive')
        self.run_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8]
        self.map = None; self.clearance = None; self.map_key = None
        self.odom = None; self.goal = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(Odometry, '/planner/odom', self.on_odom, 20)
        self.create_subscription(PoseStamped, '/fast_planner/goal', self.on_goal, qos)
        self.create_subscription(Bspline, '/planning/bspline', self.on_spline, 10)
        self.pub = self.create_publisher(String, '/fast_planner/trajectory_quality', 10)
        self.get_logger().info(f'Trajectory metrics run={self.run_id}, file={self.output_file}')

    def on_odom(self, msg): self.odom = msg
    def on_goal(self, msg): self.goal = msg
    def on_map(self, msg):
        info=msg.info
        if not info.width or not info.height or len(msg.data)!=info.width*info.height or not math.isfinite(info.resolution) or info.resolution<=0:
            return
        cells=np.asarray(msg.data,dtype=np.int8)
        q=info.origin.orientation
        key=(msg.header.frame_id, info.width,info.height,info.resolution,info.origin.position.x,info.origin.position.y,q.x,q.y,q.z,q.w,hash(cells.tobytes()))
        if key!=self.map_key:
            free=((cells>=0)&(cells<75)).reshape(info.height,info.width)
            # Treat unknown cells and outside of the map as blocked.
            self.clearance=distance_transform_edt(np.pad(free,1,constant_values=False),sampling=info.resolution)[1:-1,1:-1]
            self.map_key=key
        self.map=msg

    def sample_clearance(self, xy):
        if self.map is None or self.clearance is None or self.map.header.frame_id != 'map': return None, None
        info=self.map.info;q=info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        dx=xy[:,0]-info.origin.position.x;dy=xy[:,1]-info.origin.position.y
        ix=np.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution).astype(int)
        iy=np.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution).astype(int)
        inside=(ix>=0)&(iy>=0)&(ix<info.width)&(iy<info.height)
        dist=np.zeros(len(xy));dist[inside]=self.clearance[iy[inside],ix[inside]]
        # Center-cell EDT minus one cell diagonal gives a conservative grid approximation.
        dist=np.maximum(0,dist-math.sqrt(2)*info.resolution)
        return float(dist.min()), int((~inside).sum())

    def on_spline(self, msg):
        try:
            p=np.asarray([[a.x,a.y,a.z] for a in msg.pos_pts],dtype=float);k=np.asarray(msg.knots,dtype=float);degree=int(msg.order)
            if degree<3 or len(p)<=degree or len(k)!=len(p)+degree+1 or not np.isfinite(p).all() or not np.isfinite(k).all() or np.any(np.diff(k)<0) or k[len(p)]<=k[degree]:
                raise ValueError('invalid B-spline data')
            f=BSpline(k,p,degree);t=np.linspace(k[degree],k[len(p)],self.samples)
            x,v,a,j=(f(t,i) for i in range(4));speed=np.hypot(v[:,0],v[:,1]);valid=speed>=self.speed_floor
            length=float(np.trapz(speed,t));chord=float(np.linalg.norm(x[-1,:2]-x[0,:2]))
            curvature=np.abs(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(speed,self.speed_floor)**3
            valid_length=float(np.trapz(speed*valid,t))
            krms=float(np.sqrt(np.trapz(curvature**2*speed*valid,t)/valid_length)) if valid_length>1e-9 else None
            kmax=float(curvature[valid].max()) if valid.any() else None
            yaw=x[:,2];r=v[:,2];c=np.cos(yaw);s=np.sin(yaw)
            u=c*v[:,0]+s*v[:,1];sway=-s*v[:,0]+c*v[:,1]
            du=c*a[:,0]+s*a[:,1]+r*sway;dv=-s*a[:,0]+c*a[:,1]-r*u
            vcmd=(du+.433264281*u-.032263988)/.400421432
            wcmd=(a[:,2]-.166118168*sway+.544509453*r-.000626529)/.837536662
            dv_model=-.313324916*sway-.006741061*r-.407958500*wcmd-.000993619
            jerk=float(np.sqrt(np.mean(j[:,2]**2)));clearance,outside=self.sample_clearance(x[:,:2])
            vmax=float(vcmd.max());vmin=float(vcmd.min());wmax=float(np.abs(wcmd).max());rmax=float(np.abs(r).max());accmax=float(np.abs(a[:,2]).max())
            sway_residual=float(np.sqrt(np.mean((dv-dv_model)**2)))
            warnings=[]
            if krms is None:warnings.append('curvature_unavailable_low_speed')
            if clearance is None:warnings.append('map_unavailable')
            elif clearance<self.clearance_threshold:warnings.append('sampled_clearance_below_3m')
            if outside:warnings.append('outside_map')
            if vmin < -1e-6 or vmax > 2.0+1e-6:warnings.append('estimated_v_cmd_out_of_range')
            if wmax > .3+1e-6:warnings.append('estimated_w_cmd_out_of_range')
            if rmax > .3+1e-6:warnings.append('yaw_rate_over_0.3')
            if accmax > .1+1e-6:warnings.append('yaw_acc_over_0.1')
            if sway_residual > .05:warnings.append('sway_model_residual_rms_over_0.05')
            # These engineering scales are fixed for comparisons, not certification.
            score=None;parts=None
            if clearance is not None and krms is not None and length>1e-9:
                parts={'length':25*min(1.0,chord/length), 'curvature':30/(1+krms/.1),
                       'yaw_jerk':25/(1+jerk/.1), 'clearance':20*min(1.0,clearance/5.0)}
                score=float(sum(parts.values()))
            goal_distance=None; start_goal_distance=None; end_goal_yaw_error=None
            if self.goal is not None and self.goal.header.frame_id=='map':
                g=self.goal.pose.position;goal_distance=float(math.hypot(x[-1,0]-g.x,x[-1,1]-g.y))
                start_goal_distance=float(math.hypot(x[0,0]-g.x,x[0,1]-g.y))
                q=self.goal.pose.orientation;gyaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                delta=float(x[-1,2]-gyaw);end_goal_yaw_error=math.atan2(math.sin(delta),math.cos(delta))
            start_gap=None
            if self.odom is not None and self.odom.header.frame_id=='map':
                o=self.odom.pose.pose.position;start_gap=float(math.hypot(x[0,0]-o.x,x[0,1]-o.y))
            start_ns=msg.start_time.sec*1000000000+msg.start_time.nanosec
            plan_age_s=(self.get_clock().now().nanoseconds-start_ns)*1e-9
            out={'run_id':self.run_id,'trajectory_id':int(msg.traj_id),'received_unix_s':time.time(),'planning_age_s':plan_age_s,
                 'duration_s':float(t[-1]-t[0]),'length_m':length,'start_end_chord_m':chord,
                 'start_goal_distance_m':start_goal_distance,'end_goal_distance_m':goal_distance,
                 'end_goal_yaw_error_rad':end_goal_yaw_error,'receive_start_gap_m':start_gap,
                 'curvature_rms_m_inv':krms,'curvature_max_m_inv':kmax,'curvature_valid_length_fraction':valid_length/max(length,1e-9),
                 'yaw_rate_max_rad_s':rmax,'yaw_acc_max_rad_s2':accmax,'yaw_jerk_rms_rad_s3':jerk,
                 'v_cmd_min_est':vmin,'v_cmd_max_est':vmax,'w_cmd_abs_max_est':wmax,
                 'sway_model_residual_rms_m_s2':sway_residual,
                 'grid_clearance_lower_estimate_m':clearance,'outside_map_samples':outside,
                 'screening':'needs_review' if warnings else 'no_sampled_warning', 'warnings':warnings,
                 'engineering_score_100':score,'score_components':parts,
                 'note':'sampled estimates; not a collision, dynamics or vessel-footprint safety certificate'}
            payload=json.dumps(out,ensure_ascii=False,allow_nan=False)
            if self.output_file:
                os.makedirs(os.path.dirname(os.path.abspath(self.output_file)),exist_ok=True)
                with open(self.output_file,'a',encoding='utf-8') as file:file.write(payload+'\n')
            m=String();m.data=payload;self.pub.publish(m)
            score_text='N/A' if score is None else f'{score:.1f}'
            self.get_logger().info(f'traj={msg.traj_id} score={score_text}/100 screening={out["screening"]} length={length:.2f}m warnings={warnings}')
        except (ValueError, TypeError, OSError, IndexError) as exc:
            self.get_logger().error(f'Trajectory evaluation failed: {exc}')


def main():
    rclpy.init();node=TrajectoryQualityMonitor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
