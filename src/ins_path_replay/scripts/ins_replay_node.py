#!/usr/bin/env python3
"""Publish recorded waypoints through the existing A* output interfaces."""
import csv
import json
import math
import signal
from pathlib import Path as FilePath
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from ins_path_replay.route import load_route, decimate, distance_to_polyline


class InsReplay(Node):
    def __init__(self):
        super().__init__('ins_path_replay')
        default_csv = str(FilePath(get_package_share_directory('ins_path_replay')) /
                          'data/data_ins_20261019_153413.csv')
        for name, value in {
            'csv_path': default_csv, 'point_count': 30, 'angle_unit': 'radians',
            'latitude_column': 'INS_LAT', 'longitude_column': 'INS_LON',
            'altitude_column': 'ins_altitude', 'frame_id': 'odom',
            'advance_radius': 3.0, 'final_radius': 0.6, 'final_speed': 0.15,
            'map_margin': 100.0, 'map_resolution': 1.0,
            'output_dir': '/tmp/ins_path_replay',
        }.items():
            self.declare_parameter(name, value)
        def p(name): return self.get_parameter(name).value
        self.route_data = load_route(p('csv_path'), p('point_count'), p('latitude_column'),
                                     p('longitude_column'), p('altitude_column'), p('angle_unit'))
        self.frame = p('frame_id')
        self.advance_radius, self.final_radius, self.final_speed = p('advance_radius'), p('final_radius'), p('final_speed')
        if min(self.advance_radius, self.final_radius, self.final_speed, p('map_margin'), p('map_resolution')) <= 0:
            raise ValueError('Radii, speed threshold and map dimensions must be positive')
        self.out = FilePath(p('output_dir')).expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out/'route_metadata.json').write_text(json.dumps(self.route_data.metadata, ensure_ascii=False, indent=2))
        with open(self.out/'waypoints_30.csv', 'w') as f:
            writer = csv.writer(f); writer.writerow(['index','source_arc_m','east_m','north_m','yaw_rad','yaw_deg'])
            for i, ((x,y), yaw, arc) in enumerate(zip(self.route_data.points, self.route_data.headings, self.route_data.sample_arcs)):
                writer.writerow([i,arc,x,y,yaw,math.degrees(yaw)])
        with open(self.out/'source_enu.csv','w') as f:
            writer=csv.writer(f); writer.writerow(['east_m','north_m']); writer.writerows(self.route_data.raw)
        self.trace_file=open(self.out/'tracking.csv','w', buffering=1)
        self.trace=csv.writer(self.trace_file)
        self.trace.writerow(['elapsed_s','x','y','yaw','u','v','r','active_index','distance_to_goal_m','distance_to_hybrid_path_m'])
        self.events_file=open(self.out/'waypoint_events.jsonl','w',buffering=1)
        self.plan_file=open(self.out/'local_paths.jsonl','w',buffering=1)
        self.started_at=time.monotonic()
        self.last_sample=-1.; self.last_plot=-1.; self.last_dump=-1.
        self.index=0; self.running=False; self.completed=False
        self.started_elapsed=None; self.completed_elapsed=None; self.final_hold=None
        self.terminal_settled=False; self.terminal_settle_hold=None
        self.odom=None; self.local_path=[]; self.last_local_key=None
        self.events=[]; self.trajectory=Path(); self.trajectory.header.frame_id=self.frame
        self.qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub=self.create_publisher(Path,'/astar_channel_waypoints',self.qos)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose_from_astar',self.qos)
        self.raw_pub=self.create_publisher(Path,'/ins_replay/source_path',self.qos)
        self.track_pub=self.create_publisher(Path,'/ins_replay/tracked_path',self.qos)
        self.map_pub=self.create_publisher(OccupancyGrid,'/local_costmap',self.qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/ins_replay/waypoint_markers',self.qos)
        self.status_pub=self.create_publisher(String,'/ins_replay/status',self.qos)
        self.create_subscription(Odometry,'/odom',self.on_odom,20)
        self.create_subscription(Path,'/hybrid_a_star_trajectory',self.on_path,self.qos)
        self.epoch=self.get_clock().now().to_msg()
        self.route=Path(); self.route.header.frame_id=self.frame; self.route.header.stamp=self.epoch
        for point,yaw in zip(self.route_data.points,self.route_data.headings):
            self.route.poses.append(self.pose(point,yaw))
        self.raw_path=Path(); self.raw_path.header=self.route.header
        self.raw_path.poses=[self.pose(point,0.) for point in decimate(self.route_data.raw,0.5)]
        self.map=OccupancyGrid(); self.map.header.frame_id=self.frame
        resolution=p('map_resolution'); margin=p('map_margin')
        xs,ys=zip(*self.route_data.raw)
        ox=math.floor((min(xs)-margin)/resolution)*resolution
        oy=math.floor((min(ys)-margin)/resolution)*resolution
        self.map.info.resolution=resolution
        self.map.info.width=math.ceil((max(xs)+margin-ox)/resolution)
        self.map.info.height=math.ceil((max(ys)+margin-oy)/resolution)
        self.map.info.origin.position.x=ox; self.map.info.origin.position.y=oy
        self.map.info.origin.orientation.w=1.
        if self.map.info.width*self.map.info.height > 4000000:
            raise ValueError('Map exceeds 4 million cells; increase map_resolution')
        self.map.data=[0]*(self.map.info.width*self.map.info.height)
        self.publish_references()
        self.create_timer(0.1,self.tick)
        self.create_timer(1.,self.publish_references)
        self.get_logger().info(f'{len(self.route_data.raw)} INS samples → {len(self.route.poses)} ENU waypoints; '
                               f'{self.route_data.metadata["raw_length_m"]:.2f}m; x east, y north; open water')

    def pose(self, point, yaw):
        msg=PoseStamped(); msg.header.frame_id=self.frame; msg.header.stamp=self.epoch
        msg.pose.position.x=float(point[0]); msg.pose.position.y=float(point[1])
        msg.pose.orientation.z=math.sin(yaw/2); msg.pose.orientation.w=math.cos(yaw/2)
        return msg

    def event(self, kind, distance):
        item={'event':kind,'index':self.index,'elapsed_s':time.monotonic()-self.started_at,
              'distance_m':distance,'target':self.route_data.points[self.index]}
        self.events.append(item); self.events_file.write(json.dumps(item)+'\n')
        self.get_logger().info(f'{kind}: waypoint {self.index+1}/{len(self.route.poses)}, distance {distance:.3f}m')

    def on_odom(self,msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().error(f'Odometry frame {msg.header.frame_id} differs from {self.frame}')
            return
        self.odom=msg
        t=time.monotonic()-self.started_at
        if t-self.last_sample < 0.095: return
        self.last_sample=t
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        dist=math.dist((p.x,p.y),self.route_data.points[self.index])
        local_error=distance_to_polyline(p.x,p.y,self.local_path) if len(self.local_path)>1 else ''
        self.trace.writerow([t,p.x,p.y,yaw,msg.twist.twist.linear.x,msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z,self.index,dist,local_error])
        if not self.trajectory.poses or math.dist((p.x,p.y),
            (self.trajectory.poses[-1].pose.position.x,self.trajectory.poses[-1].pose.position.y))>0.1:
            pose=PoseStamped(); pose.header=msg.header; pose.pose=msg.pose.pose
            self.trajectory.poses.append(pose)

    def on_path(self,msg):
        if msg.header.frame_id != self.frame: return
        self.local_path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        # Cache heartbeats change timestamps; retain only genuinely changed geometry.
        key=tuple((round(x,4),round(y,4)) for x,y in self.local_path)
        if key != self.last_local_key:
            self.last_local_key=key
            self.plan_file.write(json.dumps({'elapsed_s':time.monotonic()-self.started_at,
                'active_index':self.index,'points':self.local_path})+'\n')

    def tick(self):
        t=time.monotonic()-self.started_at
        if not self.running:
            # Require both Hybrid and MPC goal consumers and an actual simulator state.
            if self.odom is not None and self.goal_pub.get_subscription_count()>=2:
                self.running=True; self.started_elapsed=t
                self.route_pub.publish(self.route)
                self.goal_pub.publish(self.route.poses[self.index]); self.event('activated',0.)
            self.publish_status(); return
        if not self.completed:
            odom_age=(self.get_clock().now().nanoseconds-
                      (self.odom.header.stamp.sec*10**9+self.odom.header.stamp.nanosec))/1e9
            if odom_age>1.0:
                self.publish_status(); return
            p=self.odom.pose.pose.position
            distance=math.dist((p.x,p.y),self.route_data.points[self.index])
            if self.index<len(self.route.poses)-1:
                if distance<=self.advance_radius:
                    self.event('reached',distance)
                    self.index+=1
                    self.goal_pub.publish(self.route.poses[self.index]); self.event('activated',math.dist((p.x,p.y),self.route_data.points[self.index]))
            else:
                q=self.odom.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                heading_error=abs(math.atan2(math.sin(yaw-self.route_data.headings[-1]),math.cos(yaw-self.route_data.headings[-1])))
                speed=math.hypot(self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y)
                if distance<=self.final_radius and speed<=self.final_speed and heading_error<=math.pi/6:
                    if self.final_hold is None: self.final_hold=t
                    if t-self.final_hold>=2.:
                        self.completed=True; self.completed_elapsed=t; self.event('completed',distance)
                        self.save_summary()
                else: self.final_hold=None
                if distance<=self.advance_radius and speed<=self.final_speed and heading_error<=math.pi/6:
                    if self.terminal_settle_hold is None: self.terminal_settle_hold=t
                    if t-self.terminal_settle_hold>=2. and not self.terminal_settled:
                        self.terminal_settled=True
                        self.event('terminal_settled',distance)
                        if not self.completed:
                            self.get_logger().warn(
                                f'Terminal stopped at {distance:.3f}m error; strict {self.final_radius:.3f}m criterion NOT met')
                        self.save_summary()
                else:
                    self.terminal_settle_hold=None; self.terminal_settled=False
        self.publish_status()
        if t-self.last_dump>5:
            self.last_dump=t; self.save_summary()

    def publish_status(self):
        state=self.summary()
        self.status_pub.publish(String(data=json.dumps(state)))

    def summary(self):
        return {'completed':self.completed,'running':self.running,'active_index':self.index,
                'terminal_settled':self.terminal_settled,
                'point_count':len(self.route.poses),'elapsed_s':time.monotonic()-self.started_at,
                'started_elapsed_s':self.started_elapsed,'completed_elapsed_s':self.completed_elapsed,
                'waypoints_reached':sum(e['event'] in ('reached','completed') for e in self.events),
                'advance_radius_m':self.advance_radius,'final_radius_m':self.final_radius,
                'final_speed_threshold_mps':self.final_speed,'output_dir':str(self.out)}

    def save_summary(self):
        path=self.out/'replay_summary.json'
        tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(self.summary(),indent=2)); tmp.replace(path)

    def publish_references(self):
        self.map.header.stamp=self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        # Keep route/goal epoch constant; a heartbeat is not a new mission.
        self.route_pub.publish(self.route); self.raw_pub.publish(self.raw_path)
        if self.running: self.goal_pub.publish(self.route.poses[self.index])
        self.trajectory.header.stamp=self.get_clock().now().to_msg()
        self.track_pub.publish(self.trajectory)
        markers=MarkerArray()
        for i,pose in enumerate(self.route.poses):
            arrow=Marker(); arrow.header=self.route.header; arrow.ns='waypoint_heading'; arrow.id=i
            arrow.type=Marker.ARROW; arrow.action=Marker.ADD; arrow.pose=pose.pose
            arrow.scale.x=5.; arrow.scale.y=1.; arrow.scale.z=0.8
            arrow.color.a=1.; arrow.color.g=0.9 if i<self.index else 0.65
            arrow.color.r=1. if i==self.index else 0.2; arrow.color.b=0.2
            markers.markers.append(arrow)
            text=Marker(); text.header=self.route.header; text.ns='waypoint_number'; text.id=i
            text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD
            text.pose.position.x=pose.pose.position.x; text.pose.position.y=pose.pose.position.y+3.
            text.pose.position.z=2.; text.pose.orientation.w=1.; text.scale.z=3.
            text.color.r=text.color.g=text.color.b=text.color.a=1.; text.text=str(i+1)
            markers.markers.append(text)
        # Render the all-free map boundary without an indexed texture shader.
        # The actual OccupancyGrid remains published for Hybrid and MPC.
        boundary=Marker(); boundary.header.frame_id=self.frame
        boundary.ns='open_water_boundary'; boundary.id=0; boundary.type=Marker.LINE_STRIP
        boundary.action=Marker.ADD; boundary.pose.orientation.w=1.; boundary.scale.x=0.8
        boundary.color.r=0.15; boundary.color.g=0.45; boundary.color.b=0.8; boundary.color.a=0.8
        ox=self.map.info.origin.position.x; oy=self.map.info.origin.position.y
        width=self.map.info.width*self.map.info.resolution; height=self.map.info.height*self.map.info.resolution
        boundary.points=[Point(x=x,y=y,z=0.) for x,y in
                         [(ox,oy),(ox+width,oy),(ox+width,oy+height),(ox,oy+height),(ox,oy)]]
        markers.markers.append(boundary)
        self.marker_pub.publish(markers)

    def close(self):
        self.save_summary()
        for f in (self.trace_file,self.events_file,self.plan_file): f.close()


def main():
    rclpy.init(); node=None
    try:
        node=InsReplay(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The evaluator and ROS launch can both send SIGINT while nodes exit.
        # Preserve telemetry and finish destruction without a second interrupt.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None: node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
