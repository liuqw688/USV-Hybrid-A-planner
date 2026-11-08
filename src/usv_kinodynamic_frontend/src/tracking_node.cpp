#include <usv_kinodynamic_frontend/shared_force_dynamics.hpp>
#include <usv_kinodynamic_frontend/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <future>
#include <iomanip>
#include <sstream>

namespace f=usv_kinodynamic_frontend;
using namespace std::chrono_literals;
class TrackingFrontend : public rclcpp::Node {
  struct Job { uint64_t id;f::State start,goal;std::shared_ptr<const f::WaterwayMap> map; };
  struct Done { Job job;f::SearchResult result;double seconds; };
 public:
  TrackingFrontend():Node("usv_tracking_frontend") {
    frame_=declare_parameter<std::string>("planning_frame","odom");
    const auto mode=declare_parameter<std::string>("dynamics_model","identified_velocity_commands");
    if(mode!="force" && mode!="identified_velocity_commands")throw std::invalid_argument("unknown dynamics_model");
    identified_=mode=="identified_velocity_commands";
    speed_limit_=declare_parameter<double>("tracking_speed",identified_?1.0:1.5);
    auto_goal_=declare_parameter<bool>("auto_goal",true);
    map_timeout_=declare_parameter<double>("map_timeout",3.0);
    collision_radius_=declare_parameter<double>("search.collision_radius",.7);
    shore_cost_weight_=declare_parameter<double>("search.shore_cost_weight",0.0);
    show_water_plane_=declare_parameter<bool>("display.water_plane",true);
    display_x_=declare_parameter<double>("display.origin_x",0.0);
    display_y_=declare_parameter<double>("display.origin_y",0.0);
    text_height_=declare_parameter<double>("display.text_height",1.0);
    if(!std::isfinite(map_timeout_) || map_timeout_<0 || !std::isfinite(collision_radius_) || collision_radius_<=0 ||
       !std::isfinite(shore_cost_weight_) || shore_cost_weight_<0 || !std::isfinite(display_x_) ||
       !std::isfinite(display_y_) || !std::isfinite(text_height_) || text_height_<=0)
      throw std::invalid_argument("invalid map/collision/display settings");
    goal_.setZero();goal_(0)=declare_parameter<double>("goal.x",40.0);
    goal_(1)=declare_parameter<double>("goal.y",10.0);goal_(2)=declare_parameter<double>("goal.yaw",0.25);
    config_.limits.min_cmd=identified_?declare_parameter<double>("identified.min_v_cmd",-.4):-10;
    config_.limits.max_cmd=identified_?declare_parameter<double>("identified.max_v_cmd",1.5):10;
    config_.limits.max_turn_cmd=identified_?declare_parameter<double>("identified.max_w_cmd",.7):5;
    config_.limits.max_u=declare_parameter<double>("search.max_surge",identified_?1.6:2.5);
    config_.limits.max_v=identified_?.8:1.5;config_.limits.max_r=identified_?.8:.45;config_.limits.allow_reverse=identified_;
    config_.horizon=declare_parameter<double>("search.horizon",500.0);config_.connection_range=120;
    config_.max_nodes=declare_parameter<int>("search.max_nodes",200000);
    config_.connection_durations=declare_parameter<std::vector<double>>("search.connection_durations",identified_?std::vector<double>{24.,36.,48.,64.,80.,100.}:std::vector<double>{12.,20.,28.,36.,48.,64.});
    config_.connection_cruise_speed=declare_parameter<double>("search.connection_cruise_speed",identified_?1.1:1.6);
    config_.duration_scaled_connection_seed=true;
    config_.max_search_time=declare_parameter<double>("search.max_search_time",8.0);
    config_.integration_step=declare_parameter<double>("search.integration_step",config_.integration_step);
    config_.output_step=declare_parameter<double>("search.output_step",config_.output_step);
    config_.goal_position=0.15;config_.goal_heading=0.12;
    config_.goal_u=0.08;config_.goal_v=0.06;config_.goal_r=0.04;config_.validate();
    if(!std::isfinite(speed_limit_) || speed_limit_<0.5 || speed_limit_>config_.limits.max_u || !goal_.allFinite())
      throw std::invalid_argument("invalid tracking speed/goal");
    hybrid_a_star_planner::dynamics::Parameters plant;
    plant.k_u=declare_parameter<double>("model.k_u",plant.k_u);
    plant.k_t=declare_parameter<double>("model.k_t",plant.k_t);
    plant.k_turn=declare_parameter<double>("model.k_turn",plant.k_turn);
    plant.k_v=declare_parameter<double>("model.k_v",plant.k_v);
    plant.k_vr=declare_parameter<double>("model.k_vr",plant.k_vr);
    plant.k_vn=declare_parameter<double>("model.k_vn",plant.k_vn);
    plant.k_r=declare_parameter<double>("model.k_r",plant.k_r);
    plant.k_r3=declare_parameter<double>("model.k_r3",plant.k_r3);
    plant.k_n=declare_parameter<double>("model.k_n",plant.k_n);
    dynamics_=identified_?f::Dynamics{}:f::sharedForceDynamics(plant);
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    const auto latch=rclcpp::QoS(1).reliable().transient_local();
    path_pub_=create_publisher<nav_msgs::msg::Path>(declare_parameter<std::string>("trajectory_topic","/hybrid_a_star_trajectory"),latch);
    speed_pub_=create_publisher<geometry_msgs::msg::TwistStamped>(declare_parameter<std::string>("speed_limit_topic","/hybrid_a_star/speed_limit"),latch);
    planning_pub_=create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("planning_status_topic","/hybrid_a_star/planning_status"),latch);
    active_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/usv_frontend/active_goal",latch);
    debug_pub_=create_publisher<std_msgs::msg::String>("/usv_frontend/status",latch);
    marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/usv_frontend/markers",latch);
    track_pub_=create_publisher<nav_msgs::msg::Path>("/usv_frontend/actual_path",latch);
    states_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/usv_frontend/state_samples",latch);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic","/odom"),20,
      [this](nav_msgs::msg::Odometry::SharedPtr message){onOdom(*message);});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(declare_parameter<std::string>("costmap_topic","/local_costmap"),latch,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message){
        try {
          auto normalized=*message;
          if(normalized.header.frame_id!=frame_) {
            geometry_msgs::msg::PoseStamped origin;origin.header=normalized.header;origin.pose=normalized.info.origin;
            normalized.info.origin=normalize(origin).pose;normalized.header.frame_id=frame_;
          }
          auto next=std::make_shared<f::WaterwayMap>(f::fromOccupancyGrid(normalized));map_=next;map_receipt_=now();
        }catch(const std::exception& error){map_.reset();RCLCPP_WARN(get_logger(),"Map rejected: %s",error.what());}
      });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(declare_parameter<std::string>("goal_topic","/goal_pose_from_astar"),10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message){onGoal(*message);});
    rviz_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message){onGoal(*message);});
    cancel_sub_=create_subscription<std_msgs::msg::Empty>("/usv_frontend/cancel",10,
      [this](std_msgs::msg::Empty::SharedPtr){++generation_;has_goal_=false;pending_=false;withdraw("cancelled");});
    timer_=create_wall_timer(50ms,[this]{tick();});
    publishPlanning("idle");
    RCLCPP_INFO(get_logger(),"Tracking frontend: /odom + /local_costmap + /goal_pose_from_astar or /goal_pose -> existing Path/speed/status interfaces. Model=%s",dynamics_.name.c_str());
  }
 private:
  bool identified_{true};
  geometry_msgs::msg::PoseStamped normalize(const geometry_msgs::msg::PoseStamped& input) {
    if(input.header.frame_id.empty())throw std::invalid_argument("empty frame");
    auto out=input;
    const auto& q=input.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if(!std::isfinite(norm) || norm<1e-9 || !std::isfinite(input.pose.position.x) || !std::isfinite(input.pose.position.y))
      throw std::invalid_argument("non-finite pose or quaternion");
    out.pose.orientation.x/=norm;out.pose.orientation.y/=norm;out.pose.orientation.z/=norm;out.pose.orientation.w/=norm;
    if(input.header.frame_id!=frame_) {
      const auto transform=tf_buffer_->lookupTransform(frame_,input.header.frame_id,tf2::TimePointZero);
      if(std::abs(transform.transform.rotation.x)>1e-4 || std::abs(transform.transform.rotation.y)>1e-4)
        throw std::invalid_argument("non-planar world transform");
      tf2::doTransform(out,out,transform);
    }
    if(std::abs(out.pose.orientation.x)>1e-4 || std::abs(out.pose.orientation.y)>1e-4)
      throw std::invalid_argument("non-planar pose");
    out.header.frame_id=frame_;return out;
  }
  static double yaw(const geometry_msgs::msg::Quaternion& q) {return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
  geometry_msgs::msg::Pose poseOf(const f::State& state,double z=0.08) const {
    geometry_msgs::msg::Pose p;p.position.x=state(0);p.position.y=state(1);p.position.z=z;
    p.orientation.z=std::sin(state(2)/2);p.orientation.w=std::cos(state(2)/2);return p;
  }
  void onOdom(const nav_msgs::msg::Odometry& odom) {
    try {
      geometry_msgs::msg::PoseStamped p;p.header=odom.header;p.pose=odom.pose.pose;p=normalize(p);
      f::State state;state<<p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation),
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z;
      if(!state.allFinite() || odom.child_frame_id.empty())throw std::invalid_argument("invalid body velocity/frame");
      measured_=state;odom_stamp_=rclcpp::Time(odom.header.stamp);have_odom_=true;
      if(tracking_) {
        if(actual_.poses.empty() || std::hypot(p.pose.position.x-actual_.poses.back().pose.position.x,
           p.pose.position.y-actual_.poses.back().pose.position.y)>0.04) {
          actual_.poses.push_back(p);
          if(actual_.poses.size()>20000)actual_.poses.erase(actual_.poses.begin(),actual_.poses.begin()+1000);
        }
      }
    }catch(const std::exception& error){have_odom_=false;RCLCPP_WARN(get_logger(),"Odometry rejected: %s",error.what());}
  }
  void onGoal(const geometry_msgs::msg::PoseStamped& message) {
    try {
      const auto target=normalize(message);goal_.setZero();goal_<<target.pose.position.x,target.pose.position.y,yaw(target.pose.orientation),0,0,0;
      auto_goal_=false;has_goal_=true;newRequest();
    }catch(const std::exception& error){RCLCPP_WARN(get_logger(),"Goal rejected: %s",error.what());}
  }
  void publishPlanning(const std::string& status) {std_msgs::msg::String m;m.data=status;planning_pub_->publish(m);}
  void withdraw(const std::string& status) {
    tracking_=false;phase_=status;path_.poses.clear();
    nav_msgs::msg::Path empty;empty.header.frame_id=frame_;empty.header.stamp=now();path_pub_->publish(empty);
    publishPlanning(status);publishDebug();
  }
  void newRequest() {
    ++generation_;pending_=true;arrived_=false;actual_.poses.clear();withdraw("planning");
    geometry_msgs::msg::PoseStamped target;target.header.frame_id=frame_;target.header.stamp=now();target.pose=poseOf(goal_,0);active_pub_->publish(target);
  }
  bool inputsReady() const {
    const double age=(now()-odom_stamp_).seconds();
    return have_odom_ && map_ && age>=-0.1 && age<0.8 &&
      (map_timeout_==0 || (now()-map_receipt_).seconds()<map_timeout_);
  }
  void tick() {
    if(!inputsReady()) {
      if(tracking_){withdraw("blocked");pending_=has_goal_;}
      return;
    }
    if(auto_goal_ && !has_goal_){has_goal_=true;newRequest();}
    if(worker_.valid() && worker_.wait_for(0ms)==std::future_status::ready) {
      try {
        auto completed=worker_.get();
        if(completed.job.id==generation_ && has_goal_) {
          const auto change=measured_-completed.job.start;
          if(change.head<2>().norm()>0.25 || std::abs(f::wrapAngle(change(2)))>0.1 || change.tail<3>().cwiseAbs().maxCoeff()>0.15) {
            pending_=true;RCLCPP_WARN(get_logger(),"Planning result starts at a stale measured state; retrying from latest /odom");
          } else accept(completed);
        }
      }catch(const std::exception& error){withdraw("blocked");RCLCPP_ERROR(get_logger(),"Planning error: %s",error.what());}
    }
    if(pending_ && !worker_.valid()) {
      pending_=false;const Job job{generation_,measured_,goal_,map_};const auto config=config_;const auto dynamics=dynamics_;
      const double radius=collision_radius_,shore_weight=shore_cost_weight_;
      worker_=std::async(std::launch::async,[job,config,dynamics,radius,shore_weight]{
        const auto begin=std::chrono::steady_clock::now();
        auto result=f::KinodynamicSearch(config,dynamics).search(job.start,job.goal,
          [job,radius](const f::State& state){return job.map->footprintFree(state,radius);},
          [job,shore_weight](const f::State& state){return shore_weight*job.map->softCost(state);});
        return Done{job,std::move(result),std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()};
      });
    }
    if(tracking_ && ++ticks_%4==0) {
      publishPathPair();actual_.header.frame_id=frame_;actual_.header.stamp=now();track_pub_->publish(actual_);
      const double distance=(measured_.head<2>()-goal_.head<2>()).norm();
      if(distance<0.8 && measured_.tail<2>().norm()<0.1 && std::hypot(measured_(3),measured_(4))<0.1 &&
        std::abs(f::wrapAngle(measured_(2)-goal_(2)))<f::pi/6) {arrived_=true;phase_="arrived";}
      publishDebug();
    }
    if(++display_ticks_%4==0)publishMarkers();
  }
  void accept(const Done& completed) {
    const auto& result=completed.result;
    search_seconds_=completed.seconds;expanded_=result.expanded;
    if(result.status!=f::SearchResult::REACH_END || result.samples.size()<2) {
      withdraw("blocked");RCLCPP_WARN(get_logger(),"No complete path: status=%d expanded=%d time=%.3f",result.status,result.expanded,completed.seconds);return;
    }
    // Recheck the complete result against the latest map before publication.
    for(const auto& sample:result.samples)if(!map_->footprintFree(sample.state,collision_radius_)) {withdraw("blocked");return;}
    path_.poses.clear();path_.header.frame_id=frame_;
    for(size_t i=0;i<result.samples.size();++i) {
      const auto& sample=result.samples[i];
      if(i!=0 && i+1!=result.samples.size() && !path_.poses.empty() &&
        std::hypot(sample.state(0)-path_.poses.back().pose.position.x,sample.state(1)-path_.poses.back().pose.position.y)<0.2)continue;
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id=frame_;pose.pose=poseOf(sample.state,0);path_.poses.push_back(pose);
    }
    // Circumcircle curvature of consecutive displayed path positions. Very
    // short chords near a stop do not support a stable geometric estimate.
    curvature_valid_=false;max_curvature_=0;curvature_segment_.clear();
    std::vector<double> path_distance(path_.poses.size(),0.0);
    for(size_t i=1;i<path_.poses.size();++i) {
      const auto &a=path_.poses[i-1].pose.position,&b=path_.poses[i].pose.position;
      path_distance[i]=path_distance[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    constexpr double endpoint_exclusion=2.0; // Metres along the reference path.
    for(size_t i=1;i+1<path_.poses.size();++i) {
      if(path_distance[i-1]<endpoint_exclusion ||
         path_distance.back()-path_distance[i+1]<endpoint_exclusion)continue;
      const auto &a=path_.poses[i-1].pose.position,&b=path_.poses[i].pose.position,&c=path_.poses[i+1].pose.position;
      const double ab=std::hypot(b.x-a.x,b.y-a.y),bc=std::hypot(c.x-b.x,c.y-b.y),ac=std::hypot(c.x-a.x,c.y-a.y);
      if(ab<0.10 || bc<0.10 || ac<0.10)continue;
      const double cross=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
      const double curvature=2.0*std::abs(cross)/(ab*bc*ac);
      if(std::isfinite(curvature) && (!curvature_valid_ || curvature>max_curvature_)) {
        curvature_valid_=true;max_curvature_=curvature;curvature_position_=b;
        curvature_segment_.clear();
        size_t first=i>5?i-5:0;
        size_t last=std::min(path_.poses.size()-1,i+5);
        while(first<i && path_distance[first]<endpoint_exclusion)++first;
        while(last>i && path_distance.back()-path_distance[last]<endpoint_exclusion)--last;
        for(size_t j=first;j<=last;++j)curvature_segment_.push_back(path_.poses[j].pose.position);
      }
    }
    std_msgs::msg::Float64MultiArray states;states.layout.dim.resize(2);
    states.layout.dim[0].label="samples";states.layout.dim[0].size=result.samples.size();states.layout.dim[0].stride=result.samples.size()*9;
    states.layout.dim[1].label=identified_?"time,x,y,psi,u,v,r,v_cmd,w_cmd":"time,x,y,psi,u,v,r,tau_y,tau_n";states.layout.dim[1].size=9;states.layout.dim[1].stride=9;
    for(const auto& sample:result.samples){states.data.push_back(sample.time);for(int i=0;i<6;++i)states.data.push_back(sample.state(i));states.data.push_back(sample.command(0));states.data.push_back(sample.command(1));}
    states_pub_->publish(states);
    tracking_=true;phase_="tracking";reference_start_=completed.job.start;reference_end_=result.samples.back().state;
    trajectory_origin_=result.origin;
    reference_duration_=result.samples.back().time;actual_.poses.clear();
    publishPathPair();publishDebug();
    RCLCPP_INFO(get_logger(),"Tracking path accepted: start=(%.2f,%.2f,u=%.2f,v=%.2f,r=%.2f) goal=(%.2f,%.2f), %zu poses, %.3f s search, %.1f s reference duration",
      reference_start_(0),reference_start_(1),reference_start_(3),reference_start_(4),reference_start_(5),goal_(0),goal_(1),path_.poses.size(),search_seconds_,reference_duration_);
  }
  void publishPathPair() {
    path_.header.stamp=now();for(auto& pose:path_.poses)pose.header=path_.header;
    geometry_msgs::msg::TwistStamped profile;profile.header=path_.header;profile.twist.linear.x=speed_limit_;
    path_pub_->publish(path_);speed_pub_->publish(profile);publishPlanning("tracking");
  }
  void publishDebug() {
    std::ostringstream out;out<<std::setprecision(15)<<"{\"request_id\":"<<generation_<<",\"status\":\""<<phase_
      <<"\",\"model\":\""<<dynamics_.name<<"\",\"search_seconds\":"<<search_seconds_<<",\"expanded\":"<<expanded_
      <<",\"reference_duration\":"<<reference_duration_<<",\"path_points\":"<<path_.poses.size();
    auto vector=[&](const char* name,const f::State& state){out<<",\""<<name<<"\":[";for(int i=0;i<6;++i)out<<(i?",":"")<<state(i);out<<"]";};
    vector("goal",goal_);if(have_odom_)vector("measured",measured_);
    if(tracking_){vector("reference_start",reference_start_);vector("reference_end",reference_end_);
      if(curvature_valid_) {out<<",\"max_path_curvature_per_m\":"<<max_curvature_<<",\"min_path_radius_m\":";
        if(max_curvature_>1e-6)out<<1.0/max_curvature_;else out<<"null";}}
    out<<"}";std_msgs::msg::String status;status.data=out.str();debug_pub_->publish(status);
  }
  void publishMarkers() {
    using Marker=visualization_msgs::msg::Marker;visualization_msgs::msg::MarkerArray array;
    Marker clear;clear.header.frame_id=frame_;clear.action=Marker::DELETEALL;array.markers.push_back(clear);
    auto base=[&](int id,int type){Marker m;m.header.frame_id=frame_;m.header.stamp=now();m.ns="live_tracking";m.id=id;m.type=type;m.action=Marker::ADD;m.pose.orientation.w=1;m.color.a=1;return m;};
    if(show_water_plane_) {
      auto water=base(0,Marker::CUBE);water.pose.position.z=-0.08;water.scale.x=200;water.scale.y=160;water.scale.z=0.02;
      water.color.r=0.02;water.color.g=0.10;water.color.b=0.16;array.markers.push_back(water);
    }
    if(has_goal_){auto goal=base(1,Marker::ARROW);goal.pose=poseOf(goal_,0.2);goal.scale.x=2;goal.scale.y=0.4;goal.scale.z=0.5;
      goal.color.r=1;goal.color.g=0.3;goal.color.b=0.2;array.markers.push_back(goal);}
    if(tracking_){auto start=base(2,Marker::ARROW);start.pose=poseOf(reference_start_,0.2);start.scale.x=2;start.scale.y=0.4;start.scale.z=0.5;
      start.color.r=0.2;start.color.g=1;start.color.b=0.3;array.markers.push_back(start);}
    // One arrow per published Path node; orientation is hull psi, not tangent.
    if(tracking_)for(size_t i=0;i<path_.poses.size();++i) {
      auto arrow=base(static_cast<int>(i),Marker::ARROW);arrow.ns="reference_heading";
      arrow.pose=path_.poses[i].pose;arrow.pose.position.z=.18;
      arrow.scale.x=.45;arrow.scale.y=.055;arrow.scale.z=.10;
      arrow.color.r=1.;arrow.color.g=.55;arrow.color.b=.12;
      array.markers.push_back(arrow);
    }
    if(tracking_ && curvature_valid_) {
      auto segment=base(6,Marker::LINE_STRIP);segment.scale.x=.32;
      segment.color.r=1;segment.color.g=.05;segment.color.b=.8;
      segment.points=curvature_segment_;for(auto& p:segment.points)p.z=.35;
      array.markers.push_back(segment);
      auto point=base(4,Marker::SPHERE);point.pose.position=curvature_position_;point.pose.position.z=.4;
      point.scale.x=point.scale.y=point.scale.z=.7;point.color.r=1;point.color.g=.1;point.color.b=.8;array.markers.push_back(point);
      auto curvature_text=base(5,Marker::TEXT_VIEW_FACING);curvature_text.pose.position=curvature_position_;
      curvature_text.pose.position.z=2.0;curvature_text.scale.z=text_height_;
      curvature_text.color.r=1;curvature_text.color.g=.4;curvature_text.color.b=.9;
      std::ostringstream value;value<<std::fixed<<std::setprecision(3)<<"Max curvature: "<<max_curvature_<<" 1/m\nMin radius: ";
      if(max_curvature_>1e-6)value<<std::setprecision(2)<<1.0/max_curvature_<<" m";else value<<"infinity (straight)";
      curvature_text.text=value.str();array.markers.push_back(curvature_text);
    }
    auto text=base(3,Marker::TEXT_VIEW_FACING);text.pose.position.x=display_x_+20;
    text.pose.position.y=display_y_+(show_water_plane_?17:45);
    text.pose.position.z=0.4;text.scale.z=text_height_;
    text.color.r=text.color.g=text.color.b=0.95;
    std::ostringstream label;label<<"LIVE MPC TRACKING / "<<(identified_?"IDENTIFIED 11-PARAMETER MODEL":"SHARED FORCE MODEL")<<"\n"<<phase_<<"  |  speed limit "<<speed_limit_<<" m/s\n"
      <<"Cyan: reference   Orange arrows: node hull heading\nYellow: measured track   Blue: simulated vessel\n2D Goal Pose: new target   No other vessels";
    if(have_odom_ && has_goal_)label<<"\nGoal distance: "<<std::fixed<<std::setprecision(2)<<(measured_.head<2>()-goal_.head<2>()).norm()<<" m";
    if(tracking_) {
      label<<"\nTrajectory: ";
      if(trajectory_origin_==f::SearchResult::DIRECT_CONNECTION)label<<"direct numerical connection";
      else if(trajectory_origin_==f::SearchResult::SEARCH_THEN_CONNECTION)label<<"Kinodynamic A* search + numerical connection";
      else if(trajectory_origin_==f::SearchResult::SEARCH_ONLY)label<<"Kinodynamic A* search";
      else label<<"unknown";
    }
    text.text=label.str();array.markers.push_back(text);marker_pub_->publish(array);
  }
  f::SearchConfig config_;f::Dynamics dynamics_;
  double map_timeout_{3},collision_radius_{.7},shore_cost_weight_{0},display_x_{0},display_y_{0},text_height_{1};
  bool show_water_plane_{true};
  bool curvature_valid_{false};double max_curvature_{0};geometry_msgs::msg::Point curvature_position_;
  std::vector<geometry_msgs::msg::Point> curvature_segment_;
  f::SearchResult::Origin trajectory_origin_{f::SearchResult::UNKNOWN};
  f::State measured_=f::State::Zero(),goal_=f::State::Zero(),reference_start_=f::State::Zero(),reference_end_=f::State::Zero();
  std::string frame_,phase_="idle";double speed_limit_,search_seconds_=0,reference_duration_=0;int expanded_=0,ticks_=0,display_ticks_=0;
  bool auto_goal_=false,have_odom_=false,has_goal_=false,pending_=false,tracking_=false,arrived_=false;
  uint64_t generation_=0;std::shared_ptr<const f::WaterwayMap> map_;
  rclcpp::Time odom_stamp_{0,0,RCL_ROS_TIME},map_receipt_{0,0,RCL_ROS_TIME};
  nav_msgs::msg::Path path_,actual_;std::future<Done> worker_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_,rviz_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_,track_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_pub_,debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<TrackingFrontend>());}catch(const std::exception& e){std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
