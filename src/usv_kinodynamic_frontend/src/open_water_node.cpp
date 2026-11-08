#include <usv_kinodynamic_frontend/kinodynamic_astar.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <future>
#include <iomanip>
#include <locale>
#include <sstream>

namespace frontend=usv_kinodynamic_frontend;
using namespace std::chrono_literals;
class OpenWaterNode : public rclcpp::Node {
  struct Request { uint64_t id; frontend::State start,goal; };
  struct Completed { Request request; frontend::SearchResult result; double elapsed; };
 public:
  OpenWaterNode() : Node("usv_open_water_frontend") {
    frame_=declare_parameter<std::string>("frame_id","map");
    extent_=declare_parameter<double>("open_water.half_extent",40.0);
    radius_=declare_parameter<double>("hull_radius",0.5);
    config_.horizon=declare_parameter<double>("search.horizon",500.0);
    config_.max_search_time=declare_parameter<double>("search.max_search_time",8.0);
    config_.integration_step=declare_parameter<double>("search.integration_step",config_.integration_step);
    config_.output_step=declare_parameter<double>("search.output_step",config_.output_step);
    config_.connection_range=declare_parameter<double>("search.connection_range",30.0);
    config_.connection_durations=declare_parameter<std::vector<double>>("search.connection_durations",
      {4.0,6.0,8.0,12.0,16.0,24.0,36.0});
    config_.max_nodes=declare_parameter<int>("search.max_nodes",200000);
    config_.heuristic_weight=declare_parameter<double>("search.heuristic_weight",5.0);
    config_.goal_position=declare_parameter<double>("goal.position_tolerance",config_.goal_position);
    config_.goal_heading=declare_parameter<double>("goal.heading_tolerance",config_.goal_heading);
    config_.validate();
    if(frame_.empty() || !std::isfinite(extent_) || !std::isfinite(radius_) || radius_<=0 || extent_<=radius_)
      throw std::invalid_argument("invalid open-water frame or extent/radius");
    start_.setZero();goal_.setZero();
    start_(0)=declare_parameter<double>("start.x",0.0);
    start_(1)=declare_parameter<double>("start.y",0.0);
    start_(2)=declare_parameter<double>("start.yaw",0.0);
    start_(3)=declare_parameter<double>("start.surge",0.0);
    start_(4)=declare_parameter<double>("start.sway",0.0);
    start_(5)=declare_parameter<double>("start.yaw_rate",0.0);
    goal_(0)=declare_parameter<double>("goal.x",2.0);
    goal_(1)=declare_parameter<double>("goal.y",1.0);
    goal_(2)=declare_parameter<double>("goal.yaw",0.5);
    if(!start_.allFinite() || !goal_.allFinite()) throw std::invalid_argument("non-finite start/goal");
    playback_=declare_parameter<bool>("preview.playback",true);
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    path_pub_=create_publisher<nav_msgs::msg::Path>("/usv_frontend/path",qos);
    marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/usv_frontend/markers",qos);
    status_pub_=create_publisher<std_msgs::msg::String>("/usv_frontend/status",qos);
    samples_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/usv_frontend/state_samples",qos);
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/usv_frontend/goal",10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
        frontend::State next=frontend::State::Zero();
        if(readPose(msg->header.frame_id,msg->pose,next)){goal_=next;queue();}
      });
    start_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/usv_frontend/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        frontend::State next=frontend::State::Zero();
        if(readPose(msg->header.frame_id,msg->pose.pose,next)){start_=next;queue();}
      });
    fixed_frame_=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped identity;
    identity.header.frame_id=frame_;identity.child_frame_id="usv_open_water_origin";
    identity.header.stamp=now();identity.transform.rotation.w=1;
    fixed_frame_->sendTransform(identity);
    timer_=create_wall_timer(50ms,[this]{tick();});
    queue();
    RCLCPP_INFO(get_logger(),"Open-water frontend ready. RViz: 2D Pose Estimate sets stationary start, 2D Goal Pose sets goal. No controller/obstacles/other vessels.");
  }
 private:
  bool readPose(const std::string& frame,const geometry_msgs::msg::Pose& pose,frontend::State& state) {
    const auto& q=pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if(frame!=frame_ || !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(norm) || norm<1e-9) {
      RCLCPP_WARN(get_logger(),"Rejected pose: requires finite x/y and valid quaternion in frame '%s'",frame_.c_str());
      return false;
    }
    const double x=q.x/norm,y=q.y/norm,z=q.z/norm,w=q.w/norm;
    if(std::abs(x)>1e-5 || std::abs(y)>1e-5){RCLCPP_WARN(get_logger(),"Rejected non-planar pose");return false;}
    state(0)=pose.position.x;state(1)=pose.position.y;
    state(2)=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    return true;
  }
  geometry_msgs::msg::Pose poseOf(const frontend::State& s,double height=0.08) const {
    geometry_msgs::msg::Pose pose;pose.position.x=s(0);pose.position.y=s(1);pose.position.z=height;
    pose.orientation.z=std::sin(s(2)/2);pose.orientation.w=std::cos(s(2)/2);return pose;
  }
  void clearPath() {
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();path_pub_->publish(path);
    std_msgs::msg::Float64MultiArray samples;samples_pub_->publish(samples);
    current_.samples.clear();
  }
  void queue() {
    ++generation_;pending_=true;clearPath();label_="PLANNING";
    publishStatus("planning",nullptr,0);
    publishMarkers();
  }
  void publishStatus(const std::string& status,const frontend::SearchResult* result,double elapsed) {
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(15);
    out<<"{\"request_id\":"<<generation_<<",\"status\":\""<<status<<"\",\"search_seconds\":"<<elapsed;
    const auto vector=[&](const char* name,const frontend::State& state){
      out<<",\""<<name<<"\":[";for(int i=0;i<6;++i)out<<(i?",":"")<<state(i);out<<"]";
    };
    vector("start",start_);vector("goal",goal_);
    if(result){
      out<<",\"expanded\":"<<result->expanded<<",\"sample_count\":"<<result->samples.size();
      if(!result->samples.empty()){
        out<<",\"duration\":"<<result->samples.back().time;vector("endpoint",result->samples.back().state);
      }
    }
    out<<"}";
    std_msgs::msg::String message;message.data=out.str();status_pub_->publish(message);
  }
  void tick() {
    if(worker_.valid() && worker_.wait_for(0ms)==std::future_status::ready){
      try{
        const auto completed=worker_.get();
        if(completed.request.id==generation_)accept(completed);
      }catch(const std::exception& error){
        clearPath();label_="SEARCH ERROR";publishStatus("error",nullptr,0);
        RCLCPP_ERROR(get_logger(),"%s",error.what());
      }
    }
    if(pending_ && !worker_.valid()){
      const Request request{generation_,start_,goal_};pending_=false;
      const auto config=config_;const double extent=extent_,radius=radius_;
      worker_=std::async(std::launch::async,[request,config,extent,radius]{
        const auto begin=std::chrono::steady_clock::now();
        const auto in_water=[extent,radius](const frontend::State& s){
          return s.allFinite() && std::abs(s(0))+radius<extent && std::abs(s(1))+radius<extent;
        };
        auto result=frontend::KinodynamicSearch(config).search(request.start,request.goal,in_water);
        return Completed{request,std::move(result),std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()};
      });
    }
    publishMarkers();
  }
  void accept(const Completed& completed) {
    current_=completed.result;preview_started_=std::chrono::steady_clock::now();
    const char* status=current_.status==frontend::SearchResult::REACH_END?"reach_end":
      current_.status==frontend::SearchResult::REACH_HORIZON?"reach_horizon":"no_path";
    label_=current_.status==frontend::SearchResult::REACH_END?"GOAL REACHED (MODEL)":
      current_.status==frontend::SearchResult::REACH_HORIZON?"PARTIAL HORIZON":"NO PATH";
    if(!current_.samples.empty()){
      nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
      std_msgs::msg::Float64MultiArray states;
      states.layout.dim.resize(2);
      states.layout.dim[0].label="samples";states.layout.dim[0].size=current_.samples.size();
      states.layout.dim[0].stride=current_.samples.size()*9;
      states.layout.dim[1].label="time,x,y,psi,u,v,r,v_cmd,w_cmd";
      states.layout.dim[1].size=9;states.layout.dim[1].stride=9;
      for(const auto& sample:current_.samples){
        geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose=poseOf(sample.state);
        path.poses.push_back(p);states.data.push_back(sample.time);
        for(int i=0;i<6;++i)states.data.push_back(sample.state(i));
        states.data.push_back(sample.command(0));states.data.push_back(sample.command(1));
      }
      path_pub_->publish(path);samples_pub_->publish(states);
    }
    publishStatus(status,&current_,completed.elapsed);
    RCLCPP_INFO(get_logger(),"%s: goal=(%.2f,%.2f,%.2f), samples=%zu, expanded=%d, search=%.3f s",
      status,goal_(0),goal_(1),goal_(2),current_.samples.size(),current_.expanded,completed.elapsed);
  }
  void publishMarkers() {
    using Marker=visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;clear.header.frame_id=frame_;clear.header.stamp=now();clear.action=Marker::DELETEALL;array.markers.push_back(clear);
    const auto marker=[&](const std::string& ns,int id,int type){
      Marker m;m.header.frame_id=frame_;m.header.stamp=now();m.ns=ns;m.id=id;m.type=type;
      m.action=Marker::ADD;m.pose.orientation.w=1;m.color.a=1;return m;
    };
    auto water=marker("open_water",0,Marker::CUBE);
    water.pose.position.z=-0.08;water.scale.x=extent_*2;water.scale.y=extent_*2;water.scale.z=0.02;
    water.color.r=0.025;water.color.g=0.12;water.color.b=0.19;array.markers.push_back(water);
    const auto arrow=[&](const frontend::State& state,int id,double red,double green,double blue,double length){
      auto m=marker("body_heading",id,Marker::ARROW);m.pose=poseOf(state,0.15);
      m.scale.x=length;m.scale.y=length*0.20;m.scale.z=length*0.22;
      m.color.r=red;m.color.g=green;m.color.b=blue;array.markers.push_back(m);
    };
    arrow(start_,0,0.2,1.0,0.4,1.1);arrow(goal_,1,1.0,0.3,0.25,1.1);
    if(!current_.samples.empty()){
      const size_t stride=std::max<size_t>(1,current_.samples.size()/18);
      for(size_t i=0;i<current_.samples.size();i+=stride)arrow(current_.samples[i].state,2+i,1,0.75,0.15,0.45);
      if(playback_){
        const double duration=current_.samples.back().time;
        const double t=duration>0?std::fmod(std::chrono::duration<double>(std::chrono::steady_clock::now()-preview_started_).count(),duration+1.0):0;
        const auto it=std::lower_bound(current_.samples.begin(),current_.samples.end(),std::min(t,duration),
          [](const frontend::Sample& sample,double time){return sample.time<time;});
        arrow((it==current_.samples.end()?current_.samples.back():*it).state,999999,0.1,0.8,1.0,1.25);
      }
    }
    auto text=marker("instructions",0,Marker::TEXT_VIEW_FACING);
    text.pose.position.x=3.5;text.pose.position.y=6;text.pose.position.z=0.3;text.scale.z=0.32;
    text.color.r=text.color.g=text.color.b=0.95;
    text.text="OPEN WATER / KINODYNAMIC A*\n"+label_+
      "\nGreen: start   Red: goal   Cyan: model preview\nOrange arrows: BODY heading\n2D Pose Estimate: start   2D Goal Pose: goal\nPreview only / no controller";
    array.markers.push_back(text);marker_pub_->publish(array);
  }
  frontend::SearchConfig config_;
  frontend::State start_,goal_;
  frontend::SearchResult current_;
  std::string frame_,label_;
  double extent_,radius_;
  bool playback_=true,pending_=false;
  uint64_t generation_=0;
  std::future<Completed> worker_;
  std::chrono::steady_clock::time_point preview_started_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> fixed_frame_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr samples_pub_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<OpenWaterNode>());}
  catch(const std::exception& error){std::cerr<<error.what()<<"\n";rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
