#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include "local_costmap_generator/inflation.hpp"

class LocalCostmapNode : public rclcpp::Node {
public:
  LocalCostmapNode():Node("local_costmap_node") {
    const auto input=declare_parameter("input_topic","/local_obstacle_map");
    const auto output=declare_parameter("output_topic","/local_costmap");
    const auto lanes=declare_parameter("lane_data_topic","/chart_lane_data");
    const auto obstacles=declare_parameter("obstacle_output_topic","/channel/obstacle_costmap");
    clip_=declare_parameter("clip_shore_soft_in_channel",true);
    p_.occupied_threshold=declare_parameter("occupied_threshold",90);
    p_.high_cost_threshold=declare_parameter("high_cost_threshold",60);
    p_.robot_radius=declare_parameter("robot_radius",5.0);
    p_.inflation_radius=declare_parameter("inflation_radius",70.0);
    p_.minimum_cost=declare_parameter("minimum_cost",1);
    p_.obstacle_lethal_threshold=declare_parameter("obstacle_lethal_threshold",85);
    p_.decay_factor=declare_parameter("decay_factor",3.0);
    if(!std::isfinite(p_.robot_radius) || !std::isfinite(p_.inflation_radius) || !std::isfinite(p_.decay_factor) || p_.robot_radius<0 || p_.inflation_radius<0 || p_.decay_factor<0 ||
      p_.high_cost_threshold<1 || p_.occupied_threshold<=p_.high_cost_threshold || p_.occupied_threshold>100 ||
      p_.minimum_cost<0 || p_.minimum_cost>98 || p_.obstacle_lethal_threshold<1 || p_.obstacle_lethal_threshold>99)throw std::invalid_argument("Invalid inflation parameters");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    cost_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>(output,qos);
    obstacle_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>(obstacles,qos);
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(input,qos,[this](nav_msgs::msg::OccupancyGrid::SharedPtr m){
      if(!m->info.width || !m->info.height || !std::isfinite(m->info.resolution) || m->info.resolution<=0 ||
        m->data.size()!=size_t(m->info.width)*m->info.height){RCLCPP_WARN(get_logger(),"Invalid input grid");return;}
      map_=m;publish();});
    lane_sub_=create_subscription<std_msgs::msg::String>(lanes,qos,[this](std_msgs::msg::String::SharedPtr m){
      try {
        auto j=nlohmann::json::parse(m->data);std::vector<std::vector<Point>> polygons;
        for(const auto& l:j.at("lanes")) {std::vector<Point> poly;for(const auto& p:l.at("points")) {
          double x=p.at(0).get<double>(),y=p.at(1).get<double>();
          if(!std::isfinite(x)||!std::isfinite(y)) {throw std::invalid_argument("Non-finite lane point");}
          poly.push_back({x,y});}
          if(poly.size()>=3)polygons.push_back(std::move(poly));}
        polygons_=std::move(polygons);lane_frame_=j.value("frame_id",std::string("map"));publish();
      }catch(const std::exception& e){polygons_.clear();RCLCPP_WARN(get_logger(),"Invalid lane geometry: %s",e.what());publish();}});
  }
private:
  struct Point {double x,y;};
  std::vector<uint8_t> mask() {
    if(!clip_ || polygons_.empty())return {};
    if(lane_frame_!=map_->header.frame_id) {
      RCLCPP_WARN(get_logger(),"Lane/map frames differ; keeping full conservative inflation (%s / %s)",lane_frame_.c_str(),map_->header.frame_id.c_str());return {};}
    const int w=map_->info.width,h=map_->info.height;const double r=map_->info.resolution;
    const auto& q=map_->info.origin.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),c=std::cos(yaw),s=std::sin(yaw);
    std::vector<uint8_t> result(map_->data.size(),0);
    for(const auto& world:polygons_) {
      std::vector<Point> poly;double minx=w,miny=h,maxx=0,maxy=0;
      for(auto p:world){double dx=p.x-map_->info.origin.position.x,dy=p.y-map_->info.origin.position.y;
        Point a{(c*dx+s*dy)/r,(-s*dx+c*dy)/r};poly.push_back(a);
        minx=std::min(minx,a.x);maxx=std::max(maxx,a.x);miny=std::min(miny,a.y);maxy=std::max(maxy,a.y);}
      // Scanline fill by cell centers; work is proportional to covered rows,
      // not all cells times the number of chart polygons.
      for(int y=std::max(0,int(std::floor(miny)));y<std::min(h,int(std::ceil(maxy)));++y) {
        std::vector<double> cuts;const double cy=y+.5;
        for(size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
          auto a=poly[j],b=poly[i];if((a.y>cy)!=(b.y>cy))cuts.push_back(a.x+(cy-a.y)*(b.x-a.x)/(b.y-a.y));}
        std::sort(cuts.begin(),cuts.end());
        for(size_t k=1;k<cuts.size();k+=2)for(int x=std::max(0,int(std::ceil(cuts[k-1]-.5)));x<std::min(w,int(std::ceil(cuts[k]-.5)));++x)result[y*w+x]=1;
      }
    }
    return result;
  }
  void publish() {
    if(!map_)return;
    auto result=local_costmap_generator::inflate(map_->data,map_->info.width,map_->info.height,map_->info.resolution,mask(),p_);
    auto cost=*map_,obstacles=*map_;cost.data=std::move(result.cost);obstacles.data=std::move(result.obstacles);
    cost_pub_->publish(cost);obstacle_pub_->publish(obstacles);
    RCLCPP_INFO(get_logger(),"Inflated grid: shoreline components=%d isolated obstacles=%d; hard clearance=%.1fm",
      result.shore_components,result.isolated_components,p_.robot_radius);
  }
  local_costmap_generator::InflationParams p_;bool clip_=true;
  std::string lane_frame_;std::vector<std::vector<Point>> polygons_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_pub_,obstacle_pub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalCostmapNode>());rclcpp::shutdown();}
