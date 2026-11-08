#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hybrid_a_star_planner::planning {
// Try the terminal-yaw offset that shortens the chosen entry turn first.
// Reflection changes both yaw and lane side; the preference must reflect too.
// This only orders samples inside the existing terminal tolerance.
inline double mainLaneHeadingOffsetPreference(double requested_turn,int lane_side) {
  if(!lane_side) return -1.0;
  if(std::abs(requested_turn)<1e-6) return -lane_side;
  const double yaw_sign=requested_turn>0 ? 1.0:-1.0;
  return yaw_sign*(requested_turn*lane_side>0 ? -1.0:1.0);
}

// Semantic distance, not a clearance map: obstacles and traffic are checked
// separately by the planner. A two-pass chamfer field costs O(map cells).
class MainLaneDistanceField {
public:
  void reset(const nav_msgs::msg::OccupancyGrid::SharedPtr &map,int main_max) {
    map_=map;distance_.clear();nearest_.clear();
    if(!map || map->info.resolution<=0 || !map->info.width || !map->info.height ||
      map->data.size()!=static_cast<std::size_t>(map->info.width)*map->info.height) return;
    const int w=map->info.width,h=map->info.height;
    distance_.assign(w*h,std::numeric_limits<float>::infinity());nearest_.assign(w*h,-1);
    for(int i=0;i<w*h;++i) if(map->data[i]>=0 && map->data[i]<=main_max) {
      distance_[i]=0;nearest_[i]=i;
    }
    auto relax=[&](int x,int y,int dx,int dy) {
      const int nx=x+dx,ny=y+dy;if(nx<0 || ny<0 || nx>=w || ny>=h) return;
      const int i=y*w+x,j=ny*w+nx;
      const float d=distance_[j]+map->info.resolution*(dx && dy ? std::sqrt(2.0):1.0);
      if(d<distance_[i]) {distance_[i]=d;nearest_[i]=nearest_[j];}
    };
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
      relax(x,y,-1,0);relax(x,y,0,-1);relax(x,y,-1,-1);relax(x,y,1,-1);
    }
    for(int y=h-1;y>=0;--y) for(int x=w-1;x>=0;--x) {
      relax(x,y,1,0);relax(x,y,0,1);relax(x,y,1,1);relax(x,y,-1,1);
    }
  }
  double distance(double x,double y) const {
    const int i=index(x,y);return i<0 ? std::numeric_limits<double>::infinity():distance_[i];
  }
  std::pair<double,double> direction(double x,double y) const {
    const int i=index(x,y);if(i<0 || nearest_[i]<0) return {0,0};
    const int seed=nearest_[i],w=map_->info.width;
    const double a=tf2::getYaw(map_->info.origin.orientation);
    const double lx=(seed%w+.5)*map_->info.resolution,ly=(seed/w+.5)*map_->info.resolution;
    return {map_->info.origin.position.x+lx*std::cos(a)-ly*std::sin(a)-x,
      map_->info.origin.position.y+lx*std::sin(a)+ly*std::cos(a)-y};
  }
private:
  int index(double x,double y) const {
    if(distance_.empty()) return -1;
    const double a=tf2::getYaw(map_->info.origin.orientation);
    const double dx=x-map_->info.origin.position.x,dy=y-map_->info.origin.position.y;
    const int ix=std::floor((dx*std::cos(a)+dy*std::sin(a))/map_->info.resolution);
    const int iy=std::floor((-dx*std::sin(a)+dy*std::cos(a))/map_->info.resolution);
    return ix<0 || iy<0 || ix>=static_cast<int>(map_->info.width) ||
      iy>=static_cast<int>(map_->info.height) ? -1:iy*map_->info.width+ix;
  }
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<float> distance_;
  std::vector<int> nearest_;
};
}
