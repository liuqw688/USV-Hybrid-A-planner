#pragma once
#include <usv_kinodynamic_frontend/kinodynamic_astar.hpp>
#include <cstdint>
#include <limits>
#include <string>

namespace usv_kinodynamic_frontend {
// No assumed TF: all states and this grid must already share frame_id.
// A rotated grid origin is supported. Heights are intentionally absent.
struct WaterwayMap {
  std::string frame_id;
  int width=0,height=0;
  double resolution=0,origin_x=0,origin_y=0,origin_yaw=0;
  std::vector<int8_t> cells;
  void validate() const {
    if (width<=0 || height<=0 || !std::isfinite(resolution) || resolution<=0 ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y) || !std::isfinite(origin_yaw) ||
        cells.size()!=static_cast<size_t>(width)*static_cast<size_t>(height))
      throw std::invalid_argument("invalid waterway occupancy grid");
    for (int cell:cells) if (cell < -1 || cell >100)
      throw std::invalid_argument("occupancy grid values must be in [-1,100]");
  }
  Eigen::Vector2d local(double x,double y) const {
    const double c=std::cos(origin_yaw),s=std::sin(origin_yaw);
    return Eigen::Vector2d(c*(x-origin_x)+s*(y-origin_y),-s*(x-origin_x)+c*(y-origin_y));
  }
  bool footprintFree(const State& state,double radius,int lethal=88,bool unknown_is_obstacle=true) const {
    if (!state.allFinite() || !std::isfinite(radius) || radius<=0 || lethal<1 || lethal>100) return false;
    const Eigen::Vector2d p=local(state(0),state(1));
    if (p(0)-radius<0 || p(1)-radius<0 || p(0)+radius>=width*resolution || p(1)+radius>=height*resolution)
      return false; // full hull must stay in observed grid bounds
    const int xmin=static_cast<int>(std::floor((p(0)-radius)/resolution));
    const int xmax=static_cast<int>(std::floor((p(0)+radius)/resolution));
    const int ymin=static_cast<int>(std::floor((p(1)-radius)/resolution));
    const int ymax=static_cast<int>(std::floor((p(1)+radius)/resolution));
    for(int y=ymin;y<=ymax;++y) for(int x=xmin;x<=xmax;++x) {
      // Exact circle-square intersection, including occupied cells inside hull.
      const double dx=std::max({x*resolution-p(0),0.0,p(0)-(x+1)*resolution});
      const double dy=std::max({y*resolution-p(1),0.0,p(1)-(y+1)*resolution});
      if (dx*dx+dy*dy>radius*radius) continue;
      const int cost=cells[static_cast<size_t>(y)*width+x];
      if (cost>=lethal || (cost<0 && unknown_is_obstacle)) return false;
    }
    return true;
  }
  double softCost(const State& state,int lethal=88) const {
    const auto p=local(state(0),state(1));
    if (!p.allFinite() || p(0)<0 || p(1)<0 || p(0)>=width*resolution || p(1)>=height*resolution)
      return std::numeric_limits<double>::infinity();
    const int x=static_cast<int>(std::floor(p(0)/resolution));
    const int y=static_cast<int>(std::floor(p(1)/resolution));
    return std::max(0,static_cast<int>(cells[static_cast<size_t>(y)*width+x]))/static_cast<double>(lethal);
  }
};
struct WaterwayConfig {
  double hull_radius=0.5,clearance=0.2,shore_cost_weight=1.0;
  int lethal_threshold=88;
  bool unknown_is_obstacle=true;
  void validate() const {
    if (!std::isfinite(hull_radius) || hull_radius<=0 || !std::isfinite(clearance) || clearance<0 ||
        !std::isfinite(shore_cost_weight) || shore_cost_weight<0 || lethal_threshold<1 || lethal_threshold>100)
      throw std::invalid_argument("invalid waterway collision/cost parameters");
  }
};
class WaterwayPlanner {
 public:
  WaterwayPlanner(SearchConfig search={},WaterwayConfig waterway={}) : search_(search),waterway_(waterway) {
    search_.validate();waterway_.validate();
  }
  SearchResult plan(const WaterwayMap& map,const State& start,const State& goal) const {
    map.validate();
    SearchConfig config=search_;
    // Keep collision samples below half a grid cell at maximum actual speed.
    config.collision_step=std::min(config.collision_step,0.5*map.resolution/
      std::hypot(config.limits.max_u,config.limits.max_v));
    const auto free=[&](const State& state){return map.footprintFree(state,
      waterway_.hull_radius+waterway_.clearance,waterway_.lethal_threshold,waterway_.unknown_is_obstacle);};
    const auto cost=[&](const State& state){return waterway_.shore_cost_weight*map.softCost(state,waterway_.lethal_threshold);};
    return KinodynamicSearch(config).search(start,goal,free,cost);
  }
 private:
  SearchConfig search_;
  WaterwayConfig waterway_;
};
}  // namespace usv_kinodynamic_frontend
