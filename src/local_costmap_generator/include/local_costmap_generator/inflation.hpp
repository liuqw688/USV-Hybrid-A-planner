#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace local_costmap_generator {
struct InflationParams {
  int occupied_threshold=90, high_cost_threshold=50, minimum_cost=1, obstacle_lethal_threshold=85;
  double robot_radius=3, inflation_radius=15, decay_factor=3;
};
struct InflationResult {std::vector<int8_t> cost, obstacles; int shore_components=0, isolated_components=0;};
// Islands/isolated obstacles are enclosed occupied components. Border-connected
// land is shoreline. A cropped island touching the border keeps the hard buffer;
// without source semantics it cannot be distinguished from the shore.
inline InflationResult inflate(const std::vector<int8_t>& input,int w,int h,double resolution,
  const std::vector<uint8_t>& channel,const InflationParams& p) {
  InflationResult out;out.cost.assign(input.size(),0);out.obstacles.assign(input.size(),0);
  if(w<=0 || h<=0 || resolution<=0 || input.size()!=size_t(w)*h)return out;
  const bool has_channel=channel.size()==input.size();
  const int dx[8]={-1,0,1,-1,1,-1,0,1},dy[8]={-1,-1,-1,0,0,1,1,1};
  std::vector<uint8_t> seen(input.size(),0),isolated(input.size(),0);
  for(int seed=0;seed<w*h;++seed) {
    if(seen[seed] || input[seed]<p.occupied_threshold)continue;
    std::vector<int> component{seed};seen[seed]=1;bool border=false;
    for(size_t k=0;k<component.size();++k) {
      int id=component[k],x=id%w,y=id/w; border=border || x==0 || y==0 || x==w-1 || y==h-1;
      for(int d=0;d<8;++d) {int nx=x+dx[d],ny=y+dy[d];if(nx<0||ny<0||nx>=w||ny>=h)continue;
        int next=ny*w+nx;if(!seen[next] && input[next]>=p.occupied_threshold){seen[next]=1;component.push_back(next);}}
    }
    if(border)++out.shore_components;else {++out.isolated_components;for(int id:component)isolated[id]=1;}
  }
  struct Cell {float d;int id,cost;bool operator<(const Cell& b)const{return d>b.d;}};
  const auto field=[&](int type,double max_distance,std::vector<int>& source_cost) {
    std::vector<float> dist(input.size(),std::numeric_limits<float>::infinity());
    source_cost.assign(input.size(),0);std::priority_queue<Cell> queue;
    for(int id=0;id<w*h;++id) {
      const bool hard=input[id]>=p.occupied_threshold;
      const bool seed=type==0 ? hard : type==1 ? bool(isolated[id]):
        input[id]>=p.high_cost_threshold && !hard && !(has_channel && channel[id]);
      if(seed){dist[id]=0;source_cost[id]=type==2?input[id]:100;queue.push({0,id,source_cost[id]});}
    }
    while(!queue.empty()) {
      auto cur=queue.top();queue.pop();if(cur.d>dist[cur.id] || cur.d>max_distance)continue;
      int x=cur.id%w,y=cur.id/w;
      for(int d=0;d<8;++d) {int nx=x+dx[d],ny=y+dy[d];if(nx<0||ny<0||nx>=w||ny>=h)continue;
        int next=ny*w+nx;float nd=cur.d+float(resolution)*(dx[d] && dy[d]?1.41421356F:1.F);
        if(nd<=max_distance && nd<dist[next]){dist[next]=nd;source_cost[next]=cur.cost;queue.push({nd,next,cur.cost});}}
    }
    return dist;
  };
  std::vector<int> sources;
  const double radius=std::max(0.,p.robot_radius),soft=std::max(0.,p.inflation_radius);
  const auto hard_dist=field(0,radius+soft,sources);
  const auto obstacle_dist=field(1,radius+soft,sources);
  std::vector<int> high_sources;const auto high_dist=field(2,soft,high_sources);
  const auto hard_cost=[&](double d) {
    if(!std::isfinite(d) || d>radius+soft)return 0;
    if(d<=radius)return 99;
    if(soft<=0)return 0;
    return std::clamp(int(std::lround(p.minimum_cost+(98.-p.minimum_cost)*
      std::exp(-p.decay_factor*(d-radius)/soft))),0,98);
  };
  for(size_t id=0;id<input.size();++id) {
    if(input[id]<0){out.cost[id]=-1;out.obstacles[id]=-1;continue;}
    if(input[id]>=p.occupied_threshold){out.cost[id]=100;out.obstacles[id]=isolated[id]?100:0;continue;}
    const int obstacle=hard_cost(obstacle_dist[id]);out.obstacles[id]=int8_t(obstacle);
    const bool in_channel=has_channel && channel[id];
    // Hard clearance around every obstacle remains everywhere. Only the long
    // shoreline/high-region SOFT gradient is clipped by the channel polygons.
    int value=in_channel ? (hard_dist[id]<=radius ? 99:obstacle):hard_cost(hard_dist[id]);
    if(!in_channel && std::isfinite(high_dist[id])) {
      const double norm=soft>0 ? high_dist[id]/soft:0;
      value=std::max(value,int(std::lround(p.minimum_cost+(high_sources[id]-p.minimum_cost)*std::exp(-p.decay_factor*norm))));
    }
    // Match Hybrid/MPC's independent-obstacle lethal threshold in the merged
    // grid as well, so global A* cannot choose an unusable intermediate goal.
    if(obstacle>=p.obstacle_lethal_threshold)value=99;
    out.cost[id]=int8_t(std::clamp(value,0,100));
  }
  return out;
}
}
