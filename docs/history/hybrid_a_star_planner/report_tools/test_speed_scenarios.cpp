#define main planner_entry_unused
#ifdef USV_BASELINE
#include USV_BASELINE_SOURCE
#else
#include "hybrid_a_star_planner/planner.hpp"
#endif
#undef main
using hybrid_a_star_planner::planning::HybridAStarPlanner;
#include <fstream>
#include <iostream>
#include <cstdlib>

int main() {
  const std::string root=std::getenv("USV_REPORT_ROOT") ? std::getenv("USV_REPORT_ROOT") : "artifacts/hybrid_speed_update";
  const std::string out=std::getenv("USV_REPORT_OUTPUT") ? std::getenv("USV_REPORT_OUTPUT") : root+"/after";
  std::ofstream metrics(out+"/scenarios.csv");
  metrics<<"scenario,success,seconds,speed,length,outside_fraction,reverse_heading\n";
  int failures=0;
  for(const std::string name:{"chart_straight","chart_bend","open_bend","open_uturn","narrow_uturn","small_j_turn","blocked"}) {
    HybridAStarPlanner p;auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.resolution=1;map->info.origin.orientation.w=1;
    auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    double sx=0,sy=0,yaw=0,gx=70,gy=25.5,goal_yaw=.349;
    if(name.find("chart")==0) {
      std::ifstream f(root+"/chart_grid.txt");int w,h;double x,y;f>>w>>h>>x>>y;
      map->info.width=w;map->info.height=h;map->info.origin.position.x=x;map->info.origin.position.y=y;
      map->data.resize(w*h);*lane=*map;
      for(int i=0,a,b;i<w*h;++i) {f>>a>>b;map->data[i]=a;lane->data[i]=b;}
      if(name=="chart_straight") {sx=-20;sy=70;gx=120;gy=65;yaw=goal_yaw=-.05;}
      else {sx=70;sy=65;gx=200;gy=80;yaw=-.05;goal_yaw=.1;}
    } else {
      map->info.width=200;map->info.height=name=="narrow_uturn" ? 28:160;
      map->info.origin.position.x=-100;map->info.origin.position.y=-.5*map->info.height;
      map->data.assign(map->info.width*map->info.height,0);*lane=*map;
      for(unsigned y=0;y<map->info.height;++y) for(unsigned x=0;x<map->info.width;++x) {
        double wx=x-99.5,wy=y+map->info.origin.position.y+.5;
        double offset=std::abs(wy-std::tan(.349)*std::max(0.,wx));
        lane->data[y*map->info.width+x]=offset<12 ? std::lround(20*offset/12):45;
      }
      if(name=="open_uturn" || name=="narrow_uturn") {gx=-30;gy=10;goal_yaw=M_PI;}
      if(name=="small_j_turn") {gx=45;gy=12;goal_yaw=.35;}
      if(name=="blocked") for(unsigned y=0;y<map->info.height;++y) map->data[y*200+110]=100;
    }
    p.setCostmap(map);p.setLaneCostmap(lane);p.setStaticLandMap(map);
    p.setParams(2,8,5,.5,.1,250000,88);p.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
    p.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
    p.setLargeTurnParams(true,2,5,1.57);p.setSearchTimeLimit(3);p.setFastGuidedConnection(true);
#ifdef USV_BASELINE
    p.setChannelParams(true,88,85,20,65,2.5,4,2,2,5,18);
#else
    p.setChannelParams(true,88,85,20,65,12,18,2,2,5,18);
#endif
    p.setStabilityParams(0,12,1.5,12,24,6);
    auto begin=std::chrono::steady_clock::now();auto path=p.search(sx,sy,yaw,gx,gy,goal_yaw,0,4);
    double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    double length=0,outside=0,left=0,right=0;
    std::ofstream csv(out+"/"+name+".csv");csv<<"x,y,yaw\n";
    for(std::size_t i=0;i<path.size();++i) {
      auto &q=path[i];csv<<q.position.x<<","<<q.position.y<<","<<tf2::getYaw(q.orientation)<<"\n";
      if(i) {double ds=std::hypot(q.position.x-path[i-1].position.x,q.position.y-path[i-1].position.y);length+=ds;
        int x=std::floor(q.position.x-map->info.origin.position.x),y=std::floor(q.position.y-map->info.origin.position.y);
        if(x<0 || y<0 || x>=int(map->info.width) || y>=int(map->info.height)) ++failures;
        else {if(map->data[y*map->info.width+x]>=88) ++failures;if(lane->data[y*map->info.width+x]>20) outside+=ds;}
        double d=hybrid_a_star_planner::normalizeAngle(tf2::getYaw(q.orientation)-tf2::getYaw(path[i-1].orientation));
        left+=std::max(0.,d);right+=std::max(0.,-d);
      }
    }
    bool okay=name=="blocked" ? path.empty():!path.empty();
    if(!path.empty()) okay=okay && std::hypot(path.back().position.x-gx,path.back().position.y-gy)<1 &&
      std::abs(hybrid_a_star_planner::normalizeAngle(tf2::getYaw(path.back().orientation)-goal_yaw))<.22;
    if(!okay) ++failures;
    metrics<<name<<","<<!path.empty()<<","<<seconds<<","<<p.planningSpeed()<<","<<length<<","<<outside/std::max(1.,length)<<","<<std::min(left,right)<<"\n";
    std::cout<<name<<" success="<<!path.empty()<<" time="<<seconds<<" speed="<<p.planningSpeed()<<" outside="<<outside/std::max(1.,length)<<std::endl;
  }
  return failures ? 1:0;
}
