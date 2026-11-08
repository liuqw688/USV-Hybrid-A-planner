#include "hybrid_a_star_planner/planner.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace hybrid_a_star_planner;
using namespace hybrid_a_star_planner::planning;
int main(int argc,char **argv) {
  if(argc<5) {std::cerr<<"output environment bearing_deg heading_deg [u v r budget optimize]\n";return 2;}
  const std::string stem=argv[1],env=argv[2];
  const double bearing=std::stod(argv[3])*M_PI/180,heading=normalizeAngle(std::stod(argv[4])*M_PI/180);
  const double u=argc>5?std::stod(argv[5]):4,v=argc>6?std::stod(argv[6]):0,r=argc>7?std::stod(argv[7]):0;
  const double budget=argc>8?std::stod(argv[8]):3;const bool optimize=argc<=9 || std::stoi(argv[9]);
  // Match TF/getYaw before narrowing to the planner's float API. Unwrapped
  // angles otherwise acquire a different error at 45/120 degree thresholds.
  double gx=20*std::cos(bearing),gy=20*std::sin(bearing);
  if(std::abs(gx)<1e-9)gx=0;
  if(std::abs(gy)<1e-9)gy=0; // exact cardinal targets, not a spurious rear offset
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=240;map->info.height=200;map->info.resolution=1;
  map->info.origin.position.x=-120;map->info.origin.position.y=-100;
  map->info.origin.orientation.w=1;map->data.assign(48000,0);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  for(int y=0;y<200;++y) for(int x=0;x<240;++x) {
    const double wy=y-99.5;int role=45;
    if(std::abs(wy)<12) role=env=="opposite"?75:std::lround(20*std::abs(wy)/12);
    else if(wy>=12 && wy<36) role=env=="opposite"?10:75;
    lane->data[y*240+x]=role;
    if(env!="open" && (wy< -20 || wy>44))map->data[y*240+x]=100;
  }
  if(env=="chart_main" || env=="chart_opposite") {
    std::ifstream input(std::filesystem::path(stem).parent_path().parent_path()/"chart_grid.txt");
    int w,h;double x0,y0;if(!(input>>w>>h>>x0>>y0))return 3;
    std::vector<int> raw(w*h),role(w*h);for(int i=0;i<w*h;++i)input>>raw[i]>>role[i];
    const bool opposite=env=="chart_opposite";
    for(int y=0;y<200;++y) for(int x=0;x<240;++x) {
      const double lx=x-119.5,ly=y-99.5,wx=-20+(opposite?-lx:lx),wy=(opposite?85:70)+(opposite?-ly:ly);
      int ix=std::floor(wx-x0),iy=std::floor(wy-y0),i=y*240+x;
      if(ix<0 || iy<0 || ix>=w || iy>=h){map->data[i]=100;lane->data[i]=45;}
      else {map->data[i]=raw[iy*w+ix];lane->data[i]=role[iy*w+ix];}
    }
  }
  HybridAStarPlanner p;p.setParams(2,8,5,.5,.1,250000,88);
  p.setMotionModel("USV_DYNAMIC",.45);p.setUsvDynamics(dynamics::Parameters{});
  p.setDynamicParams(4,.5,1,120,3.5,7,16,12,.05,3);p.setDynamicHardDistance(3);
  p.setCostmap(map);p.setStaticLandMap(map);if(env!="open")p.setLaneCostmap(lane);
  p.setChannelParams(env!="open",88,85,20,65,12,18,4,2,5,18);
  p.setStabilityParams(0,12,1.5,12,24,6);p.setTurnaroundParams(2.0943951024,.25);
  p.setGentleTurnSampling(true);p.setChannelCurveParams(60,1,.6);
  p.setLargeTurnParams(true,2,5,1.57);p.setCloseTurnParams(15,1.2,3);
  p.setAdaptiveTurnParams(true);p.setFastGuidedConnection(true);
  p.setGoalHeadingTolerance(M_PI/6);p.setSearchTimeLimit(budget);
  p.startOverallSearchBudget();const auto started=std::chrono::steady_clock::now();
  auto raw=p.search(0,0,0,gx,gy,heading,0,u,v,r);
  auto path=optimize?p.optimizeTrackingPath(raw,heading):raw;
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  for(const auto &item:{std::make_pair("_raw",raw),std::make_pair("",path)}) {
    std::ofstream out(stem+item.first+".csv");out<<std::setprecision(12)<<"x,y,yaw\n";
    for(const auto &q:item.second)out<<q.position.x<<","<<q.position.y<<","<<tf2::getYaw(q.orientation)<<"\n";
  }
  if(p.usedShootingRescue()) {
    const auto& c=p.shootingControl();std::ofstream controls(stem+"_controls.json");
    controls<<std::setprecision(12)<<"{\"speed\":"<<c.speed<<",\"radius\":"<<c.radius<<",\"settle\":"<<c.settle<<",\"duration\":[";
    for(int i=0;i<3;++i){if(i)controls<<",";controls<<c.duration[i];}controls<<"],\"turn\":[";
    for(int i=0;i<3;++i){if(i)controls<<",";controls<<c.turn[i];}controls<<"]}";
    std::ofstream trace(stem+"_states.csv");trace<<"x,y,yaw,u,v,r\n";
    for(const auto& state:p.shootingTrace())trace<<std::setprecision(12)<<state.x<<","<<state.y<<","<<state.yaw<<","<<state.u<<","<<state.v<<","<<state.r<<"\n";
  }
  const auto q=groundPathQuality(path),qr=groundPathQuality(raw);
  bool safe=true;for(const auto &pose:resampleGroundPath(path,.25))safe=safe && !p.isLethal(pose.position.x,pose.position.y);
  const double position_error=path.empty()?-1:std::hypot(path.back().position.x-gx,path.back().position.y-gy);
  const double heading_error=path.empty()?-1:std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-heading));
  std::ofstream out(stem+".json");out<<std::setprecision(12);
  out<<"{\"success\":"<<(!path.empty()?"true":"false")<<",\"safe\":"<<(safe?"true":"false")
    <<",\"rescue\":"<<(p.usedShootingRescue()?"true":"false")<<",\"seconds\":"<<seconds<<",\"speed\":"<<p.planningSpeed()<<",\"length\":"<<q.length
    <<",\"reversals\":"<<q.reversals<<",\"peak\":"<<q.peak_curvature<<",\"rms\":"<<q.rms_curvature
    <<",\"raw_reversals\":"<<qr.reversals<<",\"raw_peak\":"<<qr.peak_curvature
    <<",\"position_error\":"<<position_error<<",\"heading_error\":"<<heading_error<<",\"attempted_speeds\":[";
  bool comma=false;for(double s:p.attemptedSpeeds()){if(comma)out<<",";out<<s;comma=true;}out<<"]}";
}
