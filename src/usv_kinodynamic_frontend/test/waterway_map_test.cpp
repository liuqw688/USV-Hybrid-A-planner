#include <usv_kinodynamic_frontend/occupancy_grid.hpp>
#include <iostream>
#include <stdexcept>
using namespace usv_kinodynamic_frontend;
static int checks=0;
static void require(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
int main(){
  try{
    WaterwayMap map;map.frame_id="map";map.width=80;map.height=60;map.resolution=0.25;
    map.origin_x=-10;map.origin_y=-7.5;map.cells.assign(map.width*map.height,0);map.validate();
    State state=State::Zero(),goal=state;goal(0)=2;
    require(map.footprintFree(state,0.5),"open water footprint free");
    auto costmap=map;
    const int center=30*80+40;
    for(int value:{-1,99,100}){
      costmap.cells[center]=value;
      require(!costmap.footprintFree(state,0.5),"unknown/shore hard buffer/land rejected");
    }
    costmap.cells[center]=-1;
    require(costmap.footprintFree(state,0.5,88,false),"explicit unknown-space policy");
    costmap=map;costmap.cells[center]=40;
    require(costmap.footprintFree(state,0.5),"soft shore gradient not converted to hard land");
    require(costmap.softCost(state)>0 && costmap.softCost(state)<1,"nonnegative normalized shore cost");
    costmap=map;costmap.cells[30*80+42]=100;
    require(!costmap.footprintFree(state,0.55),"hull edge collision even when center is free");
    state(0)=9.7;
    require(!map.footprintFree(state,0.5),"full hull cannot leave map boundary");
    state.setZero();state(0)=0.37;state(1)=0.12;
    const bool before=costmap.footprintFree(state,0.55);
    const double angle=0.7,c=std::cos(angle),s=std::sin(angle);
    WaterwayMap rotated=costmap;
    rotated.origin_x=3+c*costmap.origin_x-s*costmap.origin_y;
    rotated.origin_y=-2+s*costmap.origin_x+c*costmap.origin_y;
    rotated.origin_yaw=angle;
    State transformed=state;
    transformed(0)=3+c*state(0)-s*state(1);transformed(1)=-2+s*state(0)+c*state(1);
    require(rotated.footprintFree(transformed,0.55)==before,"rotated map/world translation preserves hull collision");
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id="map";message.info.width=map.width;message.info.height=map.height;
    message.info.resolution=map.resolution;message.info.origin.position.x=map.origin_x;
    message.info.origin.position.y=map.origin_y;message.info.origin.orientation.w=1;message.data=map.cells;
    auto converted=fromOccupancyGrid(message);
    require(converted.cells==map.cells && converted.frame_id=="map","existing OccupancyGrid conversion");
    message.info.origin.orientation.x=0.1;
    bool rejected=false;
    try{fromOccupancyGrid(message);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"reject non-planar map rather than assuming TF");
    const auto result=WaterwayPlanner().plan(map,State::Zero(),goal);
    require(result.status==SearchResult::REACH_END,"standalone waterway frontend plans successfully");
    for(const auto& sample:result.samples)require(map.footprintFree(sample.state,0.7),"all planned hull samples clear");
    State too_fast=State::Zero();too_fast(3)=4;
    require(WaterwayPlanner().plan(map,too_fast,goal).status==SearchResult::NO_PATH,
      "existing 4m/s state is rejected, never silently clamped to new model limits");
    costmap=map;costmap.cells[center]=100;
    require(WaterwayPlanner().plan(costmap,State::Zero(),goal).status==SearchResult::NO_PATH,"occupied hull prevents planning");
    SearchConfig config;config.max_search_time=0.5;
    KinodynamicSearch search(config);
    const auto free=[](const State&){return true;};
    require(search.search(State::Zero(),goal,free,[](const State&){return -1.;}).status==SearchResult::NO_PATH,
      "negative environment cost rejected");
    const auto plain=search.search(State::Zero(),goal,free);
    const auto penalized=search.search(State::Zero(),goal,free,[](const State&){return 2.;});
    require(penalized.status==SearchResult::REACH_END && penalized.cost>plain.cost,"shore penalty contributes to g score");
    std::cout<<"Waterway map/planner: "<<checks<<" checks passed\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}
  return 0;
}
