#include <usv_kinodynamic_frontend/waterway_map.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
using namespace usv_kinodynamic_frontend;
int main(int argc,char** argv) {
  try {
    State start=State::Zero(),goal=State::Zero();goal(0)=2;
    SearchConfig config;
    WaterwayConfig waterway;
    std::string output;
    for(int i=1;i<argc;++i) {
      const std::string option=argv[i];
      if (option=="--help") {
        std::cout<<"usv_frontend_demo [--output result.json] [--goal-x 2] [--goal-y 0] [--goal-yaw 0] "
                   "[--max-search-time 0.8] [--hull-radius 0.5]\nNo ROS topics or actuators are used.\n";
        return 0;
      }
      if(i+1>=argc) throw std::invalid_argument("option requires a value");
      const std::string value=argv[++i];
      if(option=="--output")output=value;
      else if(option=="--goal-x")goal(0)=std::stod(value);
      else if(option=="--goal-y")goal(1)=std::stod(value);
      else if(option=="--goal-yaw")goal(2)=std::stod(value);
      else if(option=="--max-search-time")config.max_search_time=std::stod(value);
      else if(option=="--hull-radius")waterway.hull_radius=std::stod(value);
      else throw std::invalid_argument("unknown option: "+option);
    }
    WaterwayMap map;
    map.frame_id="map";map.width=160;map.height=120;map.resolution=0.25;
    map.origin_x=-20;map.origin_y=-15;map.cells.assign(map.width*map.height,0);
    const auto result=WaterwayPlanner(config,waterway).plan(map,start,goal);
    std::ofstream file;
    if(!output.empty()){file.open(output);if(!file)throw std::runtime_error("cannot open result file");}
    std::ostream& out=output.empty()?std::cout:file;
    out.imbue(std::locale::classic());out<<std::setprecision(15);
    const char* status=result.status==SearchResult::REACH_END ? "reach_end" :
      result.status==SearchResult::REACH_HORIZON ? "reach_horizon" : "no_path";
    out<<"{\n  \"model\": \"identified_velocity_command_11_coefficients\",\n"
       <<"  \"status\": \""<<status<<"\",\n  \"expanded\": "<<result.expanded
       <<",\n  \"cost\": "<<result.cost<<",\n  \"samples\": [\n";
    for(size_t i=0;i<result.samples.size();++i){
      const auto& s=result.samples[i];
      out<<"    {\"time\":"<<s.time<<",\"state\":[";
      for(int j=0;j<6;++j)out<<(j?",":"")<<s.state(j);
      out<<"],\"command\":["<<s.command(0)<<","<<s.command(1)<<"]}"<<(i+1<result.samples.size()?",":"")<<"\n";
    }
    out<<"  ]\n}\n";
    if(!output.empty())std::cout<<"USV frontend "<<status<<": "<<result.samples.size()<<" samples -> "<<output<<"\n";
    return result.status==SearchResult::NO_PATH?2:0;
  }catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}
}
