#include "local_costmap_generator/inflation.hpp"
#include <gtest/gtest.h>
#include <iostream>
using namespace local_costmap_generator;
TEST(Inflation,SourceSeparationAndHardSafety){
 const int w=100,h=60;std::vector<int8_t> raw(w*h,0);std::vector<uint8_t> lanes(w*h,0);
 for(int y=0;y<h;++y)for(int x=0;x<w;++x){if(y<5)raw[y*w+x]=100;if(y>=8&&y<40)lanes[y*w+x]=1;}
 InflationParams p;p.robot_radius=2;p.inflation_radius=15;
 auto a=inflate(raw,w,h,1,lanes,p),legacy=inflate(raw,w,h,1,{},p);
 EXPECT_TRUE(a.cost[10*w+40]==0);EXPECT_TRUE(legacy.cost[10*w+40]>0);
 EXPECT_TRUE(a.cost[6*w+40]==99);EXPECT_TRUE(a.cost[7*w+40]>0);EXPECT_TRUE(a.cost[4*w+40]==100);
 EXPECT_TRUE(a.cost[40*w+40]==0);EXPECT_TRUE(a.shore_components==1 && a.isolated_components==0);
 raw[25*w+50]=100;auto island=inflate(raw,w,h,1,lanes,p);
 EXPECT_TRUE(island.cost[25*w+50]==100 && island.obstacles[25*w+50]==100);
 EXPECT_TRUE(island.cost[25*w+52]==99 && island.obstacles[25*w+52]==99);
 EXPECT_TRUE(island.cost[25*w+56]>0 && island.obstacles[25*w+56]>0);
 for(size_t id=0;id<island.cost.size();++id) {
   if(island.obstacles[id]>=p.obstacle_lethal_threshold) {EXPECT_TRUE(island.cost[id]>=99);}
 }
 raw[20*w+20]=-1;raw[20*w+30]=50;auto b=inflate(raw,w,h,1,lanes,p);
 EXPECT_TRUE(b.cost[20*w+20]==-1 && b.obstacles[20*w+20]==-1);
 EXPECT_TRUE(b.cost[20*w+30]==0);EXPECT_TRUE(b.cost[20*w+32]==0);
 p.inflation_radius=0;auto c=inflate(raw,w,h,1,lanes,p);EXPECT_TRUE(c.cost[25*w+52]==99 && c.cost[25*w+53]==0);
 auto fallback=inflate(raw,w,h,1,{},InflationParams{});EXPECT_TRUE(fallback.cost[10*w+40]>0);
 std::cout<<"PASS: shore soft clipping, outside inflation, minimum hard clearance, preserved raw100, island independent100/99/soft, UNKNOWN, false center seed, zero radius, conservative fallback\n";
}
