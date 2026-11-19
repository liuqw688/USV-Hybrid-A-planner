#include "hybrid_a_star_planner/planner.hpp"
#include <gtest/gtest.h>
using namespace hybrid_a_star_planner;
using namespace hybrid_a_star_planner::planning;
namespace {
geometry_msgs::msg::Pose pose(double x,double y,double yaw) {
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;
  tf2::Quaternion q;q.setRPY(0,0,yaw);p.orientation=tf2::toMsg(q);return p;
}
std::vector<geometry_msgs::msg::Pose> wavyTurn(double amplitude=1.2) {
  std::vector<geometry_msgs::msg::Pose> path;
  for(int i=0;i<=240;++i) {
    const double a=M_PI*i/240;
    path.push_back(pose(4*std::sin(a),4*(1-std::cos(a)),a));
  }
  for(int i=1;i<=400;++i) {
    const double x=20.0*i/400,t=2*M_PI*x/10;
    const double y=8+amplitude*std::pow(std::sin(t),3);
    const double slope=amplitude*3*std::pow(std::sin(t),2)*std::cos(t)*2*M_PI/10;
    path.push_back(pose(-x,y,std::atan2(slope,-1.0)));
  }
  return path;
}
nav_msgs::msg::OccupancyGrid::SharedPtr emptyMap() {
  auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->info.width=240;m->info.height=200;m->info.resolution=1;
  m->info.origin.position.x=-120;m->info.origin.position.y=-100;
  m->info.origin.orientation.w=1;m->data.assign(48000,0);return m;
}
HybridAStarPlanner planner(const nav_msgs::msg::OccupancyGrid::SharedPtr &m) {
  HybridAStarPlanner p;p.setCostmap(m);p.setStaticLandMap(m);
  p.setParams(2,8,5,.5,.1,250000,88);p.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  return p;
}
}
TEST(GroundCurvature, CircleIsMeasuredIndependentlyOfBodyHeadingAndPointDensity) {
  std::vector<geometry_msgs::msg::Pose> path;
  for(int i=0;i<=600;++i) {double a=M_PI*i/600;path.push_back(pose(10*std::sin(a),10*(1-std::cos(a)),0));}
  auto q=groundPathQuality(path);
  EXPECT_EQ(q.reversals,0);EXPECT_NEAR(q.peak_curvature,.1,.002);
  EXPECT_NEAR(q.rms_curvature,.1,.002);
  auto sparse=groundPathQuality(resampleGroundPath(path,.2));
  EXPECT_NEAR(sparse.peak_curvature,q.peak_curvature,.015);
}
TEST(GeometricReference, RemovesTailOscillationsAndReducesMaximumCurvature) {
  auto p=planner(emptyMap());auto raw=wavyTurn();auto before=groundPathQuality(raw);
  ASSERT_GE(before.reversals,3);
  auto result=p.optimizeTrackingPath(raw,M_PI,8,12,.5);auto after=groundPathQuality(result);
  EXPECT_EQ(after.reversals,0);
  EXPECT_LT(after.peak_curvature,before.peak_curvature*.8);
  EXPECT_LE(after.rms_curvature,before.rms_curvature);
  EXPECT_DOUBLE_EQ(result.front().position.x,raw.front().position.x);
  EXPECT_DOUBLE_EQ(result.front().position.y,raw.front().position.y);
  EXPECT_DOUBLE_EQ(tf2::getYaw(result.front().orientation),tf2::getYaw(raw.front().orientation));
  EXPECT_DOUBLE_EQ(result.back().position.x,raw.back().position.x);
  EXPECT_DOUBLE_EQ(result.back().position.y,raw.back().position.y);
  EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(result.back().orientation)-M_PI)),M_PI/6);
}
TEST(GeometricReference, WidensAnAlreadySingleTightTurn) {
  auto p=planner(emptyMap());auto raw=wavyTurn(0);auto result=p.optimizeTrackingPath(raw,M_PI,8,12,.5);
  EXPECT_EQ(groundPathQuality(result).reversals,0);
  EXPECT_LT(groundPathQuality(result).peak_curvature,groundPathQuality(raw).peak_curvature*.8);
}
TEST(GeometricReference, RetainsGentleNecessarySingleS) {
  auto p=planner(emptyMap());std::vector<geometry_msgs::msg::Pose> raw;
  for(int i=0;i<=200;++i) {double t=i/200.0;raw.push_back(pose(20*t,3*.5*(1-t)*(1-t)*t-3*.5*(1-t)*t*t,0));}
  ASSERT_LE(groundPathQuality(raw).reversals,1);
  auto result=p.optimizeTrackingPath(raw,0);
  ASSERT_EQ(result.size(),raw.size());
  for(std::size_t i=0;i<raw.size();++i) {EXPECT_DOUBLE_EQ(result[i].position.y,raw[i].position.y);}
}
TEST(GeometricReference, CannotCutThroughLandInAnOscillatingNarrowCorridor) {
  auto raw=wavyTurn();auto map=emptyMap();map->data.assign(48000,100);
  for(const auto &p:raw) {
    int x=std::floor(p.position.x+120),y=std::floor(p.position.y+100);
    map->data[y*240+x]=0;
  }
  auto p=planner(map);auto result=p.optimizeTrackingPath(raw,M_PI,8,12,.5);
  ASSERT_EQ(result.size(),raw.size());
  for(std::size_t i=0;i<raw.size();++i) {EXPECT_DOUBLE_EQ(result[i].position.y,raw[i].position.y);}
}
TEST(GeometricReference, RechecksStationaryTrafficAndSupportsCancellation) {
  auto p=planner(emptyMap());auto raw=wavyTurn();
  auto clear=p.optimizeTrackingPath(raw,M_PI,8,12,.5);
  DynamicObstacle obstacle;obstacle.state.x=clear[clear.size()/2].position.x;
  obstacle.state.y=clear[clear.size()/2].position.y;obstacle.state.radius=.5;
  p.setDynamicObstacles({obstacle});auto result=p.optimizeTrackingPath(raw,M_PI,8,12,.5);
  if(result.size()!=raw.size()) {
    EXPECT_FALSE(p.remainingPathNeedsReplan(result,0,0,0,2,8,false));
    EXPECT_LE(groundPathQuality(result).peak_curvature,groundPathQuality(raw).peak_curvature);
  }
  auto cancellation=std::make_shared<std::atomic<bool>>(true);p.setSearchCancellation(cancellation);
  EXPECT_EQ(p.optimizeTrackingPath(raw,M_PI,8,12,.5).size(),raw.size());
}
TEST(GeometricReference, TailReshapeRetainsMainTurnWhenWiderArcHitsBanks) {
  auto raw=wavyTurn(.15);auto map=emptyMap();
  for(int y=0;y<200;++y) for(int x=0;x<240;++x) {
    if(y-100>=9 || y-100< -2) map->data[y*240+x]=100;
  }
  auto p=planner(map);const auto before=groundPathQuality(raw);
  ASSERT_GE(before.reversals,2);
  auto result=p.optimizeTrackingPath(raw,M_PI,8,12,.5);const auto after=groundPathQuality(result);
  EXPECT_LE(after.reversals,1);
  EXPECT_LE(after.peak_curvature,before.peak_curvature+1e-6);
  EXPECT_LT(after.rms_curvature,before.rms_curvature);
  ASSERT_GT(result.size(),100U);
  EXPECT_DOUBLE_EQ(result[100].position.x,raw[100].position.x);
  EXPECT_DOUBLE_EQ(result[100].position.y,raw[100].position.y);
  const auto prefix=dominantTurnEndIndex(raw);
  ASSERT_LT(prefix,result.size());
  const std::vector<geometry_msgs::msg::Pose> original_tail(raw.begin()+prefix,raw.end());
  const std::vector<geometry_msgs::msg::Pose> changed_tail(result.begin()+prefix,result.end());
  EXPECT_LE(groundPathQuality(changed_tail).peak_curvature,
    groundPathQuality(original_tail).peak_curvature+1e-6);
  for(const auto &q:result) EXPECT_FALSE(p.isLethal(q.position.x,q.position.y));
}
