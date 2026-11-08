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


namespace {
nav_msgs::msg::OccupancyGrid::SharedPtr splitLane(double rotation=0) {
  auto lane=emptyMap();
  tf2::Quaternion q;q.setRPY(0,0,rotation);lane->info.origin.orientation=tf2::toMsg(q);
  // Keep a simple local map origin; rotation is tested separately below.
  for(int y=0;y<200;++y) for(int x=0;x<240;++x) lane->data[y*240+x]=y>=100 ? 5:75;
  return lane;
}
}
TEST(MainLaneGuidance, UsesSemanticRoleAndRespectsRotatedMapCoordinates) {
  auto map=splitLane();MainLaneDistanceField field;field.reset(map,20);
  EXPECT_NEAR(field.distance(0,-10),10,1);
  EXPECT_NEAR(field.distance(0,10),0,1e-6);
  EXPECT_GT(field.direction(0,-10).second,9);
  map->info.origin.position.x=100;map->info.origin.position.y=-120;
  tf2::Quaternion q;q.setRPY(0,0,M_PI/2);map->info.origin.orientation=tf2::toMsg(q);
  field.reset(map,20);
  // Local (120,90) transforms to world (10,0). MAIN lies world-left.
  EXPECT_NEAR(field.distance(10,0),10,1);
  EXPECT_LT(field.direction(10,0).first,-9);
  EXPECT_LE(std::abs(field.direction(10,0).second),.51);
}
TEST(MainLaneGuidance, MissingMainLaneAndUnknownCellsDoNotInventDirection) {
  MainLaneDistanceField field;auto map=emptyMap();map->data.assign(map->data.size(),75);
  field.reset(map,20);EXPECT_FALSE(std::isfinite(field.distance(0,0)));
  EXPECT_DOUBLE_EQ(field.direction(0,0).first,0);
  map->data.assign(map->data.size(),-1);field.reset(map,20);
  EXPECT_FALSE(std::isfinite(field.distance(0,0)));
  field.reset(nullptr,20);EXPECT_FALSE(std::isfinite(field.distance(0,0)));
}
TEST(MainLaneGuidance, EarlierEntryCostsLessThanAnAwayLoop) {
  auto p=planner(emptyMap());p.setLaneCostmap(splitLane());
  p.setChannelParams(true,88,85,20,65,12,18,4,2,5,18);
  std::vector<geometry_msgs::msg::Pose> direct{pose(0,-10,0),pose(5,5,0),pose(20,10,0)};
  std::vector<geometry_msgs::msg::Pose> away{pose(0,-10,0),pose(10,-25,0),pose(30,-25,0),pose(20,10,0)};
  EXPECT_LT(p.mainLaneEntryCost(direct),p.mainLaneEntryCost(away));
  p.setMainLaneEntryParams(0,30,30);EXPECT_DOUBLE_EQ(p.mainLaneEntryCost(away),0);
}
TEST(MainLaneGuidance, ShortMainToOppositeDestinationKeepsDirectCrossingException) {
  auto p=planner(emptyMap());p.setLaneCostmap(splitLane());
  p.setChannelParams(true,88,85,20,65,12,18,4,2,5,18);
  std::vector<geometry_msgs::msg::Pose> short_cross{pose(0,5,0),pose(10,-5,0)};
  EXPECT_DOUBLE_EQ(p.mainLaneEntryCost(short_cross),0);
  std::vector<geometry_msgs::msg::Pose> early_exit{pose(0,5,0),pose(5,-10,0),pose(60,-10,0)};
  std::vector<geometry_msgs::msg::Pose> late_exit{pose(0,5,0),pose(50,5,0),pose(60,-10,0)};
  EXPECT_GT(p.mainLaneEntryCost(early_exit),p.mainLaneEntryCost(late_exit));
}
TEST(MainLaneGuidance, HardBlockedMainLaneStillAllowsSafeOppositeLaneContinuation) {
  auto map=emptyMap();auto lane=splitLane();
  for(std::size_t i=0;i<map->data.size();++i) if(lane->data[i]<=20)map->data[i]=100;
  auto p=planner(map);p.setLaneCostmap(lane);
  p.setChannelParams(true,88,85,20,65,12,18,4,2,5,18);
  p.setSearchTimeLimit(3);p.startOverallSearchBudget();
  auto path=p.search(0,-10,0,60,-10,0,0,4);
  ASSERT_FALSE(path.empty());
  EXPECT_LT(std::hypot(path.back().position.x-60,path.back().position.y+10),1.01);
  for(const auto &q:resampleGroundPath(path,.25))EXPECT_FALSE(p.isLethal(q.position.x,q.position.y));
}

TEST(MainLaneGuidance, HeuristicCacheIncludesEntryContextAndLiveStart) {
  auto p=planner(emptyMap());p.setLaneCostmap(splitLane());
  p.setMotionModel("USV_KINEMATIC",.45);
  p.setChannelParams(true,88,85,20,65,12,18,4,2,5,18);
  p.setMainLaneEntryParams(1,30,30);
  ASSERT_FALSE(p.search(0,-10,0,40,10,0).empty());
  const auto first=p.holonomicHeuristicBuildCount();ASSERT_GT(first,0U);
  ASSERT_FALSE(p.search(0,-10,0,40,10,0).empty());
  EXPECT_EQ(p.holonomicHeuristicBuildCount(),first);
  ASSERT_FALSE(p.search(2,-10,0,40,10,0).empty());
  EXPECT_EQ(p.holonomicHeuristicBuildCount(),first+1);
  ASSERT_FALSE(p.search(2,2,0,40,10,0).empty());
  EXPECT_EQ(p.holonomicHeuristicBuildCount(),first+2);
  p.setMainLaneEntryParams(0,30,30);
  ASSERT_FALSE(p.search(2,2,0,40,10,0).empty());
  EXPECT_EQ(p.holonomicHeuristicBuildCount(),first+3);
}

TEST(MainLaneGuidance, TerminalHeadingSampleOrderMirrorsLaneSideAndYaw) {
  for(double yaw:{0.0,.1,.5,1.5,2.8,M_PI})for(int side:{-1,1}) {
    EXPECT_DOUBLE_EQ(mainLaneHeadingOffsetPreference(yaw,side),
      -mainLaneHeadingOffsetPreference(-yaw,-side));
  }
  EXPECT_DOUBLE_EQ(mainLaneHeadingOffsetPreference(M_PI/2,1),-1);
  EXPECT_DOUBLE_EQ(mainLaneHeadingOffsetPreference(-M_PI/2,-1),1);
  EXPECT_DOUBLE_EQ(mainLaneHeadingOffsetPreference(0,0),-1);
}
