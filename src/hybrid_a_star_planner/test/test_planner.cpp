#include "hybrid_a_star_planner/planner.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdlib>
#include <fstream>
using namespace hybrid_a_star_planner;
using namespace hybrid_a_star_planner::planning;


TEST(WaypointContinuity, RecognizesOnlyForwardWaypointsInTheSameGlobalRoute) {
  nav_msgs::msg::Path route;
  route.header.frame_id = "odom";
  route.poses.resize(4);
  for (std::size_t i = 0; i < route.poses.size(); ++i) {
    route.poses[i].pose.position.x = 10.0 * static_cast<double>(i);
    route.poses[i].pose.orientation.w = 1.0;
  }
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "odom";
  goal.pose.position.x = 20.0;

  EXPECT_EQ(matchGlobalRouteGoalIndex(
      &route, goal, kNoGlobalRouteGoalIndex), 2U);
  EXPECT_EQ(matchGlobalRouteGoalIndex(&route, goal, 1U), 2U);
  EXPECT_EQ(matchGlobalRouteGoalIndex(&route, goal, 2U), kNoGlobalRouteGoalIndex);
  goal.header.frame_id = "map";
  EXPECT_EQ(matchGlobalRouteGoalIndex(
      &route, goal, kNoGlobalRouteGoalIndex), kNoGlobalRouteGoalIndex);
}


TEST(WaypointContinuity, RetainsPathOnlyForForwardWaypointAdvance) {
  constexpr auto no_index = kNoGlobalRouteGoalIndex;
  EXPECT_TRUE(isForwardRouteWaypointAdvance(true, 2U, 3U));
  EXPECT_TRUE(isForwardRouteWaypointAdvance(true, 2U, 6U));
  EXPECT_FALSE(isForwardRouteWaypointAdvance(false, 2U, 3U));
  EXPECT_FALSE(isForwardRouteWaypointAdvance(true, 3U, 3U));
  EXPECT_FALSE(isForwardRouteWaypointAdvance(true, 3U, 2U));
  EXPECT_FALSE(isForwardRouteWaypointAdvance(true, no_index, 2U));
}


TEST(MotionModelSelection, DefaultsToUsvAndRetainsCarLikeOption) {
  HybridAStarPlanner planner;
  EXPECT_TRUE(planner.usesUsvDynamicModel());
  EXPECT_FALSE(planner.usesUsvKinematicModel());
  EXPECT_EQ(planner.motionModelName(), "USV_DYNAMIC");

  planner.setParams(2.0F, 9.0F, 5.0F, 0.5F, 0.1F, 100000, 88);
  planner.setDynamicParams(4.0, 1.5, 4.0, 60.0, 1.2, 9.0, 8.0, 12.0, 0.05);
  ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.20));
  EXPECT_TRUE(planner.usesUsvKinematicModel());
  EXPECT_NEAR(planner.maximumAllowedCurvature(), 0.05, 1.0e-6);

  // The existing bicycle model remains selectable and still uses wheelbase/rudder angle.
  ASSERT_TRUE(planner.setMotionModel("CAR_LIKE", 0.20));
  EXPECT_FALSE(planner.usesUsvKinematicModel());
  EXPECT_EQ(planner.motionModelName(), "CAR_LIKE");
  EXPECT_NEAR(planner.maximumAllowedCurvature(), std::tan(0.5) / 5.0, 1.0e-6);
  EXPECT_FALSE(planner.setMotionModel("NOT_A_MODEL", 0.20));
}


TEST(DynamicTurnPreference, TightensOnlyLargeUsvReversalsWithinModelLimits) {
  HybridAStarPlanner planner;
  planner.setStabilityParams(0.0, 12.0, 1.0, 12.0, 24.0, 6.0);
  planner.setTurnaroundParams(120.0 * M_PI / 180.0, 0.25);
  EXPECT_NEAR(planner.steeringMagnitudeWeightForGoal(0.0, 30.0 * M_PI / 180.0), 12.0, 1.0e-9);
  EXPECT_NEAR(planner.steeringMagnitudeWeightForGoal(0.0, M_PI), 3.0, 1.0e-9);
  ASSERT_TRUE(planner.setMotionModel("CAR_LIKE", 0.45));
  EXPECT_NEAR(planner.steeringMagnitudeWeightForGoal(0.0, M_PI), 12.0, 1.0e-9);
}

struct BendFixture {
  HybridAStarPlanner planner;
  nav_msgs::msg::OccupancyGrid::SharedPtr map, lane;
  double angle, slope;
  BendFixture(double bend_degrees = 20.0, double half_width = 12.0, bool hard_banks = false)
    : angle(bend_degrees * deg), slope(std::tan(angle))
  {
  map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width = 180;
  map->info.height = 140;
  map->info.resolution = 1.0;
  map->info.origin.position.x = -60.0;
  map->info.origin.position.y = -60.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(map->info.width * map->info.height, 0);
  lane = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  for (unsigned y = 0; y < map->info.height; ++y) {
    for (unsigned x = 0; x < map->info.width; ++x) {
      const double wx = map->info.origin.position.x + x + 0.5;
      const double wy = map->info.origin.position.y + y + 0.5;
      const double center = slope * std::max(0.0, wx);
      const double offset = std::abs(wy - center);
      lane->data[y * map->info.width + x] = offset < half_width ?
        static_cast<int8_t>(std::lround(20.0 * offset / half_width)) : 45;
      if (hard_banks && offset >= half_width) {map->data[y * map->info.width + x] = 100;}
    }
  }
  planner.setCostmap(map);
  planner.setLaneCostmap(lane);
  planner.setParams(2.0F, 8.0F, 5.0F, 0.5F, 0.10F, 250000, 88);
  planner.setDynamicParams(4.0, 0.5, 1.0, 60.0, 3.5, 7.0, 16.0, 8.0, 0.05);
  planner.setChannelParams(true, 88, 85, 20, 65, 2.5, 4.0, 2.0, 2.0, 5.0, 18.0);
  planner.setStabilityParams(0.0, 12.0, 1.5, 12.0, 24.0, 6.0);
  planner.setMotionModel("USV_DYNAMIC", 0.45);
  }
};

TEST(SearchScheduling, BudgetExitsAndNextAttemptStillWorks) {
  BendFixture fixture;
  fixture.planner.setSearchTimeLimit(0.000001);
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_TRUE(fixture.planner.search(0, 0, 0, 70, 25.5, fixture.angle, 0, 4).empty());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(), 0.5);
  fixture.planner.setSearchTimeLimit(0.0);
  EXPECT_FALSE(fixture.planner.search(0, 0, 0, 70, 25.5, fixture.angle, 0, 4).empty());
}

TEST(SearchScheduling, SnapshotDoesNotOverwriteNewObstacleMap) {
  BendFixture fixture;
  auto worker = std::make_shared<HybridAStarPlanner>(fixture.planner);
  auto future = std::async(std::launch::async, [worker, &fixture]() {
    return worker->search(0, 0, 0, 70, 25.5, fixture.angle, 0, 4);
  });
  auto blocked = std::make_shared<nav_msgs::msg::OccupancyGrid>(*fixture.map);
  blocked->data.assign(blocked->data.size(), 100);
  fixture.planner.setChannelObstacleMap(blocked);
  auto path = future.get();
  ASSERT_FALSE(path.empty());
  fixture.planner.adoptSearchCache(*worker);
  EXPECT_TRUE(fixture.planner.remainingPathNeedsReplan(path, 0, 0, 0, 2, 8, false));
}

TEST(SearchScheduling, GuidedChannelConnectionReducesSearchWaiting) {
  BendFixture old_fixture, fast_fixture;
  const auto before = std::chrono::steady_clock::now();
  auto original = old_fixture.planner.search(0, 0, 0, 70, 25.5, old_fixture.angle, 0, 4);
  const double old_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - before).count();
  fast_fixture.planner.setFastGuidedConnection(true);
  const auto begin = std::chrono::steady_clock::now();
  auto fast = fast_fixture.planner.search(0, 0, 0, 70, 25.5, fast_fixture.angle, 0, 4);
  const double fast_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  ASSERT_FALSE(original.empty()); ASSERT_FALSE(fast.empty());
  EXPECT_EQ(fast_fixture.planner.holonomicHeuristicBuildCount(), 0U);
  EXPECT_LT(std::hypot(fast.back().position.x-70,fast.back().position.y-25.5),1.0);
  EXPECT_FALSE(fast_fixture.planner.remainingPathNeedsReplan(fast,0,0,0,2,8,false));
  std::cout << "Channel connection benchmark: baseline=" << old_ms << "ms fast=" << fast_ms << "ms\n";
}

TEST(SearchScheduling, LateResultTrimsSailedPrefixWithoutFabricatingStart) {
  std::vector<geometry_msgs::msg::Pose> path(10);
  for (std::size_t i=0;i<path.size();++i) {path[i].position.x=i; path[i].orientation.w=1;}
  ASSERT_TRUE(trimPathToMeasuredState(path,5.1,0.1,0,1.5,0.35));
  EXPECT_DOUBLE_EQ(path.front().position.x,5.0);
  EXPECT_EQ(path.size(),5U);
  EXPECT_FALSE(trimPathToMeasuredState(path,5.1,0.1,1.0,1.5,0.35));
  EXPECT_FALSE(trimPathToMeasuredState(path,5.1,3.0,0,1.5,0.35));
}

struct BendMetrics {
  double peak_curvature{0.0}, max_center_error{0.0}, turning_length{0.0}, length{0.0};
};

BendMetrics measureBend(const std::vector<geometry_msgs::msg::Pose> & path,
  double slope, const std::string & name)
{
  BendMetrics metrics;
  double previous_course = 0.0;
  std::ofstream output;
  if (const char * directory = std::getenv("USV_BEND_REPORT_DIR")) {
    output.open(std::string(directory) + "/" + name + ".csv");
    output << "x,y,yaw,curvature\n";
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1U].position;
    const auto & b = path[i].position;
    const double ds = std::hypot(b.x - a.x, b.y - a.y);
    const double course = std::atan2(b.y - a.y, b.x - a.x);
    const double k = std::abs(normalizeAngle(course - previous_course)) / std::max(1.0e-6, ds);
    metrics.peak_curvature = std::max(metrics.peak_curvature, k);
    metrics.length += ds;
    if (k > 0.001) {metrics.turning_length += ds;}
    metrics.max_center_error = std::max(metrics.max_center_error,
      std::abs(b.y - slope * std::max(0.0, b.x)));
    previous_course = course;
    if (output) {output << b.x << ',' << b.y << ',' << tf2::getYaw(path[i].orientation) << ',' << k << '\n';}
  }
  std::cout << name << ": peak curvature=" << metrics.peak_curvature <<
    ", center error=" << metrics.max_center_error << ", turning length=" << metrics.turning_length <<
    ", length=" << metrics.length << '\n';
  return metrics;
}

TEST(ChannelBroadBend, SpreadsTurnInBothDirectionsWithoutChangingMinimumRadius) {
  for (const double sign : {1.0, -1.0}) {
    SCOPED_TRACE(sign);
    BendFixture fixture(20.0 * sign);
    auto & planner = fixture.planner;
    planner.setGentleTurnSampling(false);
    planner.setChannelCurveParams(60.0, 0.0);
    const double curvature_limit = planner.maximumAllowedCurvature();
    const auto before = planner.search(-45.0F, 0.0F, 0.0F,
      80.0F, static_cast<float>(80.0 * fixture.slope), fixture.angle, 0.0, 4.0, 0.0, 0.0);
    ASSERT_GT(before.size(), 20U);
    planner.setGentleTurnSampling(true);
    planner.setChannelCurveParams(60.0, 1.0);
    const auto after = planner.search(-45.0F, 0.0F, 0.0F,
      80.0F, static_cast<float>(80.0 * fixture.slope), fixture.angle, 0.0, 4.0, 0.0, 0.0);
    ASSERT_GT(after.size(), 20U);
    const std::string side = sign > 0.0 ? "left" : "right";
    const auto old_metrics = measureBend(before, fixture.slope, side + "_before");
    const auto new_metrics = measureBend(after, fixture.slope, side + "_after");
    EXPECT_LT(new_metrics.peak_curvature, old_metrics.peak_curvature * 0.5);
    EXPECT_LT(new_metrics.peak_curvature, 1.0 / 55.0);
    // Central band of a 24m-wide lane; every model-integrated point stays inside.
    EXPECT_LT(new_metrics.max_center_error, 5.0);
    EXPECT_GT(new_metrics.turning_length, 75.0);
    EXPECT_LT(new_metrics.length, old_metrics.length * 1.1);
    EXPECT_DOUBLE_EQ(planner.maximumAllowedCurvature(), curvature_limit);
    EXPECT_LT(std::hypot(after.back().position.x - 80.0,
      after.back().position.y - 80.0 * fixture.slope), 1.0);
    for (std::size_t i = 1; i < after.size(); ++i) {
      EXPECT_FALSE(planner.isLethal(after[i].position.x, after[i].position.y));
      EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(after[i].orientation) -
        tf2::getYaw(after[i-1U].orientation))) / 0.05, 0.45 + 0.015);
    }
  }
}

TEST(ChannelBroadBend, NarrowBendCanStillUseTighterLegalTurns) {
  BendFixture fixture(45.0, 4.0, true);
  const auto path = fixture.planner.search(-30.0F, 0.0F, 0.0F,
    45.0F, 45.0F, fixture.angle, 0.0, 4.0, 0.0, 0.0);
  ASSERT_GT(path.size(), 20U);
  const auto metrics = measureBend(path, fixture.slope, "narrow");
  EXPECT_GT(metrics.peak_curvature, 1.0 / 60.0);
  for (const auto & pose : path) {
    EXPECT_FALSE(fixture.planner.isLethal(pose.position.x, pose.position.y));
  }
}

TEST(ChannelBroadBend, BroadRefinementCannotCutThroughChannelObstaclesOrTraffic) {
  BendFixture fixture;
  fixture.planner.setFastGuidedConnection(true);
  auto obstacles = std::make_shared<nav_msgs::msg::OccupancyGrid>(*fixture.map);
  for (unsigned y = 0; y < obstacles->info.height; ++y) {
    for (unsigned x = 0; x < obstacles->info.width; ++x) {
      const double wx = obstacles->info.origin.position.x + x + 0.5;
      const double wy = obstacles->info.origin.position.y + y + 0.5;
      if (wx > 5.0 && wx < 13.0 && std::abs(wy - fixture.slope * wx) < 3.0) {
        obstacles->data[y * obstacles->info.width + x] = 100;
      }
    }
  }
  fixture.planner.setChannelObstacleMap(obstacles);
  DynamicObstacle traffic;
  traffic.state = {35.0, fixture.slope * 35.0 + 10.0, 0.0, -0.5, 0.5};
  fixture.planner.setDynamicHardDistance(4.0);
  fixture.planner.setDynamicObstacles({traffic});
  const auto path = fixture.planner.search(-45.0F, 0.0F, 0.0F,
    80.0F, static_cast<float>(80.0 * fixture.slope), fixture.angle, 0.0, 4.0, 0.0, 0.0);
  ASSERT_GT(path.size(), 20U);
  measureBend(path, fixture.slope, "obstacles");
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & pose = path[i];
    EXPECT_FALSE(fixture.planner.isLethal(pose.position.x, pose.position.y));
    const double time = 0.05 * i;
    const double clearance = std::hypot(pose.position.x - traffic.state.x - traffic.state.vx * time,
      pose.position.y - traffic.state.y - traffic.state.vy * time);
    minimum_clearance = std::min(minimum_clearance, clearance);
    EXPECT_GE(clearance, 4.0);
  }
  std::cout << "moving traffic minimum clearance=" << minimum_clearance << '\n';
}
TEST(UnderactuatedRollout, PropagatesSwayAndLimitsYawRate) {
  hybrid_a_star_planner::dynamics::Parameters parameters;
  parameters.max_yaw_rate = 0.45;
  parameters.cruise_speed = 4.0;
  parameters.integration_step = 0.025;
  hybrid_a_star_planner::dynamics::State state;
  state.u = 4.0;
  for (int i = 0; i < 40; ++i) {
    state = hybrid_a_star_planner::dynamics::stepSetpoint(
      state, 4.0, 0.35, parameters, 0.05);
  }
  EXPECT_GT(state.y, 1.0);
  EXPECT_GT(state.yaw, 0.2);
  EXPECT_LT(state.v, -0.1);  // k_vr<0 makes a starboard yaw-rate command induce sway.
  EXPECT_LE(std::abs(state.r), parameters.max_yaw_rate + 1.0e-9);
  EXPECT_LE(state.u, parameters.cruise_speed + 1.0e-9);

  Policy head_on;
  head_on.locked = true;
  head_on.type = EncounterType::HEAD_ON;
  head_on.level = Risk::ACTION;
  head_on.reference = 0.0;
  EXPECT_TRUE(headingAllowedDynamic(head_on, 0.0, 0.2, 0.0, 0.45, 0.96));
  EXPECT_FALSE(headingAllowedDynamic(head_on, 0.03, 0.5, 0.0, 0.45, 0.96));
  EXPECT_TRUE(headingAllowedDynamic(head_on, -0.12, 1.2, 0.0, 0.45, 0.96));
}

TEST(LargeHeadingTurn, GuidedDynamicTurnReachesOppositeHeading) {
  for(double start_speed: {0.0,4.0}) {
    HybridAStarPlanner planner;
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=140;map->info.height=120;map->info.resolution=1.;
    map->info.origin.position.x=-70;map->info.origin.position.y=-60;
    map->info.origin.orientation.w=1;map->data.assign(16800,0);
    planner.setCostmap(map);
    planner.setParams(2,8,5,.5,.1,250000,88);
    planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
    hybrid_a_star_planner::dynamics::Parameters params;planner.setUsvDynamics(params);
    planner.setLargeTurnParams(true,2,5,1.57);
    planner.setSearchTimeLimit(3);
    auto begin=std::chrono::steady_clock::now();
    auto path=planner.search(0,0,0,-30,10,M_PI,0,start_speed,0,0);
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    ASSERT_GT(path.size(),20U) << "start speed " << start_speed << " elapsed " << elapsed;
    EXPECT_LT(std::hypot(path.back().position.x+30,path.back().position.y-10),1.0);
    EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-M_PI)),M_PI/6);
    EXPECT_LT(elapsed,3.0);
    for(std::size_t i=1;i<path.size();++i) {
      EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path[i].orientation)-
        tf2::getYaw(path[i-1].orientation)))/.05,.451);
    }
    std::cout << "large turn speed=" << start_speed << " time=" << elapsed
      << " cap=" << planner.planningSpeed() << " poses=" << path.size() << std::endl;
  }
}

TEST(LargeHeadingTurn, BothSidesAndSeveralAnglesRemainReachable) {
  for(const auto &goal: std::vector<std::array<double,3>>{
    {20,25,M_PI/2},{-20,25,3*M_PI/4},{-30,-10,-M_PI},{20,-25,-M_PI/2}}) {
    HybridAStarPlanner planner;
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=140;map->info.height=120;map->info.resolution=1.;
    map->info.origin.position.x=-70;map->info.origin.position.y=-60;
    map->info.origin.orientation.w=1;map->data.assign(16800,0);
    planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
    planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
    planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
    planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
    auto path=planner.search(0,0,0,goal[0],goal[1],goal[2],0,4,0,0);
    ASSERT_GT(path.size(),20U) << goal[0] << "," << goal[1] << "," << goal[2];
    EXPECT_LT(std::hypot(path.back().position.x-goal[0],path.back().position.y-goal[1]),1.0);
    EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-goal[2])),M_PI/6);
  }
}

TEST(LargeHeadingTurn, NarrowWaterUsesLowSpeedWithoutChangingMeasuredStart) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=120;map->info.height=28;map->info.resolution=1.;
  map->info.origin.position.x=-60;map->info.origin.position.y=-14;
  map->info.origin.orientation.w=1;map->data.assign(3360,0);
  planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
  planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
  planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
  auto path=planner.search(0,0,0,-30,10,M_PI,0,4,0,0);
  ASSERT_GT(path.size(),20U);
  EXPECT_LT(planner.planningSpeed(),4); // Dense sampling may find a faster feasible intermediate speed.
  EXPECT_GE(planner.planningSpeed(),1.2);
  EXPECT_LT(planner.planningRadius(),8);
  EXPECT_GE(planner.planningRadius(),3);
  EXPECT_NEAR(path.front().position.x,0,1e-6);
  EXPECT_GT(path[1].position.x/.05,3.7); // 初始4m/s没有瞬间重置为2m/s。
  for(const auto &p:path) {EXPECT_GE(p.position.y,-14);EXPECT_LT(p.position.y,14);}
}

TEST(LargeHeadingTurn, FastConnectionStillRejectsTrafficAndAnUnreachableWall) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=140;map->info.height=120;map->info.resolution=1.;
  map->info.origin.position.x=-70;map->info.origin.position.y=-60;
  map->info.origin.orientation.w=1;map->data.assign(16800,0);
  planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
  planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
  planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
  auto original=planner.search(0,0,0,-30,10,M_PI,0,4,0,0);
  ASSERT_GT(original.size(),20U);
  const auto &p=original[original.size()/3].position;
  DynamicObstacle ship;ship.state={p.x,p.y,0,0,1};
  planner.setDynamicHardDistance(3);planner.setDynamicObstacles({ship});
  auto alternative=planner.search(0,0,0,-30,10,M_PI,0,4,0,0);
  ASSERT_GT(alternative.size(),20U);
  for(const auto &pose:alternative) {
    EXPECT_GE(std::hypot(pose.position.x-p.x,pose.position.y-p.y),2.999);
  }
  planner.setDynamicObstacles({});
  for(int y=0;y<120;++y) {map->data[y*140+60]=100;}
  planner.setCostmap(map);
  EXPECT_TRUE(planner.search(0,0,0,-30,10,M_PI,0,4,0,0).empty());
}

TEST(CloseHeadingTurn, NearbyPositionAndLargeHeadingDifference) {
  for(const double initial_speed:{0.0,4.0}) {
  for(const auto &goal:std::vector<std::array<double,3>>{
      {0,0,M_PI},{.5,0,M_PI},{1,0,M_PI},{2,0,M_PI},{5,0,M_PI},{-5,0,M_PI},
      {5,3,3*M_PI/4},{2,2,M_PI/2},{2,-2,-M_PI/2},{3,1,3*M_PI/8}}) {
    HybridAStarPlanner planner;
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=100;map->info.height=100;map->info.resolution=1.;
    map->info.origin.position.x=-50;map->info.origin.position.y=-50;
    map->info.origin.orientation.w=1;map->data.assign(10000,0);
    planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
    planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
    planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
    planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
    auto begin=std::chrono::steady_clock::now();
    auto path=planner.search(0,0,0,goal[0],goal[1],goal[2],0,initial_speed,0,0);
    EXPECT_GT(path.size(),20U) << "goal " << goal[0] << "," << goal[1] << "," << goal[2];
    if(path.empty()) {continue;}
    EXPECT_LT(std::hypot(path.back().position.x-goal[0],path.back().position.y-goal[1]),1);
    EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-goal[2])),M_PI/6);
    double length=0;
    for(std::size_t i=1;i<path.size();++i) {
      length+=std::hypot(path[i].position.x-path[i-1].position.x,
        path[i].position.y-path[i-1].position.y);
    }
    EXPECT_LE(length,std::max({1.0,std::hypot(goal[0],goal[1]),
      planner.planningRadius()*std::abs(goal[2])})*3+40);
    EXPECT_FALSE(pathHasAvoidableGoalLoop(path,3,8));
    std::cout << "close goal=" << goal[0] << "," << goal[1] << " yaw=" << goal[2]
      << " delay=" << std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()
      << " initial_speed=" << initial_speed << " speed=" << planner.planningSpeed()
      << " length=" << length << " poses=" << path.size() << std::endl;
  }
  }
}

TEST(CloseHeadingTurn, NecessaryTurnIsNotMistakenForAnAvoidableGoalLoop) {
  std::vector<geometry_msgs::msg::Pose> path(5);
  for(auto &p:path) {p.orientation.w=1;}
  path[1].position.x=8;path[2].position.x=8;path[2].position.y=8;
  path[3].position.x=3;path[3].position.y=2;
  path.back().position.x=.5;path.back().orientation.w=0;path.back().orientation.z=1;
  EXPECT_FALSE(pathHasAvoidableGoalLoop(path,3,8));
  path.front().orientation.w=0;path.front().orientation.z=1;
  EXPECT_TRUE(pathHasAvoidableGoalLoop(path,3,8));
}

TEST(AdaptiveCTurn, RearGoalsPreferOneTurnAcrossReferenceSpeeds) {
  for(double initial_speed:{0.0,4.0}) {
    for(const auto &goal:std::vector<std::array<double,3>>{
      {-30,10,M_PI},{-30,-10,-M_PI},{-60,20,M_PI},{-15,8,M_PI},{20,25,M_PI/2}}) {
      HybridAStarPlanner planner;auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
      map->info.width=200;map->info.height=160;map->info.resolution=1;
      map->info.origin.position.x=-100;map->info.origin.position.y=-80;
      map->data.assign(32000,0);planner.setCostmap(map);
      planner.setParams(2,8,5,.5,.1,250000,88);planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
      planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
      planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
      const auto start=std::chrono::steady_clock::now();
      const auto path=planner.search(0,0,0,goal[0],goal[1],goal[2],0,initial_speed);
      ASSERT_FALSE(path.empty());
      // 后方掉头必须用新增连接解决反舵；90度原连接本身若也是C弯则允许保留。
      if(std::abs(goal[2])>2.0) {EXPECT_TRUE(planner.cTurnSelected())
        << "goal="<<goal[0]<<","<<goal[1]<<" u="<<initial_speed;}
      double left=0,right=0,course_left=0,course_right=0,previous_course=0,peak_rate=0,length=0;
      std::ofstream csv;
      if(const char *directory=std::getenv("USV_BEND_REPORT_DIR")) {
        csv.open(std::string(directory)+"/c_"+std::to_string(static_cast<int>(goal[0]))+"_"+
          std::to_string(static_cast<int>(goal[1]))+"_u"+std::to_string(static_cast<int>(initial_speed))+".csv");
        csv<<"x,y,yaw,curvature\n";
      }
      for(std::size_t i=1;i<path.size();++i) {
        const double d=normalizeAngle(tf2::getYaw(path[i].orientation)-tf2::getYaw(path[i-1].orientation));
        left+=std::max(0.0,d);right+=std::max(0.0,-d);
        peak_rate=std::max(peak_rate,std::abs(d)/.05);
        const double dx=path[i].position.x-path[i-1].position.x,dy=path[i].position.y-path[i-1].position.y;
        const double ds=std::hypot(dx,dy),course=std::atan2(dy,dx);
        const double change=i>1 ? normalizeAngle(course-previous_course) : 0;
        course_left+=std::max(0.0,change);course_right+=std::max(0.0,-change);previous_course=course;length+=ds;
        if(csv) {csv<<path[i].position.x<<","<<path[i].position.y<<","<<tf2::getYaw(path[i].orientation)<<","<<change/std::max(.001,ds)<<"\n";}
      }
      EXPECT_LT(std::min(left,right),.02);
      EXPECT_LT(std::min(course_left,course_right),.04); // 实际地面航迹也不能有明显反向曲率。
      EXPECT_LE(peak_rate,.451);
      EXPECT_LT(std::hypot(path.back().position.x-goal[0],path.back().position.y-goal[1]),1.0);
      std::cout<<"C goal="<<goal[0]<<","<<goal[1]<<" u="<<initial_speed<<" C="<<planner.cTurnSelected()
        <<" speed="<<planner.planningSpeed()<<" left="<<left<<" right="<<right
        <<" course_reverse="<<std::min(course_left,course_right)<<" length="<<length<<" time="
        <<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<std::endl;
      if(initial_speed==4 && goal[0]==-30 && goal[1]==10) {
        auto baseline=planner;baseline.setAdaptiveTurnParams(false);
        const auto old=baseline.search(0,0,0,goal[0],goal[1],goal[2],0,initial_speed);
        ASSERT_FALSE(old.empty());double old_left=0,old_right=0;
        std::ofstream before;
        if(const char *directory=std::getenv("USV_BEND_REPORT_DIR")) {
          before.open(std::string(directory)+"/c_before.csv");before<<"x,y,yaw,curvature\n";
        }
        double course=0;
        for(std::size_t i=1;i<old.size();++i) {
          const double turn=normalizeAngle(tf2::getYaw(old[i].orientation)-tf2::getYaw(old[i-1].orientation));
          old_left+=std::max(0.0,turn);old_right+=std::max(0.0,-turn);
          const double dx=old[i].position.x-old[i-1].position.x,dy=old[i].position.y-old[i-1].position.y;
          const double next=std::atan2(dy,dx),change=i>1 ? normalizeAngle(next-course) : 0;course=next;
          if(before) {before<<old[i].position.x<<","<<old[i].position.y<<","<<tf2::getYaw(old[i].orientation)
            <<","<<change/std::max(.001,std::hypot(dx,dy))<<"\n";}
        }
        EXPECT_GT(std::min(old_left,old_right),.2);
        EXPECT_LT(std::min(left,right),.1*std::min(old_left,old_right));
        std::cout<<"C comparison reverse heading: before="<<std::min(old_left,old_right)
          <<" after="<<std::min(left,right)<<std::endl;
      }
    }
  }
}

TEST(DynamicUsvPlanning, IntegratedPathHasBoundedYawRateAndSingleTurnDirection) {
  HybridAStarPlanner planner;
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width = 140;
  map->info.height = 120;
  map->info.resolution = 1.0;
  map->info.origin.position.x = -50.0;
  map->info.origin.position.y = -45.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(map->info.width * map->info.height, 0);
  planner.setCostmap(map);
  planner.setParams(2.0F, 9.0F, 5.0F, 0.5F, 0.08F, 250000, 88);
  planner.setDynamicParams(4.0, 0.5, 1.0, 60.0, 3.0, 7.0, 8.0, 8.0, 0.05);
  hybrid_a_star_planner::dynamics::Parameters parameters;
  parameters.cruise_speed = 4.0;
  parameters.max_yaw_rate = 0.45;
  planner.setUsvDynamics(parameters);
  ASSERT_TRUE(planner.setMotionModel("USV_DYNAMIC", 0.45));
  planner.setStabilityParams(0.0, 12.0, 1.5, 12.0, 24.0, 6.0);

  const auto path = planner.search(-30.0F, 0.0F, 0.0F, 20.0F, 15.0F, 0.7F,
    0.0, 4.0, 0.0, 0.0);
  ASSERT_GT(path.size(), 20U);
  int curvature_sign_changes = 0;
  int previous_sign = 0;
  double max_yaw_rate = 0.0;
  double max_ground_track_slip = 0.0;
  double max_ground_track_curvature = 0.0;
  double first_turn_distance = std::numeric_limits<double>::infinity();
  double accumulated_distance = 0.0;
  double previous_course = 0.0;
  bool have_previous_course = false;
  constexpr double sample_dt = 0.05;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dyaw = normalizeAngle(
      tf2::getYaw(path[i].orientation) - tf2::getYaw(path[i - 1U].orientation));
    max_yaw_rate = std::max(max_yaw_rate, std::abs(dyaw) / sample_dt);
    const double course = std::atan2(
      path[i].position.y - path[i - 1U].position.y,
      path[i].position.x - path[i - 1U].position.x);
    const double segment_distance = std::hypot(
      path[i].position.x - path[i - 1U].position.x,
      path[i].position.y - path[i - 1U].position.y);
    if (have_previous_course && segment_distance > 1.0e-4) {
      const double course_change = normalizeAngle(course - previous_course);
      max_ground_track_curvature = std::max(max_ground_track_curvature,
        std::abs(course_change) / segment_distance);
    }
    if (std::abs(dyaw) >= 0.002 && !std::isfinite(first_turn_distance)) {
      first_turn_distance = accumulated_distance;
    }
    accumulated_distance += segment_distance;
    previous_course = course;
    have_previous_course = true;
    const double mean_heading = tf2::getYaw(path[i - 1U].orientation) + 0.5 * dyaw;
    max_ground_track_slip = std::max(max_ground_track_slip,
      std::abs(normalizeAngle(course - mean_heading)));
    const int sign = std::abs(dyaw) < 0.002 ? 0 : (dyaw > 0.0 ? 1 : -1);
    if (sign != 0 && previous_sign != 0 && sign != previous_sign) {
      ++curvature_sign_changes;
    }
    if (sign != 0) {previous_sign = sign;}
  }
  EXPECT_LE(max_yaw_rate, 0.45 + 0.015);
  // On an open map the requested course change must remain one-directional;
  // a genuine curvature-sign transition is an S maneuver and should not occur.
  EXPECT_EQ(curvature_sign_changes, 0);
  EXPECT_LT(max_ground_track_slip, 0.25);
  EXPECT_LT(max_ground_track_curvature, 1.0 / 12.0);
  EXPECT_LT(first_turn_distance, 0.5 * accumulated_distance);
  EXPECT_LT(std::hypot(path.back().position.x - 20.0,
      path.back().position.y - 15.0), 1.1);

  // The mirrored right-turn goal should have the same C-shaped, low-slip
  // behavior; this catches accidental left-turn-only smoothing preferences.
  const auto mirrored = planner.search(-30.0F, 0.0F, 0.0F, 20.0F, -15.0F, -0.7F,
    0.0, 4.0, 0.0, 0.0);
  ASSERT_GT(mirrored.size(), 20U);
  int mirrored_sign_changes = 0;
  int mirrored_previous_sign = 0;
  for (std::size_t i = 1; i < mirrored.size(); ++i) {
    const double dyaw = normalizeAngle(tf2::getYaw(mirrored[i].orientation) -
      tf2::getYaw(mirrored[i - 1U].orientation));
    const int sign = std::abs(dyaw) < 0.002 ? 0 : (dyaw > 0.0 ? 1 : -1);
    if (sign != 0 && mirrored_previous_sign != 0 && sign != mirrored_previous_sign) {
      ++mirrored_sign_changes;
    }
    if (sign != 0) {mirrored_previous_sign = sign;}
  }
  EXPECT_EQ(mirrored_sign_changes, 0);
  EXPECT_LT(std::hypot(mirrored.back().position.x - 20.0,
      mirrored.back().position.y + 15.0), 1.1);
}

TEST(DynamicUsvPlanning, StartsAtCurrentPoseAndTurnsWithinRadiusFromRest) {
  HybridAStarPlanner planner;
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width = 140;
  map->info.height = 120;
  map->info.resolution = 1.0;
  map->info.origin.position.x = -50.0;
  map->info.origin.position.y = -45.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(map->info.width * map->info.height, 0);
  planner.setCostmap(map);
  planner.setParams(2.0F, 9.0F, 5.0F, 0.5F, 0.08F, 250000, 88);
  planner.setDynamicParams(4.0, 0.5, 1.0, 60.0, 3.0, 7.0, 8.0, 8.0, 0.05);
  hybrid_a_star_planner::dynamics::Parameters parameters;
  parameters.cruise_speed = 4.0;
  parameters.max_yaw_rate = 0.45;
  planner.setUsvDynamics(parameters);
  ASSERT_TRUE(planner.setMotionModel("USV_DYNAMIC", 0.45));

  constexpr double start_x = -30.0;
  constexpr double start_y = 0.0;
  constexpr double dt = 0.05;
  const auto path = planner.search(start_x, start_y, 0.0, 20.0, 15.0, 0.7,
    0.0, 0.0, 0.0, 0.0);
  ASSERT_GT(path.size(), 20U);
  EXPECT_NEAR(path.front().position.x, start_x, 1.0e-9);
  EXPECT_NEAR(path.front().position.y, start_y, 1.0e-9);
  EXPECT_NEAR(tf2::getYaw(path.front().orientation), 0.0, 1.0e-9);

  double max_yaw_rate = 0.0;
  double max_yaw_change_per_distance = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double yaw_change = std::abs(normalizeAngle(
      tf2::getYaw(path[i].orientation) - tf2::getYaw(path[i - 1U].orientation)));
    const double distance = std::hypot(
      path[i].position.x - path[i - 1U].position.x,
      path[i].position.y - path[i - 1U].position.y);
    max_yaw_rate = std::max(max_yaw_rate, yaw_change / dt);
    if (distance > 1.0e-4) {
      max_yaw_change_per_distance = std::max(
        max_yaw_change_per_distance, yaw_change / distance);
    }
  }
  EXPECT_LE(max_yaw_rate, 0.45 + 0.02);
  EXPECT_LE(max_yaw_change_per_distance, 1.0 / 9.0 + 0.03);
  EXPECT_LT(std::hypot(path.back().position.x - 20.0,
      path.back().position.y - 15.0), 1.1);
}

TEST(DynamicUsvPlanning, IntegratedRolloutKeepsClearOfTraffic) {
  HybridAStarPlanner planner;
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width = 120;
  map->info.height = 100;
  map->info.resolution = 1.0;
  map->info.origin.position.x = -50.0;
  map->info.origin.position.y = -50.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(map->info.width * map->info.height, 0);
  planner.setCostmap(map);
  planner.setParams(2.0F, 9.0F, 5.0F, 0.5F, 0.08F, 250000, 88);
  planner.setDynamicParams(4.0, 0.5, 1.0, 60.0, 3.0, 7.0, 8.0, 8.0, 0.05);
  hybrid_a_star_planner::dynamics::Parameters parameters;
  parameters.cruise_speed = 4.0;
  parameters.max_yaw_rate = 0.45;
  planner.setUsvDynamics(parameters);
  ASSERT_TRUE(planner.setMotionModel("USV_DYNAMIC", 0.45));
  DynamicObstacle traffic;
  traffic.state = {0.0, 0.0, 0.0, 0.0, 0.5};
  planner.setDynamicObstacles({traffic});

  const auto path = planner.search(-25.0F, 0.0F, 0.0F, 25.0F, 0.0F, 0.0F,
    0.0, 4.0, 0.0, 0.0);
  ASSERT_GT(path.size(), 20U);
  double minimum_clearance = std::numeric_limits<double>::infinity();
  double max_detour = 0.0;
  for (const auto &pose : path) {
    minimum_clearance = std::min(minimum_clearance,
      std::hypot(pose.position.x, pose.position.y));
    max_detour = std::max(max_detour, std::abs(pose.position.y));
  }
  EXPECT_GE(minimum_clearance, 2.0);
  EXPECT_GT(max_detour, 1.0);
}

TEST(MotionModelSelection, UsvSearchPathRespectsYawRateAndSpeedCurvatureLimit) {
  HybridAStarPlanner planner;
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width = 140;
  map->info.height = 120;
  map->info.resolution = 1.0;
  map->info.origin.position.x = -60.0;
  map->info.origin.position.y = -50.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(map->info.width * map->info.height, 0);
  planner.setCostmap(map);
  planner.setParams(2.0F, 9.0F, 5.0F, 0.5F, 0.08F, 250000, 88);
  planner.setDynamicParams(4.0, 1.5, 4.0, 60.0, 1.2, 9.0, 8.0, 12.0, 0.05);
  ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.20));

  const auto path = planner.search(-30.0F, 0.0F, 0.0F, 30.0F, 30.0F, 0.8F);
  ASSERT_GT(path.size(), 20U);
  const double allowed_curvature = planner.maximumAllowedCurvature();
  double measured_max_curvature = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = std::hypot(
      path[i].position.x - path[i - 1].position.x,
      path[i].position.y - path[i - 1].position.y);
    if (distance < 1.0e-5) {continue;}
    const double yaw_delta = std::abs(normalizeAngle(
      tf2::getYaw(path[i].orientation) - tf2::getYaw(path[i - 1].orientation)));
    measured_max_curvature = std::max(measured_max_curvature, yaw_delta / distance);
  }
  EXPECT_GT(measured_max_curvature, 0.005);
  EXPECT_LE(measured_max_curvature, allowed_curvature * 1.25 + 0.015);
}

TEST(CurrentDistanceSafety, TwoMetreStopBoundaryIsIndependentOfCPA) {
  EXPECT_TRUE(insideStopDistance(-270,-270,-268.01,-270,2));
  EXPECT_FALSE(insideStopDistance(-270,-270,-268,-270,2));
  EXPECT_FALSE(insideStopDistance(-270,-270,-267.99,-270,2));
  EXPECT_TRUE(insideStopDistance(0,0,0,-1.9,2));
}

TEST(EventDrivenPlanning, ReplansOnlyWhenRemainingPathBecomesUnsafe) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=100;map->info.height=80;map->info.resolution=1;
  map->info.origin.position.x=-10;map->info.origin.position.y=-40;
  map->info.origin.orientation.w=1;map->data.assign(100*80,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.1,200000,88);
  planner.setDynamicParams(4,0.5,1,60,3,6,12,8,0.05,3);
  std::vector<geometry_msgs::msg::Pose> straight;
  for(int x=0;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;pose.orientation.w=1;
    straight.push_back(pose);
  }
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 能被感知但运动轨迹与本船路径不相交，不应仅因“附近有船”重规划。
  DynamicObstacle harmless;harmless.state={20,15,0,2,0.5};
  planner.setDynamicObstacles({harmless});
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 正面对遇会进入预测碰撞域并违反右舷行动要求，必须触发一次新搜索。
  DynamicObstacle head_on;head_on.state={25,0,-2,0,0.5};
  head_on.policy.locked=true;head_on.policy.type=EncounterType::HEAD_ON;
  head_on.policy.level=Risk::ACTION;head_on.policy.reference=0;
  head_on.encounter.active=true;
  planner.setDynamicObstacles({head_on});
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));

  // 同一次会遇成功规划后不再重复消费规则触发；若缓存路径本身没有进入
  // 硬碰撞域，关闭规则复查应保持执行，防止匀速目标导致每秒重搜。
  head_on.state.y=4.0;planner.setDynamicObstacles({head_on});
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.0,5,true));
  EXPECT_FALSE(planner.remainingPathNeedsReplan(straight,0,0,0,2.0,5,false));
  head_on.state.y=0.0;planner.setDynamicObstacles({head_on});

  // 已规划到右舷且净空足够的路径继续交给MPC执行，不重复搜索。
  std::vector<geometry_msgs::msg::Pose> starboard;
  for(int x=0;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=-0.25*std::min(x,48);pose.orientation.w=1;
    starboard.push_back(pose);
  }
  EXPECT_FALSE(planner.remainingPathNeedsReplan(starboard,0,0,0,2.5,5));

  // 地图中新出现的硬障碍落在剩余路径上时，即使没有目标船也要重规划。
  planner.setDynamicObstacles({});
  map->data[40*100+20]=100;planner.setCostmap(map);
  EXPECT_TRUE(planner.remainingPathNeedsReplan(straight,0,0,0,2.5,5));
}

TEST(EventDrivenPlanning, RenewedCollisionCancelsRecoveryImmediately) {
  Policy policy;policy.locked=true;policy.recovering=true;
  policy.type=EncounterType::HEAD_ON;policy.level=Risk::ACTION;
  policy.reference=0;policy.speed=4;
  policy.update(1,0,0,0,4,{25,0,-2,0,0.5});
  EXPECT_FALSE(policy.recovering);
}

TEST(SearchPerformance, ReusesHolonomicHeuristicUntilStaticMapChanges) {
  HybridAStarPlanner planner;
  // Exercise the lattice cache; dynamic aligned legs now use a direct rollout.
  planner.setMotionModel("USV_KINEMATIC",.45);
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=80;map->info.height=40;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-20;
  map->info.origin.orientation.w=1.0;map->data.assign(80*40,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.1,100000,88);
  ASSERT_FALSE(planner.search(0,0,0,40,0,0).empty());
  const auto first_builds=planner.holonomicHeuristicBuildCount();
  ASSERT_EQ(first_builds,1U);
  ASSERT_FALSE(planner.search(2,0,0,40,0,0).empty());
  EXPECT_EQ(planner.holonomicHeuristicBuildCount(),first_builds);
  planner.setCostmap(map);
  ASSERT_FALSE(planner.search(2,0,0,40,0,0).empty());
  EXPECT_EQ(planner.holonomicHeuristicBuildCount(),first_builds+1U);
}
// 回归：预测减速时间与无等待的软连续性风险门控。
// 行为恢复必须独立于会遇记录的解除滞回。
TEST(RecoveryCompletion, DoesNotWaitForHistoricalReleaseTimer) {
  Policy p;p.locked=true;p.type=EncounterType::CROSSING_STARBOARD;
  p.level=Risk::EMERGENCY;p.reference=0;p.metric={10,-0.1,8};
  // 船尾净空满足7.5m，但尚未满足TCPA<-2s且连续两次的历史解锁条件。
  EXPECT_TRUE(p.safeToRecover(0,0,{0,10,0,2,2},7.5,false));
  EXPECT_FALSE(p.safeToRecover(0,0,{0,10,0,2,2},7.5,true));
  EXPECT_FALSE(p.safeToRecover(0,0,{10,-1,0,2,2},7.5,false));
  p.recovering=true;
  EXPECT_TRUE(headingAllowed(p,0.2,1,0,-0.1));
  EXPECT_FALSE(p.right());
  EXPECT_TRUE(p.locked);
}
TEST(RecoveryContinuity, ArrivalModelAccountsForGoalBraking) {
  HybridAStarPlanner planner;
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  EXPECT_DOUBLE_EQ(planner.estimateSegmentTime(-5,0,0,0,5),1.25);
  planner.setRecoveryParams(2,6,true,0.5,0.3,0.4);
  EXPECT_GT(planner.estimateSegmentTime(-5,0,0,0,5),4.0);
  EXPECT_TRUE(std::isfinite(planner.estimateSegmentTime(0,0,0,0,1)));
}
TEST(RecoveryContinuity, RiskGateClearsWithoutWaitingForPolicyUnlock) {
  HybridAStarPlanner planner;
  ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.45));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=220;map->info.height=180;map->info.resolution=1;
  map->info.origin.position.x=-50;map->info.origin.position.y=-90;
  map->data.assign(220*180,0);planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.1,250000,88);
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  planner.setRecoveryParams(2,6,true,0.5,0.3,0.4);
  DynamicObstacle o;o.state={0,-20,0,2,2};
  planner.setDynamicObstacles({o});
  auto avoiding=planner.search(-40,0,0,55,0,0);
  EXPECT_TRUE(planner.recoveryContinuityActive());ASSERT_FALSE(avoiding.empty());
  avoiding=planner.postProcessPath(avoiding,1,120,0.26,0.015,6,6,0.12);
  double elapsed=0;
  for(std::size_t i=1;i<avoiding.size();++i) {
    const auto &a=avoiding[i-1].position;const auto &b=avoiding[i].position;
    const double length=std::hypot(b.x-a.x,b.y-a.y);
    const int count=std::max(1,static_cast<int>(std::ceil(length/0.25)));
    double px=a.x,py=a.y;
    for(int j=1;j<=count;++j) {
      const double f=static_cast<double>(j)/count;
      const double x=a.x+(b.x-a.x)*f,y=a.y+(b.y-a.y)*f;
      elapsed+=planner.estimateSegmentTime(px,py,x,y,length/count);px=x;py=y;
      EXPECT_GE(std::hypot(x-o.state.x-o.state.vx*elapsed,
        y-o.state.y-o.state.vy*elapsed),7.45);
    }
  }
  planner.setReferencePath(avoiding);
  // 实际通过后，即使旧策略尚有锁定余量，额外连续性也立即关闭。
  o.state={0,40,0,2,2};o.policy.locked=true;
  o.policy.type=EncounterType::CROSSING_STARBOARD;o.policy.reference=0;
  o.policy.level=Risk::ACTION;
  planner.setDynamicObstacles({o});
  EXPECT_FALSE(planner.search(20,-5,-0.3,55,0,0).empty());
  EXPECT_FALSE(planner.recoveryContinuityActive());
  EXPECT_TRUE(headingAllowed(o.policy,0,1,0,-25*deg));
}

// 连续规划的近场参考应抑制左右换边，但不得覆盖动态碰撞硬约束。
TEST(TemporalStability, NearFieldPrefersPreviousSafeSide) {
  HybridAStarPlanner planner;
  ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.45));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=130;map->info.height=80;map->info.resolution=1;
  map->info.origin.position.x=-40;map->info.origin.position.y=-40;
  map->info.origin.orientation.w=1;map->data.assign(130*80,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(4,1.5,4,60,1.2,9,8,12,0.05,9);
  planner.setStabilityParams(0.5,12,1,1);
  planner.setNearFieldStabilityParams(12,45,3);
  std::vector<geometry_msgs::msg::Pose> previous;
  for(int x=-30;x<=50;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=3.0*std::sin(M_PI*(x+30)/80.0);pose.orientation.w=1;
    previous.push_back(pose);
  }
  planner.setReferencePath(previous);
  const auto path=planner.search(-30,0,0,50,0,0);
  ASSERT_FALSE(path.empty());
  double near_sum=0;int near_count=0;
  for(const auto &pose:path) if(pose.position.x>-20 && pose.position.x<10) {
    near_sum+=pose.position.y;++near_count;
  }
  ASSERT_GT(near_count,5);
  EXPECT_GT(near_sum/near_count,0.4);
}

TEST(TemporalStability, DynamicSafetyOverridesPreviousPath) {
  HybridAStarPlanner planner;
  ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.45));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=130;map->info.height=100;map->info.resolution=1;
  map->info.origin.position.x=-40;map->info.origin.position.y=-50;
  map->info.origin.orientation.w=1;map->data.assign(130*100,0);
  planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(4,0.5,1,60,3,6,8,12,0.05,3);
  planner.setNearFieldStabilityParams(12,45,3);
  std::vector<geometry_msgs::msg::Pose> previous;
  for(int x=-30;x<=50;++x) {geometry_msgs::msg::Pose p;p.position.x=x;p.orientation.w=1;previous.push_back(p);}
  planner.setReferencePath(previous);
  DynamicObstacle obstacle;obstacle.state={0,0,0,0,0.5};
  obstacle.policy.locked=true;obstacle.policy.type=EncounterType::HEAD_ON;
  obstacle.encounter.active=true;
  planner.setDynamicObstacles({obstacle});
  const auto path=planner.search(-30,0,0,50,0,0);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(planner.referenceSoftConflictActive());
  for(const auto &pose:path)
    EXPECT_GE(std::hypot(pose.position.x,pose.position.y),1.99);
}

TEST(FourMetrePerSecondPlanning, SmallerVesselsStillAvoidAllEncounterTypes) {
  for(const auto type:{EncounterType::HEAD_ON,EncounterType::CROSSING_STARBOARD,
      EncounterType::CROSSING_PORT,EncounterType::OVERTAKING}) {
    SCOPED_TRACE(toString(type));
    HybridAStarPlanner planner;
    ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.45));
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=220;map->info.height=180;map->info.resolution=1;
    map->info.origin.position.x=-50;map->info.origin.position.y=-90;
    map->info.origin.orientation.w=1;map->data.assign(220*180,0);
    planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,88);
    planner.setDynamicParams(4,0.5,1,60,3,6,8,8,0.05,3);
    planner.setStabilityParams(0.5,12,1,1);
    DynamicObstacle obstacle;
    obstacle.policy.type=type;obstacle.policy.locked=true;
    obstacle.policy.level=Risk::ACTION;obstacle.policy.speed=4;
    obstacle.policy.takeover=type==EncounterType::CROSSING_PORT;
    if(type==EncounterType::HEAD_ON) obstacle.state={60,0,-2,0,0.5};
    if(type==EncounterType::CROSSING_STARBOARD) obstacle.state={0,-20,0,2,0.5};
    if(type==EncounterType::CROSSING_PORT) obstacle.state={0,20,0,-2,0.5};
    if(type==EncounterType::OVERTAKING) obstacle.state={-10,0,2,0,0.5};
    planner.setDynamicObstacles({obstacle});
    auto path=planner.search(-40,0,0,55,0,0);
    ASSERT_GT(path.size(),20U);
    path=planner.postProcessPath(path,1,120,0.26,0.015,6,6,0.12);
    double distance=0;
    for(std::size_t i=1;i<path.size();++i) {
      const double dx=path[i].position.x-path[i-1].position.x;
      const double dy=path[i].position.y-path[i-1].position.y;
      const double length=std::hypot(dx,dy);
      const int samples=std::max(1,static_cast<int>(std::ceil(length/0.25)));
      for(int sample=1;sample<=samples;++sample) {
        const double fraction=static_cast<double>(sample)/samples;
        const double time=(distance+fraction*length)/4;
        const double x=path[i-1].position.x+fraction*dx;
        const double y=path[i-1].position.y+fraction*dy;
        EXPECT_GE(std::hypot(x-obstacle.state.x-obstacle.state.vx*time,
          y-obstacle.state.y-obstacle.state.vy*time),
          type==EncounterType::OVERTAKING ? 2.99 : 1.99);
      }
      distance+=length;
    }
  }
}

TEST(V4Policy, RiskAndRule17Sequence) {
  EXPECT_EQ(risk({80,90,20}),Risk::MONITOR);
  EXPECT_EQ(risk({45,35,15}),Risk::ACTION);
  EXPECT_EQ(risk({20,12,8}),Risk::EMERGENCY);
  EXPECT_EQ(risk({10,-1,0}),Risk::CLEAR);
  Policy p;
  VesselState target{30,30,0,-0.8,4};
  p.update(0,0,0,0,0.8,target);
  EXPECT_EQ(p.type,EncounterType::CROSSING_PORT);
  EXPECT_FALSE(p.takeover);
  p.update(1,0.8,0,0,0.8,{30,29.2,0,-0.8,4});
  EXPECT_FALSE(p.takeover);
  p.update(2,1.6,0,0,0.8,{30,28.4,0,-0.8,4});
  EXPECT_TRUE(p.takeover);
  EXPECT_FALSE(headingAllowed(p,0,2,2,0));
  EXPECT_TRUE(headingAllowed(p,-0.1,2,2,0));
}
TEST(V4Policy, ConfigurableRiskThresholds) {
  RiskThresholds t;t.monitor_dcpa=16;t.action_dcpa=13;t.emergency_dcpa=9;
  t.monitor_range=60;t.action_range=35;t.emergency_range=18;
  t.monitor_tcpa=60;t.action_tcpa=30;t.emergency_tcpa=10;
  EXPECT_EQ(risk({55,50,15.5},t),Risk::MONITOR);
  EXPECT_EQ(risk({34,29,12.5},t),Risk::ACTION);
  EXPECT_EQ(risk({17,9,8.5},t),Risk::EMERGENCY);
  EXPECT_EQ(risk({70,70,17},t),Risk::CLEAR);
}
TEST(V4Policy, EffectiveTargetAndLockRelease) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // Target turns to its starboard (west) and clears the own-ship track.
  p.update(1,0.8,0,0,0.8,{30,29.2,-0.8,0,4});
  EXPECT_TRUE(p.effective); EXPECT_FALSE(p.takeover);
  EXPECT_EQ(p.type,EncounterType::CROSSING_PORT);
  // Three distinct observations after CPA, separated by a full release margin.
  for(int i=2;i<=4;++i) p.update(i,80+i,0,0,0.8,{-40,0,-0.8,0,4});
  EXPECT_FALSE(p.locked);
}
TEST(V4Policy, ObservableTargetManoeuvreDoesNotTriggerPrematureRule17) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // A developing starboard alteration is observable, but its DCPA benefit is
  // intentionally still below the effectiveness threshold.
  p.update(1,0.8,0,0,0.8,{29.98,29.2,-0.045,-0.799,4});
  p.update(2,1.6,0,0,0.8,{29.91,28.4,-0.056,-0.798,4});
  EXPECT_FALSE(p.takeover);
  EXPECT_TRUE(p.standOn());
}
TEST(V4Policy, FailSafeStopCannotMaskRule17Risk) {
  Policy p;
  p.update(0,0,0,0,0.8,{30,30,0,-0.8,4});
  // Simulate the local controller stopping while the other vessel continues.
  p.update(1,0,0,0,0.0,{30,29.2,0,-0.8,4});
  p.update(2,0,0,0,0.0,{30,28.4,0,-0.8,4});
  EXPECT_TRUE(p.takeover);
  EXPECT_TRUE(p.right());
}
TEST(V4Policy, AnglesAndActionLevels) {
  for(double rotation:{0.0,1.0,-2.9}) {
    for(int angle=-19;angle<=19;++angle) {
      double bearing=rotation+angle*deg;
      auto a=assessEncounter(0,0,rotation,0.8,
        {50*std::cos(bearing),50*std::sin(bearing),-0.8*std::cos(rotation),
        -0.8*std::sin(rotation),4},1e9,1e9,20*deg,20*deg,112.5*deg,0.05);
      EXPECT_EQ(a.type,EncounterType::HEAD_ON);
    }
  }
  for(Risk r:{Risk::MONITOR,Risk::ACTION,Risk::EMERGENCY}) {
    Policy p;p.locked=true;p.type=EncounterType::HEAD_ON;p.level=r;
    EXPECT_FALSE(headingAllowed(p,0,4,0,0));
    EXPECT_FALSE(headingAllowed(p,0.1,4,0,0));
    EXPECT_TRUE(headingAllowed(p,-0.15,4,0,0));
  }
  Policy completed;completed.locked=true;completed.takeover=true;
  completed.type=EncounterType::CROSSING_PORT;completed.level=Risk::EMERGENCY;
  EXPECT_TRUE(headingAllowed(completed,0.0,1,0,-21*deg));
}

TEST(PathPostProcessing, ReducesGlobalZigZagWithoutMovingEndpoints) {
  HybridAStarPlanner planner;
  ASSERT_TRUE(planner.setMotionModel("CAR_LIKE", 0.45));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=80;map->info.height=60;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-30;
  map->info.origin.orientation.w=1.0;map->data.assign(80*60,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);
  std::vector<geometry_msgs::msg::Pose> raw;
  for(int x=0;x<=30;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=(x==0 || x==30)?0.0:(x%2==0?1.0:-1.0);
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  auto length=[](const auto &path) {
    double result=0;
    for(std::size_t i=1;i<path.size();++i) result+=std::hypot(
      path[i].position.x-path[i-1].position.x,
      path[i].position.y-path[i-1].position.y);
    return result;
  };
  auto heading_variation=[](const auto &path) {
    double result=0;
    for(std::size_t i=2;i<path.size();++i) {
      const double previous=std::atan2(
        path[i-1].position.y-path[i-2].position.y,
        path[i-1].position.x-path[i-2].position.x);
      const double current=std::atan2(
        path[i].position.y-path[i-1].position.y,
        path[i].position.x-path[i-1].position.x);
      result+=std::abs(normalizeAngle(current-previous));
    }
    return result;
  };
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  EXPECT_NEAR(smooth.front().position.x,raw.front().position.x,1e-9);
  EXPECT_NEAR(smooth.front().position.y,raw.front().position.y,1e-9);
  EXPECT_NEAR(smooth.back().position.x,raw.back().position.x,1e-9);
  EXPECT_NEAR(smooth.back().position.y,raw.back().position.y,1e-9);
  EXPECT_LT(length(smooth),0.85*length(raw));
  // 即使输入为每米左右交替的极端锯齿，整体累计转向波动也必须至少降低60%。
  EXPECT_LT(heading_variation(smooth),0.40*heading_variation(raw));
}

TEST(PathPostProcessing, NeverCutsAcrossDynamicShipSafetyArea) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=60;map->info.height=50;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-20;
  map->info.origin.orientation.w=1.0;map->data.assign(60*50,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);

  DynamicObstacle obstacle;
  obstacle.state={15,0,0,0,2};
  obstacle.policy.type=EncounterType::NONE;
  planner.setDynamicObstacles({obstacle});

  std::vector<geometry_msgs::msg::Pose> raw;
  for(const auto &[x,y]:std::vector<std::pair<double,double>>{
      {0,0},{5,8},{10,9},{15,9},{20,9},{25,8},{30,0}}) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;pose.position.y=y;
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  const double safe_radius=2.0+2.0+3.0;
  for(std::size_t i=1;i<smooth.size();++i) {
    const double dx=smooth[i].position.x-smooth[i-1].position.x;
    const double dy=smooth[i].position.y-smooth[i-1].position.y;
    const int samples=std::max(1,static_cast<int>(std::ceil(std::hypot(dx,dy)/0.25)));
    for(int sample=0;sample<=samples;++sample) {
      const double ratio=static_cast<double>(sample)/samples;
      const double x=smooth[i-1].position.x+ratio*dx;
      const double y=smooth[i-1].position.y+ratio*dy;
      EXPECT_GE(std::hypot(x-obstacle.state.x,y-obstacle.state.y),safe_radius-1.0e-6);
    }
  }
}

TEST(PathPostProcessing, SuppressesLongWavelengthSerpentine) {
  HybridAStarPlanner planner;
  ASSERT_TRUE(planner.setMotionModel("CAR_LIKE", 0.45));
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=140;map->info.height=60;map->info.resolution=1.0;
  map->info.origin.position.x=-10;map->info.origin.position.y=-30;
  map->info.origin.orientation.w=1.0;map->data.assign(140*60,0);
  planner.setCostmap(map);
  planner.setParams(2,9,5,0.5,0.10,250000,88);
  planner.setDynamicParams(1.2,2,3,120,1.35,12,8,8,0.05);
  std::vector<geometry_msgs::msg::Pose> raw;
  for(int x=0;x<=100;++x) {
    geometry_msgs::msg::Pose pose;pose.position.x=x;
    pose.position.y=5.0*std::sin(2.0*M_PI*x/50.0);
    pose.orientation.w=1.0;raw.push_back(pose);
  }
  const auto smooth=planner.postProcessPath(raw,1.0,120,0.26,0.015,6.0,6,0.12);
  ASSERT_GE(smooth.size(),3U);
  double raw_peak=0.0,smooth_peak=0.0;
  for(const auto &pose:raw) {
    if(pose.position.x>=10.0 && pose.position.x<=90.0)
      raw_peak=std::max(raw_peak,std::abs(pose.position.y));
  }
  for(const auto &pose:smooth) {
    if(pose.position.x>=10.0 && pose.position.x<=90.0)
      smooth_peak=std::max(smooth_peak,std::abs(pose.position.y));
  }
  EXPECT_LT(smooth_peak,0.65*raw_peak);
  EXPECT_NEAR(smooth.front().position.y,raw.front().position.y,1e-9);
  EXPECT_NEAR(smooth.back().position.y,raw.back().position.y,1e-9);
}

class PlannerV4: public ::testing::Test {
protected:
  HybridAStarPlanner planner;

  void SetUp() override {
    ASSERT_TRUE(planner.setMotionModel("USV_KINEMATIC", 0.45));
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=160;map->info.height=140;map->info.resolution=1;
    map->info.origin.position.x=-50;map->info.origin.position.y=-70;
    map->info.origin.orientation.w=1;map->data.assign(160*140,0);
    planner.setCostmap(map);planner.setParams(2,9,5,0.5,0.10,250000,40);
    planner.setDynamicParams(0.8,2,2,120,2,12,8,8,0.05);
    planner.setStabilityParams(0.0,12.0,1.0,0.6);
  }

  void check(EncounterType type, VesselState t, bool takeover=false) {
    DynamicObstacle o;o.state=t;o.policy.type=type;o.policy.locked=true;
    o.policy.level=Risk::ACTION;o.policy.takeover=takeover;
    o.encounter.type=type;o.encounter.active=true;o.encounter.tcpa=30;o.encounter.urgency=0.8;
    o.encounter.preference=o.policy.right()?TurnPreference::RIGHT:TurnPreference::NONE;
    planner.setDynamicObstacles({o});
    const auto path=planner.search(-30,0,0,55,0,0);
    ASSERT_GT(path.size(),20u);
    double time=0, minimum=1e9;
    for(std::size_t i=1;i<path.size();++i) {
      time+=std::hypot(path[i].position.x-path[i-1].position.x,
                      path[i].position.y-path[i-1].position.y)/0.8;
      minimum=std::min(minimum,std::hypot(path[i].position.x-t.x-t.vx*time,
                                        path[i].position.y-t.y-t.vy*time));
      EXPECT_TRUE(headingAllowed(o.policy,tf2::getYaw(path[i].orientation),time,0,0));
      if(type==EncounterType::HEAD_ON) {
        const double target_x=t.x+t.vx*time;
        const double forward=target_x-path[i].position.x;
        if(forward>=-8.0 && time<=12.0) {
          EXPECT_LE(normalizeAngle(tf2::getYaw(path[i].orientation)),2.01*deg);
          if(forward<=0.0) {
            const double target_y=t.y+t.vy*time;
            EXPECT_LE(path[i].position.y-target_y,0.251);
          }
        }
      }
    }
    EXPECT_GE(minimum,type==EncounterType::OVERTAKING?14.99:7.99);
    EXPECT_LT(std::hypot(path.back().position.x-55,path.back().position.y),0.6);
    EXPECT_LT(std::abs(tf2::getYaw(path.back().orientation)),0.15);
  }
};
TEST_F(PlannerV4, HeadOn) {check(EncounterType::HEAD_ON,{25,0,-0.6,0,4});}
TEST_F(PlannerV4, HeadOnInitiallyOffset) {check(EncounterType::HEAD_ON,{25,-2,-0.6,0,4});}
TEST_F(PlannerV4, StarboardCrossing) {check(EncounterType::CROSSING_STARBOARD,{0,-30,0,0.8,4});}
TEST_F(PlannerV4, PortRule17Takeover) {check(EncounterType::CROSSING_PORT,{0,30,0,-0.8,4},true);}
TEST_F(PlannerV4, PortCooperative) {check(EncounterType::CROSSING_PORT,{0,30,-0.8,0,4});}
TEST_F(PlannerV4, Overtaking) {check(EncounterType::OVERTAKING,{-5,0,0.3,0,4});}
TEST_F(PlannerV4, OpenWater) {
  auto path=planner.search(-30,0,0,50,14.1,10*deg);
  ASSERT_GT(path.size(),20u);
  EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-10*deg)),0.15);
}

TEST(ChannelFusion, ShoreInflationIsIgnoredOnlyInsideLane) {
  HybridAStarPlanner planner;
  auto local=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  local->info.width=20;local->info.height=20;local->info.resolution=1;
  local->info.origin.orientation.w=1;local->data.assign(400,90);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  lane->data.assign(400,45);
  lane->data[10*20+10]=0;
  planner.setCostmap(local);planner.setLaneCostmap(lane);
  planner.setChannelParams(true,88,85,10,65,0.65,1.1,0.8,2,5,18);
  EXPECT_FALSE(planner.isLethal(10.5,10.5));
  EXPECT_TRUE(planner.isLethal(11.5,10.5));
}

TEST(ChannelFusion, ChannelObstacleInflationIsNeverCleared) {
  HybridAStarPlanner planner;
  auto local=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  local->info.width=20;local->info.height=20;local->info.resolution=1;
  local->info.origin.orientation.w=1;local->data.assign(400,90);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  lane->data.assign(400,0);
  auto obstacle=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  obstacle->data.assign(400,0);obstacle->data[10*20+10]=85;
  planner.setCostmap(local);planner.setLaneCostmap(lane);
  planner.setChannelObstacleMap(obstacle);
  planner.setChannelParams(true,88,85,10,65,0.65,1.1,0.8,2,5,18);
  EXPECT_TRUE(planner.isLethal(10.5,10.5));
  EXPECT_FALSE(planner.isLethal(9.5,10.5));
}

TEST(WaypointHandoff, MovingQuarterTurnPreservesMeasuredSway) {
  for(double side:{-1.0,1.0}) {
    HybridAStarPlanner planner;
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=140;map->info.height=120;map->info.resolution=1.;
    map->info.origin.position.x=-70;map->info.origin.position.y=-60;
    map->info.origin.orientation.w=1;map->data.assign(16800,0);
    planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
    planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
    planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
    planner.setLargeTurnParams(true,2,5,1.57);planner.setSearchTimeLimit(3);
    const auto begin=std::chrono::steady_clock::now();
    auto path=planner.search(0,0,0,14,side*14,side*M_PI/2,0,4,side*.15,side*.025);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    ASSERT_FALSE(path.empty());
    EXPECT_LT(std::hypot(path.back().position.x-14,path.back().position.y-side*14),.5);
    EXPECT_LT(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-side*M_PI/2)),M_PI/6);
    EXPECT_LT(seconds,1.0);
    std::cout<<"moving quarter turn side="<<side<<" seconds="<<seconds<<" speed="<<planner.planningSpeed()<<std::endl;
  }
}

TEST(ManeuverBorrowing, ForwardGoalWithRightHeadingCanBorrowOppositeLane) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=160;map->info.height=28;map->info.resolution=1.;
  map->info.origin.position.x=-70;map->info.origin.position.y=-14;
  map->info.origin.orientation.w=1;map->data.assign(4480,0);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  for(int y=0;y<28;++y) for(int x=0;x<160;++x) {
    lane->data[y*160+x]=y<14 ? 5:75;
  }
  planner.setCostmap(map);planner.setLaneCostmap(lane);planner.setStaticLandMap(map);
  planner.setParams(2,8,5,.5,.1,250000,88);planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
  planner.setChannelParams(true,88,85,20,65,12,18,2,2,5,18);
  planner.setFastGuidedConnection(true);planner.setLargeTurnParams(true,2,5,1.57);
  planner.setGoalHeadingTolerance(M_PI/6);planner.setSearchTimeLimit(3);planner.startOverallSearchBudget();
  auto begin=std::chrono::steady_clock::now();
  auto path=planner.search(0,-3,0,20,-3,-M_PI/2,0,4);
  ASSERT_FALSE(path.empty());
  EXPECT_LT(std::hypot(path.back().position.x-20,path.back().position.y+3),1.01);
  EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)+M_PI/2)),M_PI/6);
  EXPECT_TRUE(std::any_of(path.begin(),path.end(),[](const auto &p){return p.position.y>0;}));
  for(const auto &p:path) {EXPECT_FALSE(planner.isLethal(p.position.x,p.position.y));}
  std::cout<<"borrowed maneuver speed="<<planner.planningSpeed()<<" seconds="<<
    std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<std::endl;
}

TEST(CompactManeuver, RejectsLongFastApproachBeforeDescendingSpeed) {
  HybridAStarPlanner planner;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=160;map->info.height=100;map->info.resolution=1.;
  map->info.origin.position.x=-70;map->info.origin.position.y=-50;
  map->info.origin.orientation.w=1;map->data.assign(16000,0);
  planner.setCostmap(map);planner.setParams(2,8,5,.5,.1,250000,88);
  planner.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  planner.setUsvDynamics(hybrid_a_star_planner::dynamics::Parameters{});
  planner.setLargeTurnParams(true,2,5,1.57);planner.setGoalHeadingTolerance(M_PI/6);
  planner.setSearchTimeLimit(3);planner.startOverallSearchBudget();
  auto path=planner.search(0,0,0,-25,8,M_PI,0,4);
  ASSERT_FALSE(path.empty());EXPECT_LT(planner.planningSpeed(),4);
  double prefix=0,length=0;bool turned=false;
  for(std::size_t i=1;i<path.size();++i) {
    const double ds=std::hypot(path[i].position.x-path[i-1].position.x,path[i].position.y-path[i-1].position.y);
    length+=ds;turned=turned || std::abs(tf2::getYaw(path[i].orientation))>.04;if(!turned) prefix+=ds;
  }
  EXPECT_LT(prefix,8);EXPECT_LT(length,65);
  std::cout<<"compact turn speed="<<planner.planningSpeed()<<" prefix="<<prefix<<" length="<<length<<std::endl;
}

TEST(SpeedSampling, TenDescendingStagesIncludeBothEndpoints) {
  HybridAStarPlanner p;
  auto speeds=p.speedSamples();
  ASSERT_EQ(speeds.size(),10U);
  EXPECT_DOUBLE_EQ(speeds.front(),4.0);
  EXPECT_NEAR(speeds.back(),1.2,1e-9);
  for(std::size_t i=1;i<speeds.size();++i) EXPECT_GT(speeds[i-1],speeds[i]);
}
TEST(SpeedSampling, FeasibleStraightUsesMaximumFromMeasuredLowSpeed) {
  BendFixture f(0);
  f.planner.setFastGuidedConnection(true);
  f.planner.setSearchTimeLimit(3);
  auto path=f.planner.search(0,0,0,60,0,0,0,1.2);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(f.planner.planningSpeed(),4);
  EXPECT_NEAR(path.front().position.x,0,1e-6);
  EXPECT_LT(path[1].position.x/.05,1.5); // acceleration starts at measured speed.
}

TEST(SpeedSampling, OppositeHeadingStraightAcceleratesWithoutLatticeTimeout) {
  BendFixture f(0);
  f.planner.setChannelParams(false,88,85,20,65,12,18,2,2,5,18);
  f.planner.setSearchTimeLimit(.2);
  auto path=f.planner.search(0,0,M_PI,-40,0,M_PI,0,1.2);
  ASSERT_FALSE(path.empty());
  EXPECT_DOUBLE_EQ(f.planner.planningSpeed(),4);
  EXPECT_LT(std::hypot(path.back().position.x+40,path.back().position.y),.4);
  for(const auto &q:path) EXPECT_LT(std::abs(q.position.y),.01);
}


namespace {
HybridAStarPlanner shortLanePlanner(bool opposite, double yaw=0.0) {
  HybridAStarPlanner p;
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=200;map->info.height=100;map->info.resolution=1;
  map->info.origin.position.x=-100;map->info.origin.position.y=-50;
  map->info.origin.orientation.w=1;map->data.assign(20000,0);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  for(int y=0;y<100;++y) for(int x=0;x<200;++x) {
    const double lateral=-(x-99.5)*std::sin(yaw)+(y-49.5)*std::cos(yaw);
    lane->data[y*200+x]=std::abs(lateral)<20 ? (opposite ? 75:5):45;
  }
  p.setCostmap(map);p.setLaneCostmap(lane);p.setStaticLandMap(map);
  p.setParams(2,8,5,.5,.1,250000,88);p.setDynamicParams(4,.5,1,60,3,7,8,8,.05);
  p.setUsvDynamics(dynamics::Parameters{});
  p.setChannelParams(true,88,85,20,65,12,18,2,2,5,18);
  p.setFastGuidedConnection(true);p.setLargeTurnParams(true,2,5,1.57);
  p.setGoalHeadingTolerance(M_PI/6);p.setSearchTimeLimit(3);
  return p;
}
}

TEST(OppositeLaneManeuver, ShortGoalsSupportBothLaneRolesAndDirections) {
  for(bool opposite:{false,true}) for(double distance:{10.0,20.0})
    for(double direction:{-M_PI/2,M_PI/2,M_PI}) {
      auto p=shortLanePlanner(opposite,M_PI);
      p.startOverallSearchBudget();
      auto start=std::chrono::steady_clock::now();
      auto path=p.search(0,0,M_PI,-distance,0,normalizeAngle(M_PI+direction),0,4);
      double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::cout<<"short lane="<<opposite<<" distance="<<distance<<" direction="<<direction
        <<" success="<<!path.empty()<<" speed="<<p.planningSpeed()<<" seconds="<<seconds<<std::endl;
      ASSERT_FALSE(path.empty())<<"opposite="<<opposite<<" distance="<<distance<<" direction="<<direction;
      EXPECT_LT(std::hypot(path.back().position.x+distance,path.back().position.y),1.01);
      EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-M_PI-direction)),M_PI/6);
      for(const auto &q:path) EXPECT_FALSE(p.isLethal(q.position.x,q.position.y));
    }
}

TEST(OppositeLaneManeuver, RearAdjacentLanePrefersTurningTowardTarget) {
  for(double mirror:{-1.0,1.0}) for(double yaw_error:{-.35,0.0,.35}) {
    auto p=shortLanePlanner(false);
    auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    lane->info.width=200;lane->info.height=100;lane->info.resolution=1;
    lane->info.origin.position.x=-100;lane->info.origin.position.y=-50;
    lane->info.origin.orientation.w=1;lane->data.resize(20000);
    for(int y=0;y<100;++y) for(int x=0;x<200;++x) {
      double wy=(y-49.5)*mirror;
      lane->data[y*200+x]=std::abs(wy)>24 ? 45:(wy>0 ? 5:75);
    }
    p.setLaneCostmap(lane);p.startOverallSearchBudget();
    auto path=p.search(0,-6*mirror,0,-20,6*mirror,normalizeAngle(-M_PI+yaw_error),0,4);
    ASSERT_FALSE(path.empty());
    double first=0;
    for(const auto &q:path) {
      double turn=normalizeAngle(tf2::getYaw(q.orientation));
      if(std::abs(turn)>.08) {first=turn;break;}
    }
    EXPECT_GT(first*mirror,0)<<"The vessel initially turned away from the target lane";
    EXPECT_LT(std::hypot(path.back().position.x+20,path.back().position.y-6*mirror),1.01);
    EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)+M_PI-yaw_error)),M_PI/6);
  }
}

TEST(SpeedSampling, LargeMapPrecomputationCannotStarveLowestSpeed) {
  auto p=shortLanePlanner(true);
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=1800;map->info.height=1000;map->info.resolution=1;
  map->info.origin.position.x=-900;map->info.origin.position.y=-500;
  map->info.origin.orientation.w=1;map->data.assign(1800000,0);
  for(int y=0;y<1000;++y) map->data[y*1800+903]=100;
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
  lane->data.assign(1800000,75);
  p.setCostmap(map);p.setStaticLandMap(map);p.setLaneCostmap(lane);
  p.setSearchTimeLimit(1.0);p.startOverallSearchBudget();
  const auto begin=std::chrono::steady_clock::now();
  auto path=p.search(0,0,0,10,0,0,0,4);
  EXPECT_TRUE(path.empty());
  const auto &stages=p.attemptedSpeeds();
  ASSERT_GE(stages.size(),10U);
  // Supplemental speeds do not replace any of the original ten stages.
  auto position=stages.begin();
  for(double expected:p.speedSamples()) {
    position=std::find_if(position,stages.end(),[&](double value){return std::abs(value-expected)<1e-8;});
    ASSERT_NE(position,stages.end());++position;
  }
  EXPECT_DOUBLE_EQ(stages.front(),4);
  EXPECT_NEAR(stages.back(),1.2,1e-9);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(),1.2);
}

TEST(OppositeLaneManeuver, NarrowShortGoalsPreserveMeasuredStartSpeed) {
  for(bool opposite:{false,true}) for(double initial_speed:{0.0,4.0})
    for(double distance:{10.0,20.0}) for(double direction:{-M_PI/2,M_PI/2,M_PI}) {
      auto p=shortLanePlanner(opposite);
      auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
      map->info.width=180;map->info.height=28;map->info.resolution=1;
      map->info.origin.position.x=-80;map->info.origin.position.y=-14;
      map->info.origin.orientation.w=1;map->data.assign(5040,0);
      auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
      lane->data.assign(5040,opposite ? 75:5);
      p.setCostmap(map);p.setStaticLandMap(map);p.setLaneCostmap(lane);
      p.startOverallSearchBudget();
      auto path=p.search(0,0,0,distance,0,direction,0,initial_speed);
      ASSERT_FALSE(path.empty())<<"lane="<<opposite<<" u="<<initial_speed<<" distance="<<distance<<" direction="<<direction;
      EXPECT_LT(std::hypot(path.back().position.x-distance,path.back().position.y),1.01);
      EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-direction)),M_PI/6);
      ASSERT_GT(path.size(),1U);
      EXPECT_NEAR(path.front().position.x,0,1e-6);
      double first_distance=std::hypot(path[1].position.x,path[1].position.y);
      EXPECT_LT(first_distance,initial_speed==0 ? .05:.25);
      for(const auto &q:path) EXPECT_FALSE(p.isLethal(q.position.x,q.position.y));
    }
}

TEST(CompactHeuristicCache, WorkerCacheRetainsCostContextAndInvalidatesOnRangeChange) {
  auto live=shortLanePlanner(true);
  live.setAdaptiveTurnParams(false);live.setFastGuidedConnection(false);
  live.setLargeTurnParams(false,2,5,1.57);
  auto worker=live;
  worker.startOverallSearchBudget();
  ASSERT_FALSE(worker.search(0,0,0,20,0,.3,0,4).empty());
  ASSERT_EQ(worker.holonomicHeuristicBuildCount(),1U);
  live.adoptSearchCache(worker);
  auto next_worker=live;
  next_worker.startOverallSearchBudget();
  ASSERT_FALSE(next_worker.search(0,0,0,20,0,.3,0,4).empty());
  EXPECT_EQ(next_worker.holonomicHeuristicBuildCount(),0U);
  next_worker.startOverallSearchBudget();
  ASSERT_FALSE(next_worker.search(-20,0,0,20,0,.3,0,4).empty());
  EXPECT_EQ(next_worker.holonomicHeuristicBuildCount(),1U);
}

namespace {
// Count substantial ground-track lobes, not body-yaw changes or tiny RK4 jitter.
int significantCourseReversals(const std::vector<geometry_msgs::msg::Pose> &path) {
  std::vector<int> lobes;
  int sign=0;double angle=0,length=0,previous_course=0;
  auto finish=[&]() {
    if(std::abs(angle)>=3.0*M_PI/180.0 && length>=.5) lobes.push_back(sign);
    angle=0;length=0;
  };
  for(std::size_t i=1;i<path.size();++i) {
    double dx=path[i].position.x-path[i-1].position.x;
    double dy=path[i].position.y-path[i-1].position.y,ds=std::hypot(dx,dy);
    if(ds<1e-5) continue;
    double course=std::atan2(dy,dx);
    if(i>1) {
      double change=normalizeAngle(course-previous_course);
      int next=change/ds>.002 ? 1:(change/ds<-.002 ? -1:0);
      if(next && next!=sign) {finish();sign=next;}
      if(next) {angle+=change;length+=ds;}
    }
    previous_course=course;
  }
  finish();int reversals=0;
  for(std::size_t i=1;i<lobes.size();++i) reversals+=lobes[i]!=lobes[i-1];
  return reversals;
}
}

TEST(CurvatureRegression, RearNarrowOffsetUsesLegalSingleTurnWithoutRepeatedS) {
  for(bool opposite:{false,true}) for(double side:{-1.0,1.0}) for(double u:{4.0,1.2}) {
    auto p=shortLanePlanner(opposite);
    p.setStabilityParams(0,12,1.5,12,24,6);
    p.setTurnaroundParams(2.0943951024,.25);
    auto path=p.search(0,0,0,-20,side*5,side*M_PI,0,u);
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(p.cTurnSelected());
    EXPECT_EQ(significantCourseReversals(path),0);
    EXPECT_LT(std::hypot(path.back().position.x+20,path.back().position.y-side*5),.4);
    EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-side*M_PI)),M_PI/6);
    for(const auto &pose:path) EXPECT_FALSE(p.isLethal(pose.position.x,pose.position.y));
  }
}

TEST(CurvatureRegression, MildEqualHeadingsConnectWithMeasuredSwayInBothLaneRoles) {
  for(bool opposite:{false,true}) for(double side:{-1.0,1.0}) {
    auto p=shortLanePlanner(opposite);
    p.setStabilityParams(0,12,1.5,12,24,6);
    p.setGentleTurnSampling(true);p.setChannelCurveParams(60,1,.6);
    const double yaw=side*15*M_PI/180;
    auto begin=std::chrono::steady_clock::now();
    auto path=p.search(0,0,yaw,20,0,yaw,0,4,side*.6,side*.1);
    ASSERT_GT(path.size(),2U);
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(),1.0);
    EXPECT_LT(std::hypot(path.back().position.x-20,path.back().position.y),1.0);
    EXPECT_LE(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-yaw)),M_PI/6);
    EXPECT_LE(significantCourseReversals(path),1);
    // Initial ground velocity includes measured sway; a fake steady-state seed fails this.
    EXPECT_NEAR(path[1].position.y/.05,side*(4*std::sin(15*M_PI/180)+.6*std::cos(15*M_PI/180)),.12);
    for(const auto &pose:path) EXPECT_FALSE(p.isLethal(pose.position.x,pose.position.y));
  }
}

TEST(CurvatureRegression, ShortConnectionCannotCrossIndependentObstacleWall) {
  auto p=shortLanePlanner(true);
  auto obstacle=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  obstacle->info.width=200;obstacle->info.height=100;obstacle->info.resolution=1;
  obstacle->info.origin.position.x=-100;obstacle->info.origin.position.y=-50;
  obstacle->info.origin.orientation.w=1;obstacle->data.assign(20000,0);
  for(int y=0;y<100;++y) obstacle->data[y*200+110]=100;
  p.setChannelObstacleMap(obstacle);p.setSearchTimeLimit(.3);
  EXPECT_TRUE(p.search(0,0,M_PI/12,20,0,M_PI/12,0,4,.6,.1).empty());
}

TEST(GoalHeadingTolerance, ThirtyDegreesAppliesToEveryMotionModelAndAngleWrap) {
  for(const std::string model:{"USV_DYNAMIC","USV_KINEMATIC","CAR_LIKE"}) {
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->info.width=20;map->info.height=20;map->info.resolution=1;
    map->info.origin.position.x=-10;map->info.origin.position.y=-10;
    map->info.origin.orientation.w=1;map->data.assign(400,0);
    HybridAStarPlanner p;p.setCostmap(map);
    p.setParams(2,8,5,.5,.1,1,88);
    ASSERT_TRUE(p.setMotionModel(model,.45));
    p.setSpeedSampling(1);p.setAdaptiveTurnParams(false);
    // Disable exact geometric tails; one lattice pop isolates terminal acceptance.
    p.setChannelParams(false,88,85,20,65,12,18,2,2,5,0);
    EXPECT_DOUBLE_EQ(p.goalHeadingTolerance(),M_PI/6);
    for(double start_yaw:{0.0,179.0*M_PI/180.0,-179.0*M_PI/180.0}) {
      for(double side:{-1.0,1.0}) {
        const double inside=normalizeAngle(start_yaw+side*29*M_PI/180);
        auto path=p.search(0,0,start_yaw,.1,0,inside,0,4);
        ASSERT_FALSE(path.empty())<<model<<" start="<<start_yaw<<" side="<<side;
        EXPECT_NEAR(std::abs(normalizeAngle(tf2::getYaw(path.back().orientation)-inside)),
          29*M_PI/180,1e-6);
        const double outside=normalizeAngle(start_yaw+side*31*M_PI/180);
        EXPECT_TRUE(p.search(0,0,start_yaw,.1,0,outside,0,4).empty())<<model;
      }
    }
  }
}

TEST(ChannelFusion,HardClearanceAndUnknownRemainLethalInsideLane) {
  HybridAStarPlanner planner;
  auto local=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  local->info.width=20;local->info.height=20;local->info.resolution=1;
  local->info.origin.orientation.w=1;local->data.assign(400,0);
  auto lane=std::make_shared<nav_msgs::msg::OccupancyGrid>(*local);
  planner.setCostmap(local);planner.setLaneCostmap(lane);
  planner.setChannelParams(true,88,85,20,65,12,18,4,2,5,18);
  local->data[210]=99;EXPECT_TRUE(planner.isLethal(10.5,10.5));
  local->data[210]=100;EXPECT_TRUE(planner.isLethal(10.5,10.5));
  local->data[210]=-1;EXPECT_TRUE(planner.isLethal(10.5,10.5));
  local->data[210]=80;EXPECT_FALSE(planner.isLethal(10.5,10.5));
}


TEST(ShootingRescue,RecordedOpenFailurePreservesMeasuredStateAndDynamics) {
  HybridAStarPlanner p;auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=240;map->info.height=200;map->info.resolution=1.;
  map->info.origin.position.x=-120;map->info.origin.position.y=-100;map->info.origin.orientation.w=1;
  map->data.assign(48000,0);p.setCostmap(map);p.setStaticLandMap(map);
  p.setParams(2,8,5,.5,.1,250000,88);p.setMotionModel("USV_DYNAMIC",.45);
  p.setUsvDynamics(dynamics::Parameters{});p.setDynamicParams(4,.5,1,120,3.5,7,16,12,.05,3);
  p.setLargeTurnParams(true,2,5,1.57);p.setCloseTurnParams(15,1.2,3);
  p.setAdaptiveTurnParams(true);p.setFastGuidedConnection(true);p.setGoalHeadingTolerance(M_PI/6);
  p.setSearchTimeLimit(3);p.startOverallSearchBudget();
  auto path=p.search(0,0,0,10,20*std::sin(M_PI/3),0,0,4);
  ASSERT_FALSE(path.empty());ASSERT_TRUE(p.usedShootingRescue());
  const auto &trace=p.shootingTrace();ASSERT_FALSE(trace.empty());
  EXPECT_DOUBLE_EQ(trace.front().u,4);EXPECT_DOUBLE_EQ(trace.front().v,0);EXPECT_DOUBLE_EQ(trace.front().r,0);
  EXPECT_LT(std::hypot(path.back().position.x-10,path.back().position.y-20*std::sin(M_PI/3)),.95);
  for(auto state:trace){EXPECT_GE(state.u,0);EXPECT_LE(state.u,4);EXPECT_LE(std::abs(state.r),.450001);}
  auto refined=p.optimizeTrackingPath(path,0);ASSERT_EQ(refined.size(),path.size());
  for(size_t i=0;i<path.size();++i) {EXPECT_DOUBLE_EQ(refined[i].position.x,path[i].position.x);EXPECT_DOUBLE_EQ(refined[i].position.y,path[i].position.y);}
}

TEST(ShootingRescue,CancellationAndHardEndpointAreNeverRelaxed) {
  dynamics::State start{0,0,0,4,0,0};dynamics::Parameters model;
  auto seeds=shootingSeeds(start,10,17.32,0,.6);ASSERT_FALSE(seeds.empty());
  EXPECT_FALSE(optimizeShooting(start,10,17.32,0,seeds.front(),model,[]{return true;}));
  HybridAStarPlanner p;auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->info.width=80;map->info.height=80;map->info.resolution=1.;map->info.origin.position.x=-40;map->info.origin.position.y=-40;map->info.origin.orientation.w=1;
  map->data.assign(6400,0);map->data[57*80+50]=100;p.setCostmap(map);p.setStaticLandMap(map);
  p.setLargeTurnParams(true,2,5,1.57);p.setCloseTurnParams(15,1.2,3);p.setSearchTimeLimit(.2);p.startOverallSearchBudget();
  EXPECT_TRUE(p.search(0,0,0,10,17.32,0,0,4).empty());EXPECT_FALSE(p.usedShootingRescue());
}
