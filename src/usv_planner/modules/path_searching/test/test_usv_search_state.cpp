/**
 * @file test_usv_search_state.cpp
 * @brief 验证搜索去重保留低速加速及不同艏摇趋势。
 */
#include "path_searching/usv_search_state.hpp"
#include "path_searching/usv_dynamics.hpp"
#include "path_searching/kinodynamic_astar.h"

#include <cmath>
#include <gtest/gtest.h>

namespace fast_planner
{
namespace
{
const Eigen::Vector3d kOrigin(-200.0, -160.0, 0.0);
const Eigen::Vector3d kVelocityResolution(0.2, 0.1, 0.05);

SearchStateIndex indexOf(const usv_dynamics::State& state)
{
  return discretizeSearchState(state, kOrigin, 0.25, 72, kVelocityResolution);
}

TEST(USVSearchState, KeepsAccelerationBeforeLeavingPoseVoxel)
{
  usv_dynamics::State start = usv_dynamics::State::Zero();
  start.head<2>() << 0.125, 0.125;
  const auto next = usv_dynamics::integrateRK4(start, {2.0, 0.0, 0.0}, 0.3, 10);
  const auto a = indexOf(start);
  const auto b = indexOf(next);
  EXPECT_TRUE(a.head<3>() == b.head<3>());
  EXPECT_FALSE(a == b);
  EXPECT_GT(b(3), a(3));
}

TEST(USVSearchState, KeepsDifferentSwayAndYawRateAtSamePose)
{
  usv_dynamics::State a = usv_dynamics::State::Zero();
  auto b = a;
  b(4) = 0.2;
  EXPECT_FALSE(indexOf(a) == indexOf(b));
  b = a;
  b(5) = -0.1;
  EXPECT_FALSE(indexOf(a) == indexOf(b));
}

TEST(USVDynamics, PassiveSurgeBrakingUsesModelDamping)
{
  const auto braking = usv_dynamics::estimatePassiveSurgeBraking(2.0, 0.08);
  const double damping = -usv_dynamics::kModelParameters.au;
  EXPECT_NEAR(braking.time_s, std::log(2.0 / 0.08) / damping, 1e-12);
  EXPECT_NEAR(braking.distance_m, (2.0 - 0.08) / damping, 1e-12);

  const auto already_slow = usv_dynamics::estimatePassiveSurgeBraking(-0.05, 0.08);
  EXPECT_DOUBLE_EQ(already_slow.time_s, 0.0);
  EXPECT_DOUBLE_EQ(already_slow.distance_m, 0.0);
}

TEST(USVSearchState, HeadingIsPeriodicAndCenteredAtNorth)
{
  usv_dynamics::State a = usv_dynamics::State::Zero();
  auto b = a;
  a(2) = -0.01;
  b(2) = 0.01;
  EXPECT_TRUE(indexOf(a) == indexOf(b));
  for (double phi : {-7.0, -3.14, -1.0, 0.0, 0.5, 3.14, 8.0}) {
    a(2) = phi;
    b(2) = phi + 4.0 * std::acos(-1.0);
    EXPECT_TRUE(indexOf(a) == indexOf(b));
  }
}

TEST(USVSearchState, HashSeparatesStateAndDynamicTime)
{
  usv_dynamics::State state = usv_dynamics::State::Zero();
  const auto a = indexOf(state);
  state(3) = 0.5;
  const auto b = indexOf(state);
  NodeHashTable table;
  PathNode stopped, moving, later;
  table.insert(a, &stopped);
  table.insert(b, &moving);
  table.insert(b, 3, &later);
  EXPECT_EQ(table.find(a), &stopped);
  EXPECT_EQ(table.find(b), &moving);
  EXPECT_EQ(table.find(b, 3), &later);
  EXPECT_EQ(table.find(b, 2), nullptr);
  table.clear();
  EXPECT_EQ(table.find(a), nullptr);
}

TEST(USVSearchState, IndexedOpenSetMaintainsOrderWhenNodesAreUpdated)
{
  PathNode first;
  PathNode second;
  PathNode third;
  first.f_score = 3.0;
  second.f_score = 1.0;
  third.f_score = 2.0;

  OpenSet open_set;
  open_set.reserve(3);
  open_set.insert(&first);
  open_set.insert(&second);
  open_set.insert(&third);
  EXPECT_EQ(open_set.top(), &second);

  open_set.erase(&third);
  third.f_score = 0.5;
  open_set.insert(&third);
  EXPECT_EQ(open_set.top(), &third);

  open_set.pop();
  EXPECT_EQ(open_set.top(), &second);
  open_set.erase(&second);
  EXPECT_EQ(open_set.top(), &first);

  open_set.clear();
  EXPECT_TRUE(open_set.empty());
  third.f_score = 4.0;
  open_set.insert(&third);
  EXPECT_EQ(open_set.top(), &third);
}

TEST(PlanarDistanceField, DoesNotBiasOpenWaterTowardGridAxes)
{
  PlanarDistanceField field;
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.2, 22.7},
              [](const Eigen::Vector2d&) { return true; });
  for (double y = 0.5; y < 29.0; y += 1.3)
    for (double x = 0.5; x < 29.0; x += 1.7)
      EXPECT_NEAR(field.detour({x, y}), 0.0, 1e-10);
  EXPECT_TRUE(field.descentDirection({3.2, 7.1}, {24.2, 22.7}).isApprox(
      Eigen::Vector2d(21.0, 15.6).normalized()));
}

TEST(PlanarDistanceField, GuidesAroundWallAndRefreshesAfterRemoval)
{
  PlanarDistanceField field;
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.5, 5.5},
              [](const Eigen::Vector2d& p) { return p.x() < 14.0 || p.x() > 16.0 || p.y() > 20.0; });
  EXPECT_GT(field.detour({5.5, 5.5}), 10.0);
  EXPECT_GT(field.descentDirection({10.5, 5.5}, {24.5, 5.5}).y(), 0.1);
  EXPECT_GT(field.detour({10.5, 5.5}), field.detour({10.5, 17.5}));
  EXPECT_NEAR(field.detour({24.5, 5.5}), 0.0, 1e-10);
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.5, 5.5},
              [](const Eigen::Vector2d&) { return true; });
  EXPECT_NEAR(field.detour({5.5, 5.5}), 0.0, 1e-10);
}

TEST(PlanarDistanceField, UnavailableFieldLeavesEuclideanGuidanceUnchanged)
{
  PlanarDistanceField field;
  EXPECT_DOUBLE_EQ(field.detour({0.0, 0.0}), 0.0);
  field.build({0.0, 0.0}, 10, 10, 1.0, {5.5, 5.5},
              [](const Eigen::Vector2d&) { return false; });
  EXPECT_DOUBLE_EQ(field.detour({2.5, 2.5}), 0.0);
  EXPECT_DOUBLE_EQ(field.detour({-1.0, 5.0}), 0.0);
}

TEST(PlanarDistanceField, GuidanceAtWallFollowsFreeGraphEdges)
{
  PlanarDistanceField field;
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.5, 5.5},
              [](const Eigen::Vector2d& p) { return p.x() < 14.0 || p.x() > 16.0 || p.y() > 20.0; });
  const Eigen::Vector2d direction = field.descentDirection({13.5, 5.5}, {24.5, 5.5});
  EXPECT_LE(direction.x(), 1e-9);
  EXPECT_GT(direction.y(), 0.0);
}

TEST(PlanarDistanceField, SoftTravelCostPrefersClearSpaceWithoutBlockingNarrowPassages)
{
  PlanarDistanceField field;
  const auto free = [](const Eigen::Vector2d&) { return true; };
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.5, 5.5}, free,
              [](const Eigen::Vector2d& p) {
                return p.x() >= 14.0 && p.x() <= 16.0 && p.y() <= 20.0 ? 100.0 : 1.0;
              });
  EXPECT_GT(field.detour({13.5, 5.5}), 0.0);
  EXPECT_GT(field.descentDirection({13.5, 5.5}, {24.5, 5.5}).y(), 0.0);
  field.build({0.0, 0.0}, 30, 30, 1.0, {24.5, 5.5}, free);
  EXPECT_NEAR(field.detour({13.5, 5.5}), 0.0, 1e-10);
}

TEST(PlanarDistanceField, PrefersLowerCostRouteAndUsesFourLevelScale)
{
  EXPECT_DOUBLE_EQ(mapTravelCostMultiplier(0.0), 1.0);
  EXPECT_DOUBLE_EQ(mapTravelCostMultiplier(10.0), 1.5);
  EXPECT_DOUBLE_EQ(mapTravelCostMultiplier(20.0), 2.0);

  PlanarDistanceField field;
  field.build({0.0, 0.0}, 32, 32, 1.0, {27.5, 5.5},
              [](const Eigen::Vector2d&) { return true; },
              [](const Eigen::Vector2d& p) {
                return p.x() >= 10.0 && p.x() <= 22.0 && p.y() < 10.0 ? 2.0 : 1.0;
              });
  EXPECT_GT(field.descentDirection({8.5, 5.5}, {27.5, 5.5}).y(), 0.0);
}
}  // namespace
}  // namespace fast_planner
