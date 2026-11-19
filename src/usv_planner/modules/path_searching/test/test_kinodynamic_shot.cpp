/**
 * @file test_kinodynamic_shot.cpp
 * @brief 固定输入复现终点掉头时的 shot 与动力学搜索，验证真实地图查询和终态。
 */
#include "path_searching/kinodynamic_astar.h"
#include "path_searching/usv_dynamics.hpp"
#include "plan_env/edt_environment.h"
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace fast_planner
{
class KinodynamicAstarTest : public ::testing::Test
{
protected:
  using State = usv_dynamics::State;
  KinodynamicAstar planner_;
  std::shared_ptr<rclcpp::Node> node_;

  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("sdf_map/use_grid_map", true)});
    node_ = std::make_shared<rclcpp::Node>("shot_regression", options);
    auto map = std::make_shared<SDFMap>();
    map->initMap(node_);
    auto publisher = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.5;
    grid.info.width = grid.info.height = 600;
    grid.info.origin.position.x = grid.info.origin.position.y = -150.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(600 * 600, 0);
    publisher->publish(grid);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!map->isMapReady() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(map->isMapReady());
    auto environment = std::make_shared<EDTEnvironment>();
    environment->setMap(map);
    planner_.setEnvironment(environment);
    planner_.max_tau_ = 0.5;
    planner_.init_max_tau_ = 2.0;
    planner_.rk4_steps_ = 10;
    planner_.max_u_ = 2.0; planner_.max_v_ = 0.5; planner_.max_r_ = 0.3;
    planner_.min_v_cmd_ = 0.0; planner_.max_v_cmd_ = 2.0; planner_.max_w_cmd_ = 0.3;
    planner_.max_ground_speed_ = 2.0; planner_.max_yaw_rate_ = 0.3;
    planner_.w_time_ = 10.0; planner_.horizon_ = 60.0; planner_.lambda_heu_ = 5.0;
    planner_.search_clearance_ = 0.5; planner_.shot_trigger_distance_ = 30.0;
    planner_.allocate_num_ = 100000; planner_.check_num_ = 5;
    planner_.resolution_ = 1.0; planner_.yaw_bins_ = 72; planner_.time_resolution_ = 0.8;
    planner_.velocity_resolution_ << 0.2, 0.1, 0.05;
    planner_.shot_durations_ = {24., 36., 48., 64., 80., 100.};
    planner_.shot_cruise_speed_ = 1.1;
    planner_.shot_integration_step_ = 0.1; planner_.shot_collision_step_ = 0.04;
    planner_.shot_pos_tolerance_ = 3.5; planner_.shot_yaw_tolerance_ = std::acos(-1.) / 6.;
    planner_.shot_u_tolerance_ = 0.08; planner_.shot_v_tolerance_ = 0.06; planner_.shot_r_tolerance_ = 0.04;
    planner_.shot_min_v_cmd_ = 0.0; planner_.shot_max_v_cmd_ = 2.0;
    planner_.shot_max_w_cmd_ = 0.3; planner_.shot_command_scale_ = 0.85;
    planner_.shot_max_u_ = 2.0; planner_.shot_max_v_ = 0.5; planner_.shot_max_r_ = 0.3;
    planner_.shot_map_height_ = 0.0; planner_.shot_fd_epsilon_ = 0.001;
    planner_.shot_damping_ = 0.02; planner_.shot_time_limit_ = 0.05;
    planner_.shot_max_iterations_ = 16;
    planner_.init();
  }

  void verifyTerminal(const State& goal)
  {
    ASSERT_FALSE(planner_.path_nodes_.empty());
    const State end = planner_.path_nodes_.back()->state;
    EXPECT_LE((end.head<2>() - goal.head<2>()).norm(), planner_.shot_pos_tolerance_);
    const double yaw = end(2) - goal(2);
    EXPECT_LE(std::abs(std::atan2(std::sin(yaw), std::cos(yaw))), planner_.shot_yaw_tolerance_);
    EXPECT_LE(std::abs(end(3) - goal(3)), planner_.shot_u_tolerance_);
    EXPECT_LE(std::abs(end(4) - goal(4)), planner_.shot_v_tolerance_);
    EXPECT_LE(std::abs(end(5) - goal(5)), planner_.shot_r_tolerance_);
  }

  void verifyWarmReplay()
  {
    const Eigen::Vector3d goal(0., 40., std::acos(-1.));
    verifySearch({0., 0., 0.}, {1.5, 0., 0.}, goal);
    ASSERT_GT(planner_.path_nodes_.size(), 1u);
    const auto* first = planner_.path_nodes_[1];
    const State measured = usv_dynamics::integrateRK4(
      first->parent->state, first->input, std::min(0.2, first->duration), 5);
    planner_.cacheSuccessfulPath();
    planner_.reset();
    const int status = planner_.search(measured.head<3>(), measured.tail<3>(), first->input,
      goal, Eigen::Vector3d::Zero(), false);
    ASSERT_EQ(status, KinodynamicAstar::REACH_END);
    EXPECT_EQ(planner_.iter_num_, 0);
    ASSERT_FALSE(planner_.path_nodes_.empty());
    EXPECT_NEAR((planner_.path_nodes_.front()->state - measured).norm(), 0.0, 1e-10);
    State terminal; terminal << goal, Eigen::Vector3d::Zero();
    verifyTerminal(terminal);
    planner_.cacheSuccessfulPath();
    planner_.current_goal_(0) += 5.0;
    EXPECT_EQ(planner_.seedPreviousPath(planner_.path_nodes_.front()), planner_.path_nodes_.front());
    EXPECT_TRUE(planner_.previous_path_.empty());
  }

  void verifyBlockedWarmReplay()
  {
    verifySearch({0., 0., 0.}, {1.5, 0., 0.}, {0., 40., std::acos(-1.)});
    const State measured = planner_.path_nodes_.front()->state;
    const auto samples = planner_.getKinoTraj(0.1);
    ASSERT_GT(samples.size(), 10u);
    const Eigen::Vector3d obstacle(samples[samples.size() / 3](0),
      samples[samples.size() / 3](1), 0.0);
    planner_.cacheSuccessfulPath();
    auto publisher = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.5;
    grid.info.width = grid.info.height = 600;
    grid.info.origin.position.x = grid.info.origin.position.y = -150.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(600 * 600, 0);
    const int x = static_cast<int>((obstacle(0) + 150.0) / 0.5);
    const int y = static_cast<int>((obstacle(1) + 150.0) / 0.5);
    grid.data[y * grid.info.width + x] = 100;
    publisher->publish(grid);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((planner_.environment_->getInflateOccupancy(obstacle) == 0 ||
            !planner_.environment_->isReady()) &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_NE(planner_.environment_->getInflateOccupancy(obstacle), 0);
    planner_.reset();
    auto root = planner_.path_node_pool_[planner_.use_node_num_++];
    root->state = measured;
    root->parent = nullptr;
    root->g_score = 0.0;
    planner_.search_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    const auto seed = planner_.seedPreviousPath(root);
    EXPECT_NE(seed, root);
    EXPECT_FALSE(planner_.terminalState(seed->state));
    for (auto node = seed; node != root; node = node->parent) {
      const Eigen::Vector3d point(node->state(0), node->state(1), 0.0);
      EXPECT_EQ(planner_.environment_->getInflateOccupancy(point), 0);
      EXPECT_GE(planner_.environment_->getDistance(point), planner_.search_clearance_);
    }
  }

  void verifyNominalCacheDoesNotDrift()
  {
    const Eigen::Vector3d goal(0., 40., std::acos(-1.));
    verifySearch({0., 0., 0.}, {1.5, 0., 0.}, goal);
    planner_.cacheSuccessfulPath();
    const auto endpoint = [this]() -> State {
      const auto& last = planner_.previous_path_.back();
      return usv_dynamics::integrateRK4(last.start, last.command, last.duration,
        std::max(1, static_cast<int>(std::ceil(last.duration / 0.04))));
    };
    const State original_end = endpoint();
    for (int i = 0; i < 30; ++i) {
      SCOPED_TRACE(i);
      const auto first = planner_.previous_path_.front();
      State measured = usv_dynamics::integrateRK4(first.start, first.command,
        std::min(0.1, first.duration), 5);
      measured(0) += 0.02;
      planner_.reset();
      ASSERT_EQ(planner_.search(measured.head<3>(), measured.tail<3>(), first.command,
        goal, Eigen::Vector3d::Zero(), false), KinodynamicAstar::REACH_END);
      ASSERT_EQ(planner_.iter_num_, 0);
      EXPECT_NEAR((planner_.path_nodes_.front()->state - measured).norm(), 0., 1e-10);
      planner_.cacheSuccessfulPath();
      EXPECT_NEAR((endpoint() - original_end).norm(), 0., 1e-5);
    }
  }

  void verifyShotReplayKeepsOriginalExtent()
  {
    const State start = (State() << 0., 0., 0., 1.4, 0., 0.).finished();
    const Eigen::Vector3d command(1.7, 0., 0.);
    const State goal = usv_dynamics::integrateRK4(start, command, 8., 200);
    planner_.horizon_ = 5.;
    planner_.previous_path_ = {{start, command, 8., true}};
    planner_.previous_goal_ = goal;
    planner_.reset();
    ASSERT_EQ(planner_.search(start.head<3>(), start.tail<3>(), command, goal.head<3>(),
      goal.tail<3>(), false), KinodynamicAstar::REACH_END);
    EXPECT_EQ(planner_.iter_num_, 0);
    verifyTerminal(goal);
  }

  void verifyRollingReplayExtendsHorizon()
  {
    const State start = (State() << 0., 0., 0., 1.4, 0., 0.).finished();
    const Eigen::Vector3d command(1.7, 0., 0.);
    const Eigen::Vector3d goal(0., 120., 0.);
    planner_.horizon_ = 5.;
    planner_.previous_path_ = {{start, command, 2., false}};
    planner_.previous_goal_ << goal, Eigen::Vector3d::Zero();
    planner_.reset();
    ASSERT_EQ(planner_.search(start.head<3>(), start.tail<3>(), command, goal,
      Eigen::Vector3d::Zero(), false), KinodynamicAstar::REACH_HORIZON);
    ASSERT_GT(planner_.path_nodes_.size(), 2u);
    const State cached_end = usv_dynamics::integrateRK4(start, command, 2., 100);
    EXPECT_NEAR((planner_.path_nodes_[4]->state - cached_end).norm(), 0., 1e-5);
    EXPECT_GE((planner_.path_nodes_.back()->state.head<2>() - start.head<2>()).norm(),
      planner_.horizon_);
    EXPECT_FALSE(planner_.terminalState(planner_.path_nodes_.back()->state));
  }

  void verifyRollingReplayStopsInBrakingZone()
  {
    const State start = (State() << 0., 0., 0., 1.4, 0., 0.).finished();
    const Eigen::Vector3d command(1.7, 0., 0.);
    planner_.current_goal_ << 0., 60., std::acos(-1.), 0., 0., 0.;
    planner_.previous_goal_ = planner_.current_goal_;
    planner_.previous_path_ = {{start, command, 40., false}};
    planner_.reset();
    auto root = planner_.path_node_pool_[planner_.use_node_num_++];
    root->state = start;
    root->parent = nullptr;
    root->g_score = 0.0;
    planner_.search_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    const auto seed = planner_.seedPreviousPath(root);
    ASSERT_NE(seed, root);
    const double distance = (seed->state.head<2>() - planner_.current_goal_.head<2>()).norm();
    EXPECT_LE(distance, planner_.shotTriggerDistance(seed->state));
    EXPECT_GT(distance, planner_.shotTriggerDistance(seed->state) -
        planner_.max_ground_speed_ * planner_.max_tau_);
    EXPECT_FALSE(planner_.terminalState(seed->state));
    EXPECT_EQ(planner_.replay_end_, nullptr);
    ASSERT_NE(seed->parent, root);
    EXPECT_GT((seed->parent->state.head<2>() - planner_.current_goal_.head<2>()).norm(),
        planner_.shotTriggerDistance(seed->parent->state));

    planner_.reset();
    root = planner_.path_node_pool_[planner_.use_node_num_++];
    root->state = start;
    root->state(1) = 45.;
    root->parent = nullptr;
    root->g_score = 0.0;
    planner_.previous_path_ = {{root->state, command, 20., false}};
    EXPECT_EQ(planner_.seedPreviousPath(root), root);
  }

  bool shot(const State& start, const State& goal)
  {
    planner_.reset();
    auto root = planner_.path_node_pool_[planner_.use_node_num_++];
    root->state = start;
    root->parent = nullptr;
    return planner_.computeShotTraj(root, goal, 0.0);
  }

  void verifyTerminalSettlingSeed()
  {
    State start, goal;
    start << 55.12168197, 76.13341858, -1.29101795, 0.73373205, 0.01369572, 0.02410472;
    goal << 55., 83.92304845, 2.61799388, 0., 0., 0.;
    ASSERT_TRUE(shot(start, goal));
    verifyTerminal(goal);
    double duration = 0.0;
    for (size_t i = 1; i < planner_.path_nodes_.size(); ++i)
      duration += planner_.path_nodes_[i]->duration;
    EXPECT_LE(duration, 48.);
    const auto samples = planner_.getKinoTraj(0.04);
    double total_turn = 0.0;
    for (size_t i = 1; i < samples.size(); ++i)
      total_turn += std::abs(samples[i](2) - samples[i - 1](2));
    EXPECT_LT(total_turn, 2. * std::acos(-1.));
  }

  void verifyCruiseSpeedTerminalStop()
  {
    planner_.min_turn_radius_ = 8.0;
    planner_.shot_cruise_speed_ = 3.0;
    planner_.shot_max_u_ = 3.0;
    planner_.shot_max_v_cmd_ = 5.0;
    for (double heading : {0.0, -1.5707963267948966, 2.0943951023931953}) {
      SCOPED_TRACE(heading);
      State start, goal;
      start << 0., 0., heading, 3., 0., 0.;
      goal << -30. * std::sin(heading), 30. * std::cos(heading), heading, 0., 0., 0.;
      planner_.search_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
      ASSERT_TRUE(shot(start, goal));
      verifyTerminal(goal);
    }
  }

  void verifySearch(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& goal)
  {
    planner_.reset();
    const auto began = std::chrono::steady_clock::now();
    const auto deadline = began + std::chrono::milliseconds(400);
    const auto command = usv_dynamics::commandFromBodyAccelerations(
      velocity(0), velocity(1), velocity(2), 0., 0.);
    int status = planner_.search(start, velocity, command, goal,
      Eigen::Vector3d::Zero(), true, false, -1., deadline);
    if (status == KinodynamicAstar::NO_PATH) {
      planner_.reset();
      status = planner_.search(start, velocity, command, goal,
        Eigen::Vector3d::Zero(), false, false, -1., deadline);
    }
    std::cout << "Fixed-input search wall ms: " << std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count() << std::endl;
    ASSERT_EQ(status, KinodynamicAstar::REACH_END);
    State terminal; terminal << goal, Eigen::Vector3d::Zero();
    verifyTerminal(terminal);
  }
};

TEST_F(KinodynamicAstarTest, TerminalTurnShot)
{
  for (double heading : {0.0, 1.0471975512, 2.0943951024, 3.1415926536}) {
    SCOPED_TRACE(heading);
    State start, goal;
    start << 0., 0., 0.37, 1.354, 0.074, 0.071;
    goal << 0., 15.7, heading, 0., 0., 0.;
    ASSERT_TRUE(shot(start, goal));
    verifyTerminal(goal);
  }
}

TEST_F(KinodynamicAstarTest, TerminalSeedLeavesTimeForYawRateToSettle)
{
  verifyTerminalSettlingSeed();
}

TEST_F(KinodynamicAstarTest, CruiseSpeedReachesStationaryTerminal)
{
  verifyCruiseSpeedTerminalStop();
}

TEST_F(KinodynamicAstarTest, ReproducesFormerNodePoolExhaustion)
{
  verifySearch({-7.86827, 53.1134, 0.00189954}, {2., 0.0032771, 0.000121291},
    {-8., 98., std::acos(-1.)});
}

TEST_F(KinodynamicAstarTest, ConnectsBeforeBrakingHeuristicPlateau)
{
  verifySearch({-7.87059, 35.5147, 0.0014501}, {2., 0.00291787, -0.000596631},
    {-8., 98., 0.});
}

TEST_F(KinodynamicAstarTest, ReplaysPreviousManeuverFromMeasuredState)
{
  verifyWarmReplay();
}

TEST_F(KinodynamicAstarTest, RevalidatesCachedCommandsAfterMapUpdate)
{
  verifyBlockedWarmReplay();
}

TEST_F(KinodynamicAstarTest, RepeatedReplayPreservesNominalTerminalManeuver)
{
  verifyNominalCacheDoesNotDrift();
}

TEST_F(KinodynamicAstarTest, ShotReplayDoesNotClipAnAcceptedTerminalManeuver)
{
  verifyShotReplayKeepsOriginalExtent();
}

TEST_F(KinodynamicAstarTest, RollingReplayRetainsSafePrefixAndExtendsHorizon)
{
  verifyRollingReplayExtendsHorizon();
}

TEST_F(KinodynamicAstarTest, RollingReplayDoesNotSkipTerminalBrakingZone)
{
  verifyRollingReplayStopsInBrakingZone();
}

TEST_F(KinodynamicAstarTest, ExpiredBudgetReturnsNoTrajectory)
{
  const int status = planner_.search({0., 0., 0.}, {1., 0., 0.}, {1., 0., 0.},
    {0., 50., 0.}, Eigen::Vector3d::Zero(), false, false, -1.,
    std::chrono::steady_clock::now() - std::chrono::milliseconds(1));
  EXPECT_EQ(status, KinodynamicAstar::TIMEOUT);
  EXPECT_TRUE(planner_.getKinoTraj(0.1).empty());
}
}  // namespace fast_planner
