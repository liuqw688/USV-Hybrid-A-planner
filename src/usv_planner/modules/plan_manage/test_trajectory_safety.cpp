/**
 * @file test_trajectory_safety.cpp
 * @brief 验证旧参考继续执行的必要条件和样条发布前的碰撞检查。
 */
#include <plan_manage/planner_manager.h>
#include <gtest/gtest.h>
#include <path_searching/turning_constraint.hpp>
#include <path_searching/usv_dynamics.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

namespace fast_planner {
class FastPlannerManagerTest : public ::testing::Test {
protected:
  FastPlannerManager manager_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  nav_msgs::msg::OccupancyGrid grid_;

  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("sdf_map/use_grid_map", true)});
    node_ = std::make_shared<rclcpp::Node>("trajectory_safety", options);
    manager_.sdf_map_ = std::make_shared<SDFMap>();
    manager_.sdf_map_->initMap(node_);
    manager_.clock_ = node_->get_clock();
    publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    grid_.header.frame_id = "map";
    grid_.info.resolution = 0.5;
    grid_.info.width = grid_.info.height = 200;
    grid_.info.origin.position.x = grid_.info.origin.position.y = -50.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(200 * 200, 0);
    publisher_->publish(grid_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!manager_.sdf_map_->isMapReady() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(manager_.sdf_map_->isMapReady());
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(12, 3);
    for (int i = 0; i < points.rows(); ++i) points(i, 1) = i;
    manager_.local_data_.position_traj_ = NonUniformBspline(points, 3, 0.5);
    manager_.local_data_.velocity_traj_ = manager_.local_data_.position_traj_.getDerivative();
    manager_.local_data_.acceleration_traj_ = manager_.local_data_.velocity_traj_.getDerivative();
    manager_.pp_.max_u_ = 2.1;
    manager_.pp_.max_v_ = 1.0;
    manager_.pp_.max_r_ = 0.3;
    manager_.local_data_.duration_ = manager_.local_data_.position_traj_.getTimeSum();
    manager_.local_data_.start_time_ = node_->now();
    manager_.local_data_.traj_id_ = 1;
  }

  void verifyNewObstacle() {
    double distance;
    auto& trajectory = manager_.local_data_.position_traj_;
    ASSERT_TRUE(manager_.collisionFree(trajectory, 0., trajectory.getTimeSum(), distance));
    manager_.accepted_terminal_ = true;
    manager_.accepted_goal_ << 0., 10., 0., 0., 0., 0.;
    const Eigen::Vector3d goal = manager_.accepted_goal_.head<3>();
    const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    const Eigen::Vector3d current = trajectory.evaluateDeBoorT(0.0);
    ASSERT_TRUE(manager_.reuseTerminalTrajectory(current, goal, velocity));
    const auto pose = trajectory.evaluateDeBoorT(1.5);
    const int x = static_cast<int>((pose(0) + 50.) / 0.5);
    const int y = static_cast<int>((pose(1) + 50.) / 0.5);
    grid_.data[y * grid_.info.width + x] = 100;
    publisher_->publish(grid_);
    const Eigen::Vector3d point(pose(0), pose(1), 0.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((manager_.sdf_map_->getInflateOccupancy(point) == 0 ||
            !manager_.sdf_map_->isMapReady()) &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_NE(manager_.sdf_map_->getInflateOccupancy(point), 0);
    EXPECT_FALSE(manager_.collisionFree(trajectory, 0., trajectory.getTimeSum(), distance));
    EXPECT_FALSE(manager_.checkTrajCollision(distance, &current));
    EXPECT_FALSE(manager_.reuseTerminalTrajectory(current, goal, velocity));
  }

  void verifyInvalidReference() {
    double distance;
    Eigen::Vector3d current = manager_.local_data_.position_traj_.evaluateDeBoorT(0.6);
    ASSERT_TRUE(manager_.checkTrajCollision(distance, &current));
    const double progress = manager_.collision_check_time_;
    EXPECT_GT(progress, 0.0);
    current(1) -= 0.2;
    EXPECT_TRUE(manager_.checkTrajCollision(distance, &current));
    EXPECT_GE(manager_.collision_check_time_, progress);
    current(0) += 3.0;
    EXPECT_FALSE(manager_.checkTrajCollision(distance, &current));
    manager_.local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
    EXPECT_FALSE(manager_.checkTrajCollision(distance));
  }

  void verifyObstacleSpeedLimit() {
    auto & fast = manager_.local_data_.position_traj_;
    double distance;
    ASSERT_TRUE(manager_.collisionFree(fast, 0., fast.getTimeSum(), distance));
    // 障碍在航迹侧方：无碰撞但进入 5 m 限速范围。
    const int x = static_cast<int>((4.0 + 50.0) / 0.5);
    const int y = static_cast<int>((5.0 + 50.0) / 0.5);
    grid_.data[y * grid_.info.width + x] = 100;
    publisher_->publish(grid_);
    const Eigen::Vector3d nearby(0.0, 5.0, 0.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    const Eigen::Vector3d obstacle(4.25, 5.25, 0.0);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      if (manager_.sdf_map_->isMapReady() &&
          manager_.sdf_map_->getInflateOccupancy(obstacle) != 0 &&
          manager_.sdf_map_->getInflateOccupancy(nearby) == 0 &&
          manager_.sdf_map_->getObstacleDistance(nearby) < 5.0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_NE(manager_.sdf_map_->getInflateOccupancy(obstacle), 0);
    ASSERT_EQ(manager_.sdf_map_->getInflateOccupancy(nearby), 0);
    ASSERT_LT(manager_.sdf_map_->getObstacleDistance(nearby), 5.0);
    EXPECT_FALSE(manager_.collisionFree(fast, 0., fast.getTimeSum(), distance));
    NonUniformBspline slow(fast.getControlPoint(), 3, 1.05);
    EXPECT_TRUE(manager_.collisionFree(slow, 0., slow.getTimeSum(), distance));
  }

  void verifyExpiredTerminalReference() {
    auto & trajectory = manager_.local_data_.position_traj_;
    ASSERT_TRUE(trajectory.enforceEndVelocity(Eigen::Vector3d::Zero()));
    manager_.local_data_.velocity_traj_ = trajectory.getDerivative();
    manager_.local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
    manager_.accepted_terminal_ = true;
    const Eigen::Vector3d end = trajectory.evaluateDeBoorT(manager_.local_data_.duration_);
    double distance;
    EXPECT_TRUE(manager_.checkTrajCollision(distance, &end));
    Eigen::Vector3d far = end;
    far(0) += 2.1;
    EXPECT_FALSE(manager_.checkTrajCollision(distance, &far));
    manager_.accepted_terminal_ = false;
    EXPECT_FALSE(manager_.checkTrajCollision(distance, &end));
  }

  void verifyReplanClock() {
    manager_.local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(1.0);
    Eigen::Vector3d measured = manager_.local_data_.position_traj_.evaluateDeBoorT(1.0);
    measured(1) -= 0.5;
    Eigen::Vector3d pose, speed, accel;
    ASSERT_TRUE(manager_.sampleReplanStart(measured, pose, speed, accel));
    EXPECT_GT(pose(1) - measured(1), 0.49);
    EXPECT_NEAR(speed(1), 2.0, 1e-8);
    measured(0) += 3.0;
    const Eigen::Vector3d accepted = pose;
    EXPECT_FALSE(manager_.sampleReplanStart(measured, pose, speed, accel));
    EXPECT_TRUE(pose.isApprox(accepted));
    measured(0) -= 3.0;
    measured(2) = std::acos(-1.0);
    EXPECT_FALSE(manager_.sampleReplanStart(measured, pose, speed, accel));
    measured(2) = 0.0;
    manager_.local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
    EXPECT_FALSE(manager_.sampleReplanStart(measured, pose, speed, accel));
  }

  void verifyTerminalReuseClock() {
    manager_.accepted_terminal_ = true;
    manager_.accepted_goal_ << 0., 10., 0., 0., 0., 0.;
    const Eigen::Vector3d goal = manager_.accepted_goal_.head<3>();
    const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    const auto begin = manager_.local_data_.start_time_;
    const auto points = manager_.local_data_.position_traj_.getControlPoint();
    for (int i = 0; i < 10; ++i) {
      const auto current = manager_.local_data_.position_traj_.evaluateDeBoorT(0.0);
      ASSERT_TRUE(manager_.reuseTerminalTrajectory(current, goal, velocity));
      EXPECT_EQ(manager_.local_data_.start_time_, begin);
      EXPECT_TRUE(manager_.local_data_.position_traj_.getControlPoint().isApprox(points));
    }
    Eigen::Vector3d changed_goal = goal;
    changed_goal(0) += 1.;
    EXPECT_FALSE(manager_.reuseTerminalTrajectory({0., 1., 0.}, changed_goal, velocity));
    EXPECT_FALSE(manager_.reuseTerminalTrajectory({3., 1., 0.}, goal, velocity));
    manager_.local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(10.0);
    EXPECT_FALSE(manager_.reuseTerminalTrajectory({0., 1., 0.}, goal, velocity));
  }

  void verifyTurnGuard() {
    manager_.min_turn_radius_ = 8.0;
    for (const double radius : {3.0, 12.0}) {
      Eigen::MatrixXd points(43, 3);
      for (int i = 0; i < points.rows(); ++i) {
        const double angle = (i - 1) * 0.08;
        points.row(i) << radius*std::cos(angle), radius*std::sin(angle), angle;
      }
      NonUniformBspline curve(points, 3, 0.5);
      EXPECT_EQ(manager_.turningFeasible(curve), radius > 8.0);
      curve.lengthenTime(2.0);
      EXPECT_EQ(manager_.turningFeasible(curve), radius > 8.0);
    }
    auto straight = manager_.local_data_.position_traj_;
    EXPECT_TRUE(manager_.turningFeasible(straight));
    ASSERT_TRUE(straight.enforceEndVelocity(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(manager_.turningFeasible(straight));
  }

  struct PlanMetrics {
    bool success{false};
    bool reached_terminal{false};
    double displacement{0.0};
    double mean_offset{0.0};
    double turn_ratio{0.0};
    double min_radius{1e9};
    double nearest_obstacle{1e9};
  };

  PlanMetrics runNavigationPlan(const std::string & name, bool centered, bool obstacle,
    double radius, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
    bool terminal, double start_surge = 1.5, bool allow_prefix = false, bool narrow = false) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", USV_PLANNER_TEST_CONFIG});
    options.parameter_overrides({rclcpp::Parameter("manager/min_turn_radius", radius)});
    auto node = std::make_shared<rclcpp::Node>("usv_planner_node", options);
    FastPlannerManager planner;
    planner.initPlanModules(*node);
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/channel_costmap", rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.width = grid.info.height = 400;
    grid.info.resolution = 0.5;
    grid.info.origin.position.x = grid.info.origin.position.y = -60.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(400*400);
    for (int y = 0; y < 400; ++y) {
      for (int x = 0; x < 400; ++x) {
        const double east = -60.0 + (x+0.5)*0.5, north = -60.0 + (y+0.5)*0.5;
        grid.data[y*400+x] = std::abs(east) > 16.0 ? 10 :
          (centered ? static_cast<int>(std::lround(8.0*std::abs(east)/16.0)) : 0);
        if (obstacle && std::hypot(east, north-28.0) < 2.0) grid.data[y*400+x] = 100;
        if (narrow && north > 18.0 && north < 45.0 &&
            std::abs(east) > 4.5 && std::abs(east) < 6.0) grid.data[y*400+x] = 100;
      }
    }
    publisher->publish(grid);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!planner.sdf_map_->isMapReady() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    PlanMetrics metrics;
    if (!planner.sdf_map_->isMapReady()) return metrics;
    const Eigen::Vector3d speed(start_surge, 0.0, 0.0);
    const auto command = usv_dynamics::commandFromBodyAccelerations(start_surge, 0., 0., 0., 0.);
    const auto began = std::chrono::steady_clock::now();
    metrics.success = planner.kinodynamicReplan(start, speed, command, goal,
      terminal ? Eigen::Vector3d::Zero().eval() : Eigen::Vector3d(1.,0.,0.));
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-began).count();
    if (!metrics.success) return metrics;
    EXPECT_TRUE(planner.trajectoryMatchesGoalHeading(goal.z()));
    EXPECT_TRUE(planner.trajectoryMatchesGoalHeading(goal.z() + 2.0 * std::acos(-1.0)));
    EXPECT_FALSE(planner.trajectoryMatchesGoalHeading(goal.z() + std::acos(-1.0)));
    metrics.reached_terminal = planner.accepted_terminal_;
    auto & curve = planner.local_data_.position_traj_;
    auto velocity = curve.getDerivative();
    auto acceleration = velocity.getDerivative();
    std::ofstream csv;
    if (const char * output = std::getenv("USV_NAVIGATION_TEST_OUTPUT")) {
      std::filesystem::create_directories(output);
      csv.open(std::filesystem::path(output)/(name+".csv"));
      csv << "t,x,y,phi,vx,vy,r,ax,ay,alpha,cost,clearance\n";
    }
    const int count = std::max(1, static_cast<int>(std::ceil(curve.getTimeSum()/0.025)));
    int offset_count = 0;
    double nearest_obstacle = std::numeric_limits<double>::infinity();
    double max_near_obstacle_speed = 0.0;
    for (int i = 0; i <= count; ++i) {
      const double t = curve.getTimeSum()*i/count;
      const Eigen::Vector3d p = curve.evaluateDeBoorT(t), v = velocity.evaluateDeBoorT(t),
        a = acceleration.evaluateDeBoorT(t);
      metrics.turn_ratio = std::max(metrics.turn_ratio,
        turningRatios(v, a, radius).cwiseAbs().maxCoeff());
      const double speed = v.head<2>().norm();
      const double cross = std::abs(v.x()*a.y()-v.y()*a.x());
      if (speed >= kTurningSpeedFloor && cross > 1e-9)
        metrics.min_radius = std::min(metrics.min_radius, speed*speed*speed/cross);
      if (t > 5.0) {metrics.mean_offset += std::abs(p.x()); ++offset_count;}
      const Eigen::Vector3d map_point(p.x(), p.y(), 0.0);
      EXPECT_EQ(planner.sdf_map_->getInflateOccupancy(map_point), 0);
      const double obstacle_distance = planner.sdf_map_->getObstacleDistance(map_point);
      nearest_obstacle = std::min(nearest_obstacle, obstacle_distance);
      if (obstacle_distance <= 5.0) {
        max_near_obstacle_speed = std::max(max_near_obstacle_speed, speed);
        EXPECT_LE(speed, 1.0 + 1e-9);
      }
      if (csv) csv << t << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
        << v.x() << ',' << v.y() << ',' << v.z() << ',' << a.x() << ',' << a.y()
        << ',' << a.z() << ',' << planner.sdf_map_->getTravelCost(map_point) << ','
        << planner.sdf_map_->getDistance(map_point) << '\n';
    }
    metrics.mean_offset /= std::max(1, offset_count);
    metrics.nearest_obstacle = nearest_obstacle;
    EXPECT_LE(metrics.turn_ratio, 1.00501);
    const Eigen::Vector3d end = curve.evaluateDeBoorT(curve.getTimeSum());
    EXPECT_LT(std::abs(std::atan2(std::sin(end.z()-goal.z()), std::cos(end.z()-goal.z()))),
      std::acos(-1.)/18. + 1e-9);
    metrics.displacement = (end.head<2>() - start.head<2>()).norm();
    if (terminal && (metrics.reached_terminal || !allow_prefix)) {
      EXPECT_TRUE(metrics.reached_terminal);
      EXPECT_LE((end.head<2>()-goal.head<2>()).norm(), 3.5);
      EXPECT_LE(std::abs(std::atan2(std::sin(end.z()-goal.z()), std::cos(end.z()-goal.z()))),
        std::acos(-1.)/6.);
      EXPECT_LE(velocity.evaluateDeBoorT(curve.getTimeSum()).norm(), 1e-8);
    }
    std::cout << "NAVIGATION_RESULT " << name << " ms=" << wall_ms
      << " mean_offset=" << metrics.mean_offset << " min_radius=" << metrics.min_radius
      << " turn_ratio=" << metrics.turn_ratio << " terminal=" << metrics.reached_terminal
      << " displacement=" << metrics.displacement << " obstacle_distance=" << nearest_obstacle
      << " near_obstacle_max_speed=" << max_near_obstacle_speed << '\n';
    return metrics;
  }
};

TEST_F(FastPlannerManagerTest, RejectsNewObstacleInPreviousReference) { verifyNewObstacle(); }
TEST_F(FastPlannerManagerTest, LimitsSpeedNearNoncollidingObstacle) { verifyObstacleSpeedLimit(); }
TEST_F(FastPlannerManagerTest, RejectsDetachedOrExpiredPreviousReference) { verifyInvalidReference(); }
TEST_F(FastPlannerManagerTest, HoldsOnlyValidStationaryTerminalReference) { verifyExpiredTerminalReference(); }
TEST_F(FastPlannerManagerTest, MissingPreviousReferenceDoesNotReadItsTimestamp) {
  manager_.local_data_.traj_id_ = 0;
  manager_.local_data_.start_time_ = rclcpp::Time(static_cast<int64_t>(0), RCL_SYSTEM_TIME);
  double distance = -1.0;
  EXPECT_FALSE(manager_.checkTrajCollision(distance));
  EXPECT_DOUBLE_EQ(distance, 0.0);
  Eigen::Vector3d position = Eigen::Vector3d::Ones();
  Eigen::Vector3d velocity = position, acceleration = position;
  EXPECT_FALSE(manager_.sampleReplanStart(Eigen::Vector3d::Zero(), position, velocity, acceleration));
  EXPECT_TRUE(position.isOnes());
  EXPECT_TRUE(velocity.isOnes());
  EXPECT_TRUE(acceleration.isOnes());
}
TEST_F(FastPlannerManagerTest, RollingReplanPreservesReferenceClock) { verifyReplanClock(); }
TEST_F(FastPlannerManagerTest, CompleteReferenceReuseNeverRetimesOrIgnoresGoalChange) {
  verifyTerminalReuseClock();
}
TEST_F(FastPlannerManagerTest, FinalSplineTurnGuardAndTimeScaling) {verifyTurnGuard();}
TEST_F(FastPlannerManagerTest, NavigationIntegration) {
  const auto baseline = runNavigationPlan("lane_uniform", false, false, 0.0,
    {6., 0., 0.}, {6., 120., 0.}, false);
  ASSERT_TRUE(baseline.success);
  const auto centered = runNavigationPlan("lane_centered", true, false, 8.0,
    {6., 0., 0.}, {6., 120., 0.}, false);
  EXPECT_TRUE(centered.success);
  if (centered.success) {
    EXPECT_LT(centered.mean_offset, 0.5 * baseline.mean_offset);
  }
  const auto detour = runNavigationPlan("lane_obstacle", true, true, 8.0,
    {0., 0., 0.}, {0., 120., 0.}, false);
  EXPECT_TRUE(detour.success);
  const auto uturn_old = runNavigationPlan("uturn_unlimited", false, false, 0.0,
    {0., 0., 0.}, {0., 40., std::acos(-1.)}, true);
  EXPECT_TRUE(uturn_old.success);
  const auto uturn = runNavigationPlan("uturn_limited", true, false, 8.0,
    {0., 0., 0.}, {0., 40., std::acos(-1.)}, true);
  EXPECT_TRUE(uturn.success);
}

TEST_F(FastPlannerManagerTest, PlansThroughNarrowSlowZone) {
  const auto result = runNavigationPlan("narrow_slow_zone", false, false, 8.0,
    {0., 0., 0.}, {0., 120., 0.}, false, 3.0, false, true);
  EXPECT_TRUE(result.success);
  if (result.success) EXPECT_LE(result.nearest_obstacle, 5.0);
}

TEST_F(FastPlannerManagerTest, TurningAcrossInitialHeadings) {
  const double pi = std::acos(-1.0);
  for (const double heading : {-pi/2.0, pi/2.0, pi}) {
    const auto result = runNavigationPlan("heading_" + std::to_string(heading), true, false, 8.0,
      {0., 0., heading}, {0., 40., 0.}, true, 1.5, true);
    ASSERT_TRUE(result.success) << "initial heading=" << heading;
    EXPECT_GT(result.displacement, 2.0) << "initial heading=" << heading;
  }
  const auto from_rest = runNavigationPlan("turn_from_rest", true, false, 8.0,
    {0., 0., 0.}, {0., 40., pi}, true, 0.0, true);
  ASSERT_TRUE(from_rest.success);
  EXPECT_GT(from_rest.displacement, 2.0);
}

}  // namespace fast_planner
