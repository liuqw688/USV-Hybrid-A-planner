/**
 * @file test_grid_map_costs.cpp
 * @brief 验证航道代价查询与局部障碍融合。
 */
#include "plan_env/sdf_map.h"

#include <gtest/gtest.h>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

namespace fast_planner
{
namespace
{
nav_msgs::msg::OccupancyGrid makeGrid(const int width, const int height, const int8_t value)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 1.0;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width * height), value);
  return grid;
}

bool spinUntil(const std::shared_ptr<rclcpp::Node>& node,
               const std::function<bool()>& condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    if (condition())
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

TEST(SDFMapCostMap, LocalFreeObservationsPreserveFourLevelBaseCosts)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sdf_map/use_grid_map", true),
      rclcpp::Parameter("sdf_map/grid_topic", "/test_channel_costmap"),
      rclcpp::Parameter("sdf_map/grid_local_topic", "/test_local_map"),
      rclcpp::Parameter("sdf_map/grid_local_range", 20.0),
      rclcpp::Parameter("sdf_map/obstacles_inflation", 0.0)});
  auto node = std::make_shared<rclcpp::Node>("grid_cost_regression", options);
  auto map = std::make_shared<SDFMap>();
  map->initMap(node);
  auto base_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/test_channel_costmap", rclcpp::QoS(1).reliable().transient_local());
  auto local_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/test_local_map", rclcpp::SensorDataQoS());
  auto odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
      "/sdf_map/odom", rclcpp::SensorDataQoS());

  auto base = makeGrid(40, 40, 10);
  base.data[5 * 40 + 5] = 0;
  base.data[5 * 40 + 7] = 20;
  base.data[5 * 40 + 8] = 100;
  base_publisher->publish(base);
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 20.5;
  odometry.pose.pose.position.y = 20.5;
  odom_publisher->publish(odometry);
  ASSERT_TRUE(spinUntil(node, [&map]() {return map->isMapReady();}));

  auto local = makeGrid(40, 40, -1);
  local.data[5 * 40 + 5] = 0;
  local.data[5 * 40 + 7] = 0;
  local.data[10 * 40 + 12] = 100;
  local_publisher->publish(local);
  const Eigen::Vector3d same_direction(5.5, 5.5, 0.0);
  const Eigen::Vector3d opposite_direction(7.5, 5.5, 0.0);
  const Eigen::Vector3d temporary_obstacle(12.5, 10.5, 0.0);
  ASSERT_TRUE(spinUntil(node, [&map, &temporary_obstacle]() {
    return map->getTravelCost(temporary_obstacle) == 100.0;
  }));
  EXPECT_DOUBLE_EQ(map->getTravelCost(same_direction), 0.0);
  EXPECT_DOUBLE_EQ(map->getTravelCost(Eigen::Vector3d(6.5, 5.5, 0.0)), 10.0);
  EXPECT_DOUBLE_EQ(map->getTravelCost(opposite_direction), 20.0);
  EXPECT_NE(map->getInflateOccupancy(temporary_obstacle), 0);

  local.data[10 * 40 + 12] = 0;
  local_publisher->publish(local);
  ASSERT_TRUE(spinUntil(node, [&map, &temporary_obstacle]() {
    return map->getTravelCost(temporary_obstacle) == 10.0;
  }));
  EXPECT_DOUBLE_EQ(map->getTravelCost(same_direction), 0.0);
  EXPECT_DOUBLE_EQ(map->getTravelCost(opposite_direction), 20.0);
}
TEST(SDFMapCostMap, InterpolatedTravelGradientRespectsRotatedGrid)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  for (double yaw : {0.0, 0.7}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("sdf_map/use_grid_map", true),
      rclcpp::Parameter("sdf_map/grid_topic", "/test_gradient_map")});
    auto node = std::make_shared<rclcpp::Node>("grid_gradient_regression", options);
    auto map = std::make_shared<SDFMap>();
    map->initMap(node);
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/test_gradient_map", rclcpp::QoS(1).reliable().transient_local());
    auto grid = makeGrid(10, 10, 0);
    grid.info.origin.position.x = -8.0;
    grid.info.origin.position.y = 4.0;
    grid.info.origin.orientation.w = std::cos(0.5*yaw);
    grid.info.origin.orientation.z = std::sin(0.5*yaw);
    for (int y = 0; y < 10; ++y)
      for (int x = 0; x < 10; ++x) grid.data[y*10+x] = 5+x+2*y;
    publisher->publish(grid);
    ASSERT_TRUE(spinUntil(node, [&map]() {return map->isMapReady();}));
    const Eigen::Vector3d point(-8.0+3.2*std::cos(yaw)-4.1*std::sin(yaw),
      4.0+3.2*std::sin(yaw)+4.1*std::cos(yaw), 0.0);
    double cost;
    Eigen::Vector3d gradient;
    map->evaluateTravelCostWithGradient(point, cost, gradient);
    EXPECT_NEAR(cost, 14.9, 1e-10);
    EXPECT_NEAR(gradient.x(), std::cos(yaw)-2.0*std::sin(yaw), 1e-10);
    EXPECT_NEAR(gradient.y(), std::sin(yaw)+2.0*std::cos(yaw), 1e-10);
    EXPECT_DOUBLE_EQ(gradient.z(), 0.0);
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero(); delta(axis) = 1e-5;
      EXPECT_NEAR(gradient(axis), (map->getTravelCost(point+delta)-
        map->getTravelCost(point-delta))/2e-5, 1e-8);
    }
  }
}

TEST(SDFMapCostMap, InputObstacleDistanceIsSeparateFromPlannerInflation)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sdf_map/use_grid_map", true),
      rclcpp::Parameter("sdf_map/grid_topic", "/test_distance_map"),
      rclcpp::Parameter("sdf_map/obstacles_inflation", 3.0)});
  auto node = std::make_shared<rclcpp::Node>("grid_distance_regression", options);
  auto map = std::make_shared<SDFMap>();
  map->initMap(node);
  auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/test_distance_map", rclcpp::QoS(1).reliable().transient_local());
  auto grid = makeGrid(40, 40, 0);
  grid.data[20 * 40 + 20] = 100;
  publisher->publish(grid);
  ASSERT_TRUE(spinUntil(node, [&map]() {return map->isMapReady();}));
  const Eigen::Vector3d point(25.5, 20.5, 0.0);
  EXPECT_GT(map->getObstacleDistance(point), 4.0);
  EXPECT_LT(map->getObstacleDistance(point), 5.0);
  EXPECT_LT(map->getDistance(point), map->getObstacleDistance(point) - 2.0);
  EXPECT_DOUBLE_EQ(map->getObstacleDistance(Eigen::Vector3d(-1., 20., 0.)), 0.0);
  grid.data[20 * 40 + 20] = 0;
  publisher->publish(grid);
  ASSERT_TRUE(spinUntil(node, [&map, &point]() {return map->getObstacleDistance(point) > 8.0;}));
}
}  // namespace
}  // namespace fast_planner
