/**
 * @file test_navigation_cost.cpp
 * @brief 校验含侧滑的转弯约束及实际样条通行代价的解析梯度。
 */
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/turning_constraint.hpp>
#include <gtest/gtest.h>

namespace fast_planner
{
namespace
{
class SmoothCostMap : public SearchEnvironment
{
public:
  bool isReady() const override {return true;}
  bool usesGridMap() const override {return false;}
  void getRegion(Eigen::Vector3d & origin, Eigen::Vector3d & size) override
  {origin.setConstant(-100.0); size.setConstant(200.0);}
  double getResolution() override {return 0.5;}
  int getInflateOccupancy(const Eigen::Vector3d &) override {return 0;}
  double getDistance(const Eigen::Vector3d &) override {return 20.0;}
  double evaluateCoarseDistance(const Eigen::Vector3d &, double) override {return 20.0;}
  void evaluateDistanceWithGradient(const Eigen::Vector3d &, double,
    double & distance, Eigen::Vector3d & gradient) override
  {distance = 20.0; gradient.setZero();}
  void evaluateTravelCostWithGradient(const Eigen::Vector3d & p, double & cost,
    Eigen::Vector3d & gradient) override
  {cost = 0.25*p.x()*p.x(); gradient = {0.5*p.x(), 0.0, 0.0};}
};
}  // namespace

TEST(TurningConstraint, IncludesSideslipAccelerationAndSeparateYawRate)
{
  const auto side = turningRatios({2.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 8.0);
  EXPECT_DOUBLE_EQ(side(0), 2.0);
  EXPECT_DOUBLE_EQ(side(1), 0.0);
  const auto yaw = turningRatios({0.0, 2.0, 0.5}, {0.0, 0.0, 0.0}, 8.0);
  EXPECT_DOUBLE_EQ(yaw(0), 0.0);
  EXPECT_DOUBLE_EQ(yaw(1), 2.0);
  EXPECT_TRUE(turningRatios({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 8.0).allFinite());
  EXPECT_TRUE(turningRatios({2.0, 0.0, 2.0}, {0.0, 4.0, 0.0}, 0.0).isZero());
}

TEST(TurningConstraint, AnalyticGradientMatchesFiniteDifferences)
{
  for (const Eigen::Vector3d velocity : {Eigen::Vector3d(1.2, 0.3, 0.4),
      Eigen::Vector3d(0.08, -0.1, -0.2)}) {
    const Eigen::Vector3d acceleration(-0.7, 1.4, 0.5);
    Eigen::Vector3d gv, ga, unused;
    turningCost(velocity, acceleration, 8.0, gv, ga);
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d dv = Eigen::Vector3d::Zero(); dv(axis) = 1e-6;
      Eigen::Vector3d tmp;
      const double vp = turningCost(velocity+dv, acceleration, 8., tmp, unused);
      const double vm = turningCost(velocity-dv, acceleration, 8., tmp, unused);
      const double ap = turningCost(velocity, acceleration+dv, 8., tmp, unused);
      const double am = turningCost(velocity, acceleration-dv, 8., tmp, unused);
      EXPECT_NEAR(gv(axis), (vp-vm)/2e-6, 1e-3);
      EXPECT_NEAR(ga(axis), (ap-am)/2e-6, 1e-3);
    }
  }
}

class BsplineNavigationTest : public ::testing::Test
{
protected:
  void verifyGradient(double radius)
  {
    BsplineOptimizer optimizer;
    optimizer.setEnvironment(std::make_shared<SmoothCostMap>());
    optimizer.channel_weight_ = 2.0;
    optimizer.min_turn_radius_ = radius;
    optimizer.setBsplineInterval(0.8);
    vector<Eigen::Vector3d> points;
    for (int i = 0; i < 8; ++i)
      points.emplace_back(0.15*i*i, i*0.7, 0.1*i*i);
    vector<Eigen::Vector3d> gradient(points.size()), scratch(points.size());
    double cost;
    optimizer.calcNavigationCost(points, cost, gradient);
    EXPECT_GT(cost, 0.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        auto q = points;
        q[i](axis) += 1e-6;
        double cp, cm;
        optimizer.calcNavigationCost(q, cp, scratch);
        q[i](axis) -= 2e-6;
        optimizer.calcNavigationCost(q, cm, scratch);
        EXPECT_NEAR(gradient[i](axis), (cp-cm)/2e-6, 1e-4);
      }
    }
  }
};
TEST_F(BsplineNavigationTest, TravelCostGradient) {verifyGradient(0.0);}
TEST_F(BsplineNavigationTest, TravelAndCurvatureGradient) {verifyGradient(8.0);}
}  // namespace fast_planner
