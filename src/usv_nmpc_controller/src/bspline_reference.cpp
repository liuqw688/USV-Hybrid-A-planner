/**
 * @file bspline_reference.cpp
 * @brief USV B 样条校验、求值和 NMPC 参考点生成。
 */

#include "usv_nmpc_controller/bspline_reference.hpp"

#include "usv_nmpc_controller/coordinate_conventions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace usv_nmpc
{
namespace
{
using Point = std::vector<double>;

/** @brief 非均匀 B 样条；点的维数由控制点决定。 */
class Curve
{
public:
  Curve(int degree, std::vector<Point> controls, std::vector<double> knots)
  : degree_(degree), controls_(std::move(controls)), knots_(std::move(knots)) {}

  double startTime() const {return knots_[static_cast<std::size_t>(degree_)];}
  double endTime() const
  {
    return knots_[controls_.size()];
  }

  Point evaluate(double time) const
  {
    const int last = static_cast<int>(controls_.size()) - 1;
    const double t = std::clamp(time, startTime(), endTime());
    int span = degree_;
    if (t >= endTime()) {
      span = last;
    } else {
      while (span < last && t >= knots_[static_cast<std::size_t>(span + 1)]) {
        ++span;
      }
    }

    std::vector<Point> work(static_cast<std::size_t>(degree_ + 1));
    for (int j = 0; j <= degree_; ++j) {
      work[static_cast<std::size_t>(j)] =
        controls_[static_cast<std::size_t>(span - degree_ + j)];
    }
    for (int level = 1; level <= degree_; ++level) {
      for (int j = degree_; j >= level; --j) {
        const int left = span - degree_ + j;
        const int right = span + j - level + 1;
        const double denominator = knots_[static_cast<std::size_t>(right)] -
          knots_[static_cast<std::size_t>(left)];
        const double alpha = denominator > 1e-12 ?
          (t - knots_[static_cast<std::size_t>(left)]) / denominator : 0.0;
        auto & current = work[static_cast<std::size_t>(j)];
        const auto & previous = work[static_cast<std::size_t>(j - 1)];
        for (std::size_t axis = 0; axis < current.size(); ++axis) {
          current[axis] = (1.0 - alpha) * previous[axis] + alpha * current[axis];
        }
      }
    }
    return work.back();
  }

  Curve derivative() const
  {
    std::vector<Point> controls;
    controls.reserve(controls_.size() - 1U);
    for (std::size_t i = 0; i + 1U < controls_.size(); ++i) {
      const double denominator = knots_[i + static_cast<std::size_t>(degree_) + 1U] -
        knots_[i + 1U];
      const double scale = denominator > 1e-12 ?
        static_cast<double>(degree_) / denominator : 0.0;
      Point point(controls_[i].size(), 0.0);
      for (std::size_t axis = 0; axis < point.size(); ++axis) {
        point[axis] = scale * (controls_[i + 1U][axis] - controls_[i][axis]);
      }
      controls.push_back(std::move(point));
    }
    std::vector<double> knots(knots_.begin() + 1, knots_.end() - 1);
    return Curve(degree_ - 1, std::move(controls), std::move(knots));
  }

private:
  int degree_;
  std::vector<Point> controls_;
  std::vector<double> knots_;
};

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(),
           [](double value) {return std::isfinite(value);});
}

}  // namespace

struct BsplineReference::Data
{
  Data(
    int degree_in, std::vector<Point> controls_in, std::vector<double> knots_in,
    double start_in, double end_in, std::uint64_t mission_id_in)
  : degree(degree_in), knots(std::move(knots_in)),
    pose_curve(degree, std::move(controls_in), knots),
    derivative_curve(pose_curve.derivative()), start(start_in), end(end_in),
    mission_id(mission_id_in)
  {
    const auto endpoint_derivative = derivative_curve.evaluate(end);
    stationary_endpoint =
      std::hypot(endpoint_derivative[0], endpoint_derivative[1]) <= 0.05 &&
      std::abs(endpoint_derivative[2]) <= 0.02;
  }

  int degree;
  std::vector<double> knots;
  Curve pose_curve;
  Curve derivative_curve;
  double start;
  double end;
  std::uint64_t mission_id;
  bool stationary_endpoint{false};
};

bool BsplineReference::configure(
  const usv_interfaces::msg::UsvBspline & message, std::string & error)
{
  data_.reset();
  error.clear();
  const int degree = message.order;
  const std::size_t point_count = message.control_points.size();
  if (degree < 1 || degree > 5 || point_count <= static_cast<std::size_t>(degree)) {
    error = "order 必须为 1..5，且控制点数量必须大于 order";
    return false;
  }
  if (message.knots.size() != point_count + static_cast<std::size_t>(degree) + 1U) {
    error = "knots 数量必须等于 control_points 数量 + order + 1";
    return false;
  }

  std::vector<double> knots(message.knots.begin(), message.knots.end());
  if (!allFinite(knots) || !std::is_sorted(knots.begin(), knots.end())) {
    error = "knots 必须有限且单调不减";
    return false;
  }
  const double start_time = knots[static_cast<std::size_t>(degree)];
  const double end_time = knots[point_count];
  if (end_time - start_time <= 1e-6) {
    error = "B 样条有效时间区间为空";
    return false;
  }

  std::vector<Point> pose_controls;
  pose_controls.reserve(point_count);
  for (const auto & point : message.control_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.phi)) {
      error = "B 样条控制点包含非有限值";
      return false;
    }
    pose_controls.push_back({point.x, point.y, point.phi});
  }
  // 航向采用模型导航角：0=北、-pi/2=东。三个量共用同一节点向量。
  for (std::size_t i = 1; i < pose_controls.size(); ++i) {
    pose_controls[i][2] = coordinates::unwrapNear(
      pose_controls[i][2], pose_controls[i - 1U][2]);
  }

  auto data = std::make_shared<Data>(degree, std::move(pose_controls),
      std::move(knots), start_time, end_time,
      static_cast<std::uint64_t>(message.traj_id));
  data_ = std::move(data);
  return true;
}

double BsplineReference::duration() const
{
  return data_ ? data_->end - data_->start : 0.0;
}

bool BsplineReference::sample(double elapsed_s, RefPoint & reference) const
{
  if (!data_ || !std::isfinite(elapsed_s)) {
    return false;
  }
  const double duration_s = data_->end - data_->start;
  const double elapsed = std::max(0.0, elapsed_s);
  const double spline_elapsed = std::min(elapsed, duration_s);
  const double spline_time = data_->start + spline_elapsed;
  auto pose = data_->pose_curve.evaluate(spline_time);
  auto derivative = data_->derivative_curve.evaluate(spline_time);
  if (elapsed > duration_s) {
    const double extension = elapsed - duration_s;
    pose[0] += derivative[0] * extension;
    pose[1] += derivative[1] * extension;
    pose[2] += derivative[2] * extension;
  }

  const double psi = pose[2];
  const double east_rate = derivative[0];
  const double north_rate = derivative[1];
  const double yaw_rate = derivative[2];
  const double u = -std::sin(psi) * east_rate + std::cos(psi) * north_rate;
  const double v = std::cos(psi) * east_rate + std::sin(psi) * north_rate;
  const double ground_speed = std::hypot(east_rate, north_rate);
  const double course = ground_speed > 1e-9 ?
    std::atan2(-east_rate, north_rate) : psi;
  reference = {
    pose[0], pose[1], psi, u, v, yaw_rate,
    ground_speed > 1e-9 ? yaw_rate / ground_speed : 0.0, course};
  return std::isfinite(reference.x) && std::isfinite(reference.y) &&
         std::isfinite(reference.psi) && std::isfinite(reference.u) &&
         std::isfinite(reference.v) && std::isfinite(reference.r);
}

bool BsplineReference::buildHorizon(
  double elapsed_s, int horizon_steps, double dt,
  std::vector<RefPoint> & references) const
{
  references.clear();
  if (!data_ || !std::isfinite(elapsed_s) || !std::isfinite(dt) ||
    elapsed_s < 0.0 || horizon_steps < 1 || dt <= 0.0)
  {
    return false;
  }
  references.reserve(static_cast<std::size_t>(horizon_steps + 1));
  for (int k = 0; k <= horizon_steps; ++k) {
    RefPoint point;
    if (!sample(elapsed_s + static_cast<double>(k) * dt, point)) {
      references.clear();
      return false;
    }
    references.push_back(point);
  }
  return true;
}

bool BsplineReference::buildTrajectory(
  double sample_period_s, ReferenceTrajectory & trajectory,
  std::string & error) const
{
  trajectory = {};
  error.clear();
  if (!data_) {
    error = "B 样条尚未通过校验";
    return false;
  }
  if (!std::isfinite(sample_period_s) || sample_period_s <= 0.0) {
    error = "参考离散采样周期必须是有限正数";
    return false;
  }

  const double duration_s = duration();
  const double count_as_double = std::ceil(duration_s / sample_period_s) + 1.0;
  if (!std::isfinite(count_as_double) || count_as_double > 50000.0) {
    error = "B 样条过长，采样点超过 50000";
    return false;
  }
  const std::size_t sample_count = static_cast<std::size_t>(count_as_double);
  trajectory.mission_id = data_->mission_id;
  trajectory.terminal = data_->stationary_endpoint;
  trajectory.points.reserve(sample_count);
  double previous_x = 0.0;
  double previous_y = 0.0;
  bool have_previous = false;

  for (std::size_t i = 0; i < sample_count; ++i) {
    const double elapsed = std::min(
      duration_s, static_cast<double>(i) * sample_period_s);
    RefPoint reference;
    if (!sample(elapsed, reference)) {
      error = "B 样条求值产生非有限参考状态";
      trajectory = {};
      return false;
    }
    const double path_speed = std::hypot(reference.u, reference.v);
    const double signed_speed = reference.u < 0.0 ? -path_speed : path_speed;
    const double arc_length = have_previous ? trajectory.points.back().s +
      std::hypot(reference.x - previous_x, reference.y - previous_y) : 0.0;
    trajectory.points.push_back({
        elapsed, arc_length, reference.x, reference.y, reference.psi,
        signed_speed, reference.curvature, reference.r, reference.course,
        path_speed > 1e-9 ? reference.u / signed_speed : 0.0,
        path_speed > 1e-9 ? reference.v / signed_speed : 0.0});
    previous_x = reference.x;
    previous_y = reference.y;
    have_previous = true;
  }

  if (trajectory.points.empty() || trajectory.points.back().t < duration_s - 1e-9) {
    RefPoint endpoint;
    if (!sample(duration_s, endpoint)) {
      error = "B 样条终点求值失败";
      trajectory = {};
      return false;
    }
    const double arc_length = trajectory.points.back().s +
      std::hypot(endpoint.x - previous_x, endpoint.y - previous_y);
    const double path_speed = std::hypot(endpoint.u, endpoint.v);
    const double signed_speed = endpoint.u < 0.0 ? -path_speed : path_speed;
    trajectory.points.push_back({
        duration_s, arc_length, endpoint.x, endpoint.y, endpoint.psi,
        signed_speed, endpoint.curvature, endpoint.r, endpoint.course,
        path_speed > 1e-9 ? endpoint.u / signed_speed : 0.0,
        path_speed > 1e-9 ? endpoint.v / signed_speed : 0.0});
  }
  if (trajectory.points.size() < 2U) {
    error = "B 样条有效采样点不足";
    trajectory = {};
    return false;
  }
  return true;
}

bool buildReferenceFromBspline(
  const usv_interfaces::msg::UsvBspline & message,
  double sample_period_s,
  ReferenceTrajectory & trajectory,
  std::string & error)
{
  BsplineReference evaluator;
  if (!evaluator.configure(message, error)) {
    trajectory = {};
    return false;
  }
  return evaluator.buildTrajectory(sample_period_s, trajectory, error);
}

}  // namespace usv_nmpc
