/**
 * @file clockwise_route.cpp
 * @brief 周期样条拟合、光滑收窄和安全航道生成。
 */

#include "clockwise_route.hpp"
#include "route_water.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace usv_map_tools
{
namespace
{
// 减少凸包控制点以平滑 GNSS 尖角，保证法向偏移曲线的规则性。
constexpr std::size_t kControlCount = 8;
constexpr std::size_t kMaxSamples = 200000;

double distance(RoutePoint a, RoutePoint b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double turn(RoutePoint a, RoutePoint b, RoutePoint c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

std::vector<RoutePoint> convex_hull(std::vector<RoutePoint> points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("route reference coordinates must be finite");
    }
  }
  std::sort(points.begin(), points.end(), [](auto a, auto b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  points.erase(std::unique(points.begin(), points.end(), [](auto a, auto b) {
    return a.x == b.x && a.y == b.y;
  }), points.end());
  if (points.size() < 3) {
    throw std::invalid_argument("route needs at least three distinct, non-collinear GNSS points");
  }
  std::vector<RoutePoint> hull;
  for (const auto & point : points) {
    while (hull.size() >= 2 && turn(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size && turn(hull[hull.size() - 2], hull.back(), *it) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  hull.pop_back();
  if (hull.size() < 3) {
    throw std::invalid_argument("route needs non-collinear GNSS points");
  }
  // 单调链生成逆时针顶点；反转后用于顺时针航线。
  std::reverse(hull.begin(), hull.end());
  return hull;
}

std::vector<RoutePoint> spline_controls(const std::vector<RoutePoint> & hull)
{
  if (hull.size() <= kControlCount) {
    return hull;
  }
  std::vector<double> arc{0.0};
  for (std::size_t i = 0; i < hull.size(); ++i) {
    arc.push_back(arc.back() + distance(hull[i], hull[(i + 1) % hull.size()]));
  }
  // 按近似等周长选择凸包顶点，保持严格凸性，使周期样条曲率同号并滤除 GNSS 抖动。
  std::vector<RoutePoint> controls;
  std::size_t previous = hull.size();
  for (std::size_t i = 0; i < kControlCount; ++i) {
    const double target = arc.back() * static_cast<double>(i) / kControlCount;
    auto index = static_cast<std::size_t>(
      std::lower_bound(arc.begin(), arc.end(), target) - arc.begin());
    if (index > 0 && target - arc[index - 1] < arc[index] - target) {
      --index;
    }
    if (index < hull.size() && index != previous) {
      controls.push_back(hull[index]);
      previous = index;
    }
  }
  return controls.size() >= 3 ? controls : hull;
}

RoutePose evaluate(const std::vector<RoutePoint> & controls, double parameter)
{
  const auto segment = static_cast<std::size_t>(parameter);
  const double t = parameter - static_cast<double>(segment);
  const double u = 1.0 - t;
  const std::array<double, 4> weights{
    u * u * u / 6.0, (3 * t * t * t - 6 * t * t + 4) / 6.0,
    (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0, t * t * t / 6.0};
  const std::array<double, 4> derivatives{
    -u * u / 2.0, (3 * t * t - 4 * t) / 2.0,
    (-3 * t * t + 2 * t + 1) / 2.0, t * t / 2.0};
  RoutePoint point{0.0, 0.0};
  RoutePoint tangent{0.0, 0.0};
  for (std::size_t i = 0; i < weights.size(); ++i) {
    const auto & control = controls[(segment + i) % controls.size()];
    point.x += weights[i] * control.x;
    point.y += weights[i] * control.y;
    tangent.x += derivatives[i] * control.x;
    tangent.y += derivatives[i] * control.y;
  }
  return {point, std::atan2(tangent.y, tangent.x)};
}

// 导数与 evaluate() 使用同一周期三次样条，确保切向和曲率一致。
std::array<RoutePoint, 3> derivatives(const std::vector<RoutePoint> & controls, double parameter)
{
  const auto segment = static_cast<std::size_t>(parameter);
  const double t = parameter - static_cast<double>(segment), u = 1.0 - t;
  const std::array<std::array<double, 4>, 3> weights{{
    {{-u * u / 2, (3 * t * t - 4 * t) / 2, (-3 * t * t + 2 * t + 1) / 2, t * t / 2}},
    {{u, 3 * t - 2, 1 - 3 * t, t}}, {{-1, 3, -3, 1}}}};
  std::array<RoutePoint, 3> result{};
  for (std::size_t d = 0; d < result.size(); ++d) {
    for (std::size_t i = 0; i < 4; ++i) {
      const auto p = controls[(segment + i) % controls.size()];
      result[d].x += weights[d][i] * p.x;
      result[d].y += weights[d][i] * p.y;
    }
  }
  return result;
}

constexpr double kOffsetError = 0.002;  // 偏移曲线到弦的最大偏差，单位 m。

std::vector<RoutePose> fairway_stations(
  const SafeRoute & route, double width, double spacing_m)
{
  std::vector<RoutePose> result;
  const auto norm = [](RoutePoint p) {return std::hypot(p.x, p.y);};
  const auto & controls = route.controls;
  std::function<void(double, double, int)> subdivide;
  subdivide = [&](double a, double b, int depth) {
      const double h = b - a;
      const auto da = derivatives(controls, a), db = derivatives(controls, b);
      const auto dm = derivatives(controls, (a + b) / 2);
      const double acceleration = std::max(norm(da[1]), norm(db[1]));
      const double speed = norm(dm[0]) - acceleration * h / 2;
      // 单位法向量满足 ||normal''|| <= ||s'''||/v + 3||s''||^2/v^2。
      // 每个节点区间的二阶导数为仿射函数，其范数上界取两端最大值。
      const double curvature_bound = speed > 0 ? acceleration / (speed * speed) :
        std::numeric_limits<double>::infinity();
      const double second = acceleration + width *
        (norm(dm[2]) / speed + 3 * acceleration * acceleration / (speed * speed));
      const bool regular = speed > 0 && width * curvature_bound < 0.98;
      const double max_speed = norm(dm[0]) + acceleration * h / 2;
      if (regular && second * h * h / 8 <= kOffsetError && max_speed * h <= spacing_m) {
        result.push_back(evaluate(controls, a));
        if (result.size() > kMaxSamples) {
          throw std::runtime_error("fairway exceeds geometry verification limit");
        }
      } else {
        if (depth >= 18) {
          throw std::runtime_error("lane width exceeds a safe local turning radius");
        }
        subdivide(a, (a + b) / 2, depth + 1);
        subdivide((a + b) / 2, b, depth + 1);
      }
    };
  for (std::size_t i = 0; i < controls.size(); ++i) {
    subdivide(static_cast<double>(i), static_cast<double>(i + 1), 0);
  }
  const auto start = std::min_element(result.begin(), result.end(), [&](auto a, auto b) {
    return distance(a.position, route.poses.front().position) <
           distance(b.position, route.poses.front().position);
  });
  std::rotate(result.begin(), start, result.end());
  result.push_back(result.front());
  return result;
}
std::vector<RoutePose> sample_route(
  const std::vector<RoutePoint> & controls, RoutePoint start_point, double spacing_m)
{
  if (!std::isfinite(spacing_m) || spacing_m <= 0.0) {
    throw std::invalid_argument("route_spacing_m must be finite and positive");
  }
  double max_edge = 0.0;
  for (std::size_t i = 0; i < controls.size(); ++i) {
    max_edge = std::max(max_edge, distance(controls[i], controls[(i + 1) % controls.size()]));
  }
  // 用弧长表获得近似等距采样；位置和切向仍在原样条上求值，包括闭合处。
  const double subdivisions = std::max(8.0, std::ceil(4.0 * max_edge / spacing_m));
  if (!std::isfinite(subdivisions) || subdivisions * controls.size() > kMaxSamples) {
    throw std::invalid_argument("route_spacing_m is too small for this GNSS extent");
  }
  const auto steps = static_cast<std::size_t>(subdivisions) * controls.size();
  const double parameter_step = 1.0 / subdivisions;
  std::vector<double> arc(steps + 1, 0.0);
  auto previous = evaluate(controls, 0.0).position;
  for (std::size_t i = 1; i <= steps; ++i) {
    const auto point = evaluate(controls, static_cast<double>(i) * parameter_step).position;
    arc[i] = arc[i - 1] + distance(previous, point);
    previous = point;
  }
  const auto count = static_cast<std::size_t>(std::max(8.0, std::ceil(arc.back() / spacing_m)));
  std::vector<RoutePose> result;
  result.reserve(count + 1);
  std::size_t upper = 1;
  for (std::size_t i = 0; i < count; ++i) {
    const double target = arc.back() * static_cast<double>(i) / static_cast<double>(count);
    while (upper < steps && arc[upper] < target) {
      ++upper;
    }
    const double fraction = (target - arc[upper - 1]) / (arc[upper] - arc[upper - 1]);
    const double parameter = (static_cast<double>(upper - 1) + fraction) * parameter_step;
    result.push_back(evaluate(controls, parameter));
  }
  // 将起点选在首个 GNSS 记录附近，保持闭合处光滑和曲率同号。
  const auto start = std::min_element(result.begin(), result.end(), [&](auto a, auto b) {
    return distance(a.position, start_point) < distance(b.position, start_point);
  });
  std::rotate(result.begin(), start, result.end());
  result.push_back(result.front());
  return result;
}

std::vector<RoutePoint> approximate(
  const std::vector<RoutePoint> & controls, double error_m)
{
  double second_derivative = 0.0;
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const auto a = controls[i];
    const auto b = controls[(i + 1) % controls.size()];
    const auto c = controls[(i + 2) % controls.size()];
    second_derivative = std::max(second_derivative,
      std::hypot(a.x - 2 * b.x + c.x, a.y - 2 * b.y + c.y));
  }
  // 每个多项式区间满足 ||s''|| <= max ||Pi - 2Pi+1 + Pi+2||。
  // 线性插值偏差上界为 max||s''|| * h^2 / 8，用于连续曲线净距校验。
  const double steps = std::max(8.0, std::ceil(std::sqrt(second_derivative / (8 * error_m))));
  if (!std::isfinite(steps) || steps * controls.size() > kMaxSamples) {
    throw std::invalid_argument("GNSS extent exceeds route verification limit");
  }
  const auto count = static_cast<std::size_t>(steps) * controls.size();
  std::vector<RoutePoint> result;
  result.reserve(count + 1);
  for (std::size_t i = 0; i < count; ++i) {
    result.push_back(evaluate(controls, static_cast<double>(i) / steps).position);
  }
  result.push_back(result.front());
  return result;
}

struct Bounds
{
  RoutePoint min, max;
  explicit Bounds(const std::vector<RoutePoint> & points) : min(points.at(0)), max(min)
  {
    for (const auto p : points) {
      min.x = std::min(min.x, p.x);
      min.y = std::min(min.y, p.y);
      max.x = std::max(max.x, p.x);
      max.y = std::max(max.y, p.y);
    }
  }
  RoutePoint center() const {return {(min.x + max.x) / 2, (min.y + max.y) / 2};}
  RoutePoint extent() const {return {max.x - min.x, max.y - min.y};}
};

using Transform = std::array<double, 4>;  // X/Y 正缩放系数及 X/Y 平移量；平移量单位 m。

std::vector<RoutePoint> transform(
  const std::vector<RoutePoint> & points, RoutePoint center, const Transform & v)
{
  std::vector<RoutePoint> result;
  result.reserve(points.size());
  for (const auto p : points) {
    result.push_back({(p.x - center.x) * v[0] + center.x + v[2],
      (p.y - center.y) * v[1] + center.y + v[3]});
  }
  return result;
}

std::vector<RoutePoint> shape_route(
  std::vector<RoutePoint> controls, double inward_bow_m, double bottom_rounding_m)
{
  if (!std::isfinite(inward_bow_m) || inward_bow_m < 0.0 ||
    !std::isfinite(bottom_rounding_m) || bottom_rounding_m < 0.0)
  {
    throw std::invalid_argument("route shaping distances must be finite and nonnegative");
  }
  if (controls.size() < 3 || (inward_bow_m == 0.0 && bottom_rounding_m == 0.0)) {
    return controls;
  }
  const Bounds bounds(controls);
  const auto centre = bounds.center();
  const auto extent = bounds.extent();
  const auto move_towards = [&](std::size_t index, double amount) {
      if (amount == 0.0) {return;}
      const auto p = controls[index];
      const double dx = centre.x - p.x, dy = centre.y - p.y;
      const double length = std::hypot(dx, dy);
      if (length > 1e-9) {
        const double distance = std::min(amount, 0.2 * std::max(extent.x, extent.y));
        controls[index] = {p.x + dx * distance / length, p.y + dy * distance / length};
      }
    };
  // 东侧控制点形成浅内弯；南侧控制点向环心移动可增大局部转弯半径，并保持样条光滑。
  const auto east = static_cast<std::size_t>(std::max_element(
    controls.begin(), controls.end(), [](auto a, auto b) {return a.x < b.x; }) - controls.begin());
  const auto south = static_cast<std::size_t>(std::min_element(
    controls.begin(), controls.end(), [](auto a, auto b) {return a.y < b.y; }) - controls.begin());
  move_towards(east, inward_bow_m);
  move_towards(south, bottom_rounding_m);
  return controls;
}

Transform fit_water(
  const std::vector<RoutePoint> & line, const std::vector<RoutePoint> & reference,
  const RouteWater & water)
{
  const Transform identity{1, 1, 0, 0};
  if (water.unsafe_length(line) == 0.0) {
    return identity;
  }
  const Bounds shape(line), gnss(reference);
  const auto center = shape.center();
  const auto extent = shape.extent();
  // 目标函数保留 GNSS 包络及南侧转弯区域，防止安全路线过度收缩。
  const Transform desired{
    gnss.extent().x / extent.x, gnss.extent().y / extent.y,
    gnss.center().x - center.x, gnss.center().y - center.y};
  const auto target = transform(line, center, desired);
  const Transform lower{0.65, 0.65, -0.2 * extent.x, -0.2 * extent.y};
  const Transform upper{1.2, 1.2, 0.2 * extent.x, 0.2 * extent.y};
  double best_cost = std::numeric_limits<double>::infinity();
  Transform best = identity;
  const auto score = [&](const Transform & v) {
      const auto points = transform(line, center, v);
      double displacement = 0.0;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const double d = distance(points[i], target[i]);
        displacement += d * d / static_cast<double>(points.size());
      }
      const double unsafe = water.unsafe_length(points);
      if (unsafe == 0.0 && displacement < best_cost) {
        best_cost = displacement;
        best = v;
      }
      return displacement + unsafe * std::max(1000.0, std::max(extent.x, extent.y));
    };

  // 四个有界变量保持样条严格凸；固定随机种子使相同输入的差分进化搜索可复现。
  constexpr std::size_t population_size = 48;
  std::array<Transform, population_size> population;
  std::array<double, population_size> costs;
  std::mt19937 random(20261029);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_int_distribution<std::size_t> index(0, population_size - 1);
  for (std::size_t i = 0; i < population_size; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      population[i][j] = i == 0 ? identity[j] : (i == 1 ?
        std::clamp(desired[j], lower[j], upper[j]) : lower[j] + unit(random) * (upper[j] - lower[j]));
    }
    costs[i] = score(population[i]);
  }
  for (int generation = 0; generation < 180; ++generation) {
    for (std::size_t i = 0; i < population_size; ++i) {
      std::size_t a, b, c;
      do {a = index(random);} while (a == i);
      do {b = index(random);} while (b == i || b == a);
      do {c = index(random);} while (c == i || c == a || c == b);
      Transform trial = population[i];
      const auto forced_axis = index(random) % 4;
      for (std::size_t j = 0; j < 4; ++j) {
        if (j == forced_axis || unit(random) < 0.85) {
          trial[j] = std::clamp(population[a][j] +
            0.7 * (population[b][j] - population[c][j]), lower[j], upper[j]);
        }
      }
      const double cost = score(trial);
      if (cost < costs[i]) {
        population[i] = trial;
        costs[i] = cost;
      }
    }
  }
  if (!std::isfinite(best_cost)) {
    throw std::runtime_error(
      "no clockwise smooth loop found near the GNSS envelope at the requested clearance");
  }
  return best;
}
}  // namespace

std::vector<RoutePose> make_clockwise_route(
  const std::vector<RoutePoint> & reference, double spacing_m)
{
  const auto controls = spline_controls(convex_hull(reference));
  return sample_route(controls, reference.front(), spacing_m);
}

SafeRoute make_safe_clockwise_route(
  const std::vector<RoutePoint> & reference, double spacing_m,
  const usv_map::Chart & chart, double clearance_m,
  double inward_bow_m, double bottom_rounding_m)
{
  if (!std::isfinite(clearance_m) || clearance_m < 0.0) {
    throw std::invalid_argument("clearance_m must be finite and nonnegative");
  }
  auto controls = spline_controls(convex_hull(reference));
  controls = shape_route(std::move(controls), inward_bow_m, bottom_rounding_m);
  // 搜索前校验采样间距，避免无效输入进入几何计算。
  sample_route(controls, reference.front(), spacing_m);
  const RouteWater water(chart, clearance_m + 0.2);
  const auto coarse = approximate(controls, 0.02);
  const auto adjustment = fit_water(coarse, reference, water);
  controls = transform(controls, Bounds(coarse).center(), adjustment);
  auto poses = sample_route(controls, reference.front(), spacing_m);
  std::vector<RoutePoint> published;
  for (const auto & pose : poses) {
    published.push_back(pose.position);
  }
  constexpr double approximation_error = 0.002;
  const double curve_clearance = water.clearance(approximate(controls, approximation_error));
  const double path_clearance = water.clearance(published);
  const double minimum = std::min(curve_clearance - approximation_error, path_clearance);
  if (minimum < clearance_m) {
    throw std::runtime_error("continuous route or published Path failed the minimum clearance check");
  }
  return {std::move(poses), minimum, std::move(controls)};
}

Fairway make_fairway(
  const SafeRoute & route, double lane_width_m, double spacing_m,
  const usv_map::Chart & chart, double edge_clearance_m, double centerline_inset_m)
{
  if (!std::isfinite(lane_width_m) || lane_width_m <= 0 ||
    !std::isfinite(spacing_m) || spacing_m <= 0 ||
    !std::isfinite(edge_clearance_m) || edge_clearance_m < 0 ||
    !std::isfinite(centerline_inset_m) || centerline_inset_m < 0 ||
    !std::isfinite(lane_width_m + centerline_inset_m + edge_clearance_m))
  {
    throw std::invalid_argument("lane width must be positive; clearance and inset nonnegative and finite");
  }
  if (route.controls.size() < 3 || route.poses.empty() ||
    !std::isfinite(route.minimum_clearance_m) ||
    route.minimum_clearance_m < 0 ||
    (centerline_inset_m == 0 && route.minimum_clearance_m < lane_width_m + edge_clearance_m))
  {
    throw std::runtime_error("centreline has insufficient clearance for the requested fairway");
  }
  Fairway result;
  result.lane_width_m = lane_width_m;
  // 按最大偏移量约束弦误差，并排除偏移曲线的尖点。
  result.center = fairway_stations(route, lane_width_m + centerline_inset_m, spacing_m);
  const auto offset = [](RoutePose p, double d) {
      return RoutePoint{p.position.x + d * std::sin(p.yaw),
        p.position.y - d * std::cos(p.yaw)};
    };
  constexpr double pi = 3.14159265358979323846;
  std::vector<RoutePoint> center_points;
  for (auto & pose : result.center) {
    pose.position = offset(pose, centerline_inset_m);
    center_points.push_back(pose.position);
    result.inner.push_back(offset(pose, lane_width_m));
    result.outer.push_back(offset(pose, -lane_width_m));
    result.clockwise.push_back({offset(pose, lane_width_m / 2), pose.yaw});
    result.counterclockwise.push_back({offset(pose, -lane_width_m / 2),
      std::remainder(pose.yaw + pi, 2 * pi)});
  }
  std::reverse(result.counterclockwise.begin(), result.counterclockwise.end());
  const RouteWater water(chart, 0.0);
  // 内缩可能使中心线接近岛屿；移动后的中心线必须重新校验净距。
  result.centerline_clearance_m = water.clearance(center_points) - kOffsetError;
  if (result.centerline_clearance_m < lane_width_m + edge_clearance_m) {
    throw std::runtime_error("moved centreline failed the minimum shoreline/obstacle clearance check");
  }
  const double polygon_clearance = water.corridor_clearance(result.outer, result.inner);
  // 校验完整航道多边形，覆盖两侧边界之间可能存在的岛屿。
  result.minimum_clearance_m = std::min(
    result.centerline_clearance_m - lane_width_m, polygon_clearance - kOffsetError);
  if (result.minimum_clearance_m < edge_clearance_m) {
    throw std::runtime_error("fairway polygon failed the minimum edge clearance check");
  }
  // 单个 ORIENT 只能表示局部方向；每段 TSSLPT 限制为 20 m 和 10° 转角，并保留边界采样点。
  for (std::size_t begin = 0; begin + 1 < result.center.size(); ) {
    auto end = begin + 1;
    double length = distance(result.center[begin].position, result.center[end].position);
    while (end + 1 < result.center.size()) {
      const double next = length + distance(
        result.center[end].position, result.center[end + 1].position);
      if (next > 20 || std::abs(std::remainder(
          result.center[end + 1].yaw - result.center[begin].yaw, 2 * pi)) > pi / 18)
      {
        break;
      }
      length = next;
      ++end;
    }
    for (bool clockwise : {true, false}) {
      FairwaySection section;
      section.clockwise = clockwise;
      section.yaw = result.center[(begin + end) / 2].yaw + (clockwise ? 0 : pi);
      for (auto i = begin; i <= end; ++i) {
        section.polygon.push_back(result.center[i].position);
      }
      const auto & boundary = clockwise ? result.inner : result.outer;
      for (auto i = end + 1; i-- > begin; ) {
        section.polygon.push_back(boundary[i]);
      }
      section.polygon.push_back(section.polygon.front());
      result.sections.push_back(std::move(section));
    }
    begin = end;
  }
  return result;
}
}  // namespace usv_map_tools
