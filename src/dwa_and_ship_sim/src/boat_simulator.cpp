#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class BoatSimulator : public rclcpp::Node
{
public:
  // [功能与联系] 建立本船初始状态、简化惯性模型与控制订阅，启动20Hz运动和数据发布；只能运行一个base_link发布者。
  BoatSimulator()
  : Node("boat_simulator")
  {
    // ==================== 船初始状态 ====================
    const double legacy_start_x = declare_parameter<double>("star_x", 0.0);
    const double legacy_start_y = declare_parameter<double>("star_y", 0.0);
    x_ = declare_parameter<double>("start_x", legacy_start_x);
    y_ = declare_parameter<double>("start_y", legacy_start_y);
    yaw_ = declare_parameter<double>("start_yaw", 0.0);

    // 仿真速度上限必须与DWA max_vel_x一致，否则控制器发布高速命令后会在
    // 仿真器中被静默截断，Hybrid A*的时间同步动态避碰也会使用错误航速。
    max_linear_speed_ = std::max(
      0.0, declare_parameter<double>("max_linear_speed", 1.2));

    // 当前实际线速度、角速度；初速同样受显式仿真上限约束。
    v_ = std::clamp(
      declare_parameter<double>("initial_speed", 0.0),
      -max_linear_speed_, max_linear_speed_);
    w_ = 0.0;

    // 接收到的目标控制命令
    cmd_v_ = v_;
    cmd_w_ = 0.0;

    // ==================== 圆形障碍物 ====================
    // 格式：{圆心 x, 圆心 y, 半径}
    circle_obstacles_.push_back({5.0, 0.0, 1.0});
    circle_obstacles_.push_back({10.0, 2.0, 0.8});
    circle_obstacles_.push_back({7.0, -3.0, 0.6});
    circle_obstacles_.push_back({3.0, 4.0, 0.5});
    circle_obstacles_.push_back({-2.0, -2.0, 0.7});

    // ==================== 正方形障碍物 ====================
    /*
     * 正方形顶点必须按顺时针或逆时针顺序填写。
     *
     * 此正方形边长为 2.0 m，中心大约在 (8.0, -1.5)。
     *
     *     (7.0,-0.5) -------- (9.0,-0.5)
     *          |                  |
     *          |       正方形      |
     *          |                  |
     *     (7.0,-2.5) -------- (9.0,-2.5)
     */
    PolygonObstacle square;
    square.name = "square_obstacle";
    square.r = 1.0;
    square.g = 0.7;
    square.b = 0.0;

    square.vertices.push_back({7.0, -6.5});
    square.vertices.push_back({9.0, -6.5});
    square.vertices.push_back({9.0, -4.5});
    square.vertices.push_back({7.0, -4.5});

    polygon_obstacles_.push_back(square);

    // ==================== 三角形障碍物 ====================
    /*
     * 三角形顶点按顺时针或逆时针填写。
     *
     *             (12.0, 5.2)
     *                 /\
     *                /  \
     *               / 三角 \
     *              /  形   \
     *   (10.8,3.0) -------- (13.2,3.0)
     */
    PolygonObstacle triangle;
    triangle.name = "triangle_obstacle";
    triangle.r = 0.8;
    triangle.g = 0.1;
    triangle.b = 0.9;

    triangle.vertices.push_back({10.8, 3.0});
    triangle.vertices.push_back({13.2, 3.0});
    triangle.vertices.push_back({12.0, 5.2});

    polygon_obstacles_.push_back(triangle);
    if(!declare_parameter<bool>("static_obstacles_enabled",true)) {
      circle_obstacles_.clear();polygon_obstacles_.clear();
    }

    // ==================== ROS 订阅 ====================
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::QoS(10),
      std::bind(
        &BoatSimulator::cmdCallback,
        this,
        std::placeholders::_1));

    // ==================== ROS 发布 ====================
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(20));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::QoS(10));

    obstacle_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/sim_obstacles",
      rclcpp::QoS(1).transient_local());

    boat_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
      "/sim_boat_marker",
      rclcpp::QoS(10));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 控制频率 20 Hz
    update_period_ = 0.05;

    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_period_)),
      std::bind(&BoatSimulator::update, this));

    // 障碍物本身不移动，定时发布即可
    marker_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BoatSimulator::publishObstacleMarkers, this));

    publishObstacleMarkers();

    RCLCPP_INFO(
      get_logger(),
      "Boat simulator started. "
      "Circles=%zu, polygons=%zu",
      circle_obstacles_.size(),
      polygon_obstacles_.size());
  }

private:
  static constexpr double kPi =
    3.14159265358979323846;

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  struct CircleObstacle
  {
    double x{0.0};
    double y{0.0};
    double radius{1.0};
  };

  struct PolygonObstacle
  {
    std::string name;
    std::vector<Point2D> vertices;

    // RViz 显示颜色
    double r{1.0};
    double g{0.0};
    double b{0.0};
  };

  // 接收 DWA 或其他控制器发布的速度命令
  // [功能与联系] 接收本船Twist并按速度上限缓存命令；updateBoatMotion经加速度约束逐步接近目标速率。
  void cmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_v_ = std::clamp(
      msg->linear.x,
      -max_linear_speed_,
      max_linear_speed_);

    cmd_w_ = std::clamp(
      msg->angular.z,
      -max_angular_speed_,
      max_angular_speed_);
  }

  // 仿真主循环：运动、里程计、TF、激光、船 Marker
  // [功能与联系] 本船仿真定时入口，依次调用updateBoatMotion、publishOdomAndTf、publishLaserScan与publishBoatMarker，统一推进状态并发布观测。
  void update()
  {
    updateBoatMotion();
    publishOdomAndTf();
    publishLaserScan();
    publishBoatMarker();
  }

  // 简单一阶速度响应模型
  // [功能与联系] 按固定加速度限制逼近目标v/w，再积分位置与航向；只是简化惯性，不是水动力学模型。
  void updateBoatMotion()
  {
    /*
     * 用加速度限制模拟船体惯性。
     * 不会瞬间达到 cmd_vel 给定的目标速度。
     */
    v_ = approach(
      v_,
      cmd_v_,
      linear_acc_limit_ * update_period_);

    w_ = approach(
      w_,
      cmd_w_,
      angular_acc_limit_ * update_period_);

    // 差速/单体船简化运动学模型
    x_ += v_ * std::cos(yaw_) * update_period_;
    y_ += v_ * std::sin(yaw_) * update_period_;
    yaw_ = normalizeAngle(yaw_ + w_ * update_period_);
  }

  // 发布 /odom 和 odom -> base_link TF
  // [功能与联系] 发布本船/odom及odom→base_link，反馈给航道分类、两级规划器并供RViz定位。
  void publishOdomAndTf()
  {
    const rclcpp::Time stamp = now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    geometry_msgs::msg::Quaternion orientation =
      tf2::toMsg(q);

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = orientation;

    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = w_;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  // 发布激光雷达 /scan
  // [功能与联系] 对内置几何障碍进行射线交点模拟/scan；不会自动扫描river_chart海图或形成障碍costmap。
  void publishLaserScan()
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = now();
    scan.header.frame_id = "base_link";

    scan.angle_min = -kPi;
    scan.angle_max = kPi;
    scan.angle_increment =
      2.0 * kPi /
      static_cast<double>(laser_beam_count_ - 1);

    scan.time_increment = 0.0;
    scan.scan_time = update_period_;

    scan.range_min = laser_range_min_;
    scan.range_max = laser_range_max_;

    scan.ranges.resize(
      static_cast<std::size_t>(laser_beam_count_),
      laser_range_max_);

    scan.intensities.resize(
      static_cast<std::size_t>(laser_beam_count_),
      0.0F);

    /*
     * 对每一根激光束：
     * 1. 与所有圆形障碍物求交；
     * 2. 与所有正方形、三角形等多边形边缘求交；
     * 3. 保存最近的相交距离。
     */
    for (int i = 0; i < laser_beam_count_; ++i) {
      const double laser_angle =
        scan.angle_min +
        static_cast<double>(i) *
        scan.angle_increment;

      // 激光在 odom 坐标系中的方向
      const double world_angle = yaw_ + laser_angle;

      const Point2D ray_origin{x_, y_};

      const Point2D ray_direction{
        std::cos(world_angle),
        std::sin(world_angle)};

      double nearest_range = laser_range_max_;

      // 检查圆形障碍物
      for (const CircleObstacle & obstacle :
        circle_obstacles_)
      {
        const double range = rayCircleIntersection(
          ray_origin,
          ray_direction,
          obstacle);

        if (range >= laser_range_min_ &&
          range < nearest_range)
        {
          nearest_range = range;
        }
      }

      // 检查正方形、三角形等多边形障碍物
      for (const PolygonObstacle & polygon :
        polygon_obstacles_)
      {
        const double range = rayPolygonIntersection(
          ray_origin,
          ray_direction,
          polygon);

        if (range >= laser_range_min_ &&
          range < nearest_range)
        {
          nearest_range = range;
        }
      }

      /*
       * ROS LaserScan 规范：
       * 没有命中时可以设为 inf。
       * 这里为了与许多简单局部栅格节点兼容，
       * 使用 range_max + 0.01 作为无效量测。
       */
      if (nearest_range >= laser_range_max_) {
        scan.ranges[static_cast<std::size_t>(i)] =
          std::numeric_limits<float>::infinity();
      } else {
        scan.ranges[static_cast<std::size_t>(i)] =
          static_cast<float>(nearest_range);

        scan.intensities[static_cast<std::size_t>(i)] =
          1.0F;
      }
    }

    scan_pub_->publish(scan);
  }

  /*
   * 激光射线与圆形障碍物的求交。
   *
   * 射线：
   *   P(t) = origin + t * direction, t >= 0
   *
   * 返回：
   *   最近交点距离；
   *   若没有交点，返回 laser_range_max_。
   */
  // [功能与联系] 求射线与内置圆障碍最近正交点距离，供模拟LaserScan使用。
  double rayCircleIntersection(
    const Point2D & origin,
    const Point2D & direction,
    const CircleObstacle & circle) const
  {
    const double dx = origin.x - circle.x;
    const double dy = origin.y - circle.y;

    const double b =
      2.0 * (dx * direction.x + dy * direction.y);

    const double c =
      dx * dx +
      dy * dy -
      circle.radius * circle.radius;

    const double discriminant = b * b - 4.0 * c;

    if (discriminant < 0.0) {
      return laser_range_max_;
    }

    const double sqrt_discriminant =
      std::sqrt(discriminant);

    const double t1 =
      (-b - sqrt_discriminant) * 0.5;

    const double t2 =
      (-b + sqrt_discriminant) * 0.5;

    double result = laser_range_max_;

    if (t1 >= laser_range_min_) {
      result = t1;
    }

    if (t2 >= laser_range_min_) {
      result = std::min(result, t2);
    }

    return result;
  }

  /*
   * 激光射线与多边形求交。
   *
   * 多边形由若干线段构成：
   * vertex[i] -> vertex[i+1]
   * 最后一条边：
   * vertex[last] -> vertex[0]
   */
  // [功能与联系] 遍历多边形边求射线最近交点，供模拟LaserScan使用。
  double rayPolygonIntersection(
    const Point2D & ray_origin,
    const Point2D & ray_direction,
    const PolygonObstacle & polygon) const
  {
    if (polygon.vertices.size() < 3) {
      return laser_range_max_;
    }

    double nearest_range = laser_range_max_;

    for (std::size_t i = 0;
      i < polygon.vertices.size();
      ++i)
    {
      const Point2D & edge_start =
        polygon.vertices[i];

      const Point2D & edge_end =
        polygon.vertices[
        (i + 1) % polygon.vertices.size()];

      const double range = raySegmentIntersection(
        ray_origin,
        ray_direction,
        edge_start,
        edge_end);

      if (range >= laser_range_min_ &&
        range < nearest_range)
      {
        nearest_range = range;
      }
    }

    return nearest_range;
  }

  /*
   * 射线和线段求交。
   *
   * 射线：
   *   R(t) = origin + t * direction
   *   t >= 0
   *
   * 线段：
   *   S(u) = segment_start + u * (segment_end - segment_start)
   *   0 <= u <= 1
   *
   * 返回射线方向上的距离 t。
   */
  // [功能与联系] 用二维叉积求射线/线段交点及有效范围，供多边形扫描使用。
  double raySegmentIntersection(
    const Point2D & ray_origin,
    const Point2D & ray_direction,
    const Point2D & segment_start,
    const Point2D & segment_end) const
  {
    const Point2D segment_direction{
      segment_end.x - segment_start.x,
      segment_end.y - segment_start.y};

    const double denominator =
      cross(ray_direction, segment_direction);

    // 平行或几乎平行，无交点
    if (std::abs(denominator) < 1.0e-10) {
      return laser_range_max_;
    }

    const Point2D difference{
      segment_start.x - ray_origin.x,
      segment_start.y - ray_origin.y};

    const double t =
      cross(difference, segment_direction) /
      denominator;

    const double u =
      cross(difference, ray_direction) /
      denominator;

    // t >= 0 表示在射线前方；u 在 [0,1] 表示在线段内
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
      return t;
    }

    return laser_range_max_;
  }

  // 发布圆、正方形、三角形等障碍物 Marker
  // [功能与联系] 显示内置静态障碍几何；河道模式关闭内置障碍，Marker不会自动形成代价地图。
  void publishObstacleMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action =
      visualization_msgs::msg::Marker::DELETEALL;

    markers.markers.push_back(delete_all);

    int marker_id = 0;

    // 发布圆形障碍物
    for (const CircleObstacle & obstacle :
      circle_obstacles_)
    {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "odom";
      marker.header.stamp = now();

      marker.ns = "circle_obstacles";
      marker.id = marker_id++;

      marker.type =
        visualization_msgs::msg::Marker::CYLINDER;

      marker.action =
        visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = 0.35;

      marker.pose.orientation.w = 1.0;

      marker.scale.x = 2.0 * obstacle.radius;
      marker.scale.y = 2.0 * obstacle.radius;
      marker.scale.z = 0.7;

      marker.color.a = 0.90F;
      marker.color.r = 0.15F;
      marker.color.g = 0.15F;
      marker.color.b = 0.15F;

      markers.markers.push_back(marker);
    }

    // 发布正方形、三角形等多边形障碍物
    for (const PolygonObstacle & polygon :
      polygon_obstacles_)
    {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "odom";
      marker.header.stamp = now();

      marker.ns = polygon.name;
      marker.id = marker_id++;

      /*
       * LINE_STRIP 显示多边形边框。
       * 最后重复加入第一个顶点，使图形闭合。
       */
      marker.type =
        visualization_msgs::msg::Marker::LINE_STRIP;

      marker.action =
        visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.12;

      marker.color.a = 1.0F;
      marker.color.r = static_cast<float>(polygon.r);
      marker.color.g = static_cast<float>(polygon.g);
      marker.color.b = static_cast<float>(polygon.b);

      marker.pose.orientation.w = 1.0;

      marker.lifetime = rclcpp::Duration::from_seconds(0.0);

      for (const Point2D & vertex :
        polygon.vertices)
      {
        geometry_msgs::msg::Point point;

        point.x = vertex.x;
        point.y = vertex.y;
        point.z = 0.10;

        marker.points.push_back(point);
      }

      // 追加起点，闭合正方形/三角形
      if (!polygon.vertices.empty()) {
        geometry_msgs::msg::Point point;

        point.x = polygon.vertices.front().x;
        point.y = polygon.vertices.front().y;
        point.z = 0.10;

        marker.points.push_back(point);
      }

      markers.markers.push_back(marker);

      /*
       * 额外发布一个半透明填充面。
       * 仅用于 RViz 显示，不参与激光计算。
       */
      visualization_msgs::msg::Marker fill_marker;

      fill_marker.header.frame_id = "odom";
      fill_marker.header.stamp = now();

      fill_marker.ns = polygon.name + "_fill";
      fill_marker.id = marker_id++;

      fill_marker.type =
        visualization_msgs::msg::Marker::TRIANGLE_LIST;

      fill_marker.action =
        visualization_msgs::msg::Marker::ADD;

      fill_marker.pose.orientation.w = 1.0;

      fill_marker.color.a = 0.20F;
      fill_marker.color.r = static_cast<float>(polygon.r);
      fill_marker.color.g = static_cast<float>(polygon.g);
      fill_marker.color.b = static_cast<float>(polygon.b);

      /*
       * 使用扇形三角剖分填充凸多边形。
       * 当前正方形与三角形都是凸多边形，适用。
       */
      if (polygon.vertices.size() >= 3) {
        for (std::size_t i = 1;
          i + 1 < polygon.vertices.size();
          ++i)
        {
          geometry_msgs::msg::Point p0;
          geometry_msgs::msg::Point p1;
          geometry_msgs::msg::Point p2;

          p0.x = polygon.vertices[0].x;
          p0.y = polygon.vertices[0].y;
          p0.z = 0.02;

          p1.x = polygon.vertices[i].x;
          p1.y = polygon.vertices[i].y;
          p1.z = 0.02;

          p2.x = polygon.vertices[i + 1].x;
          p2.y = polygon.vertices[i + 1].y;
          p2.z = 0.02;

          fill_marker.points.push_back(p0);
          fill_marker.points.push_back(p1);
          fill_marker.points.push_back(p2);
        }
      }

      markers.markers.push_back(fill_marker);
    }

    obstacle_marker_pub_->publish(markers);
  }

  // RViz 显示无人船
  // [功能与联系] 在本船姿态处显示模型；与TF显示重复开启时可能看似有两艘船。
  void publishBoatMarker()
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp = now();

    marker.ns = "sim_boat";
    marker.id = 0;

    marker.type =
      visualization_msgs::msg::Marker::ARROW;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 0.15;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = 1.0;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    marker.color.a = 1.0F;
    marker.color.r = 0.0F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;

    boat_marker_pub_->publish(marker);
  }

  // [功能与联系] 计算二维叉积，用于射线与边界交点求解。
  static double cross(
    const Point2D & a,
    const Point2D & b)
  {
    return a.x * b.y - a.y * b.x;
  }

  // [功能与联系] 以最大步长将当前速率逼近指令值，供简化惯性更新。
  static double approach(
    const double current,
    const double target,
    const double maximum_change)
  {
    if (current < target) {
      return std::min(
        current + maximum_change,
        target);
    }

    return std::max(
      current - maximum_change,
      target);
  }

  // [功能与联系] 归一化角度，消除跨±pi的跳变；方向比较、运动积分和COLREG约束共同使用。
  static double normalizeAngle(double angle)
  {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }

    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }

    return angle;
  }

  // ==================== 船状态 ====================
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  double v_{0.0};
  double w_{0.0};

  double cmd_v_{0.0};
  double cmd_w_{0.0};

  // ==================== 船体运动约束 ====================
  double max_linear_speed_{1.2};
  double max_angular_speed_{1.5};

  double linear_acc_limit_{0.8};
  double angular_acc_limit_{2.0};

  double update_period_{0.05};

  // ==================== 激光雷达参数 ====================
  int laser_beam_count_{360};

  double laser_range_min_{0.10};
  double laser_range_max_{15.0};

  // ==================== 障碍物 ====================
  std::vector<CircleObstacle> circle_obstacles_;
  std::vector<PolygonObstacle> polygon_obstacles_;

  // ==================== ROS 通信对象 ====================
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_pub_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    obstacle_marker_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    boat_marker_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr
    update_timer_;

  rclcpp::TimerBase::SharedPtr
    marker_timer_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<BoatSimulator>());

  rclcpp::shutdown();

  return 0;
}
