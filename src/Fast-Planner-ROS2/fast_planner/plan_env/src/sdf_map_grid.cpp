// 无人船二维栅格后端：静态底图与局部观测融合，保留原有查询接口。
#include <plan_env/sdf_map.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>

void SDFMap::initGridMap()
{
  // 真正的分辨率、尺寸和原点以收到的 OccupancyGrid 为准。
  mp_.resolution_ = node_->declare_parameter<double>("sdf_map/resolution", 0.1);
  mp_.frame_id_ = node_->declare_parameter<std::string>("sdf_map/frame_id", "map");
  mp_.obstacles_inflation_ = node_->declare_parameter<double>("sdf_map/obstacles_inflation", 0.0);
  std::string topic = node_->declare_parameter<std::string>("sdf_map/grid_topic", "/map");
  grid_occupied_threshold_ = node_->declare_parameter<int>("sdf_map/grid_occupied_threshold", 75);
  grid_unknown_is_occupied_ = node_->declare_parameter<bool>("sdf_map/grid_unknown_is_occupied", true);
  grid_visualization_stride_ = node_->declare_parameter<int>("sdf_map/grid_visualization_stride", 10);
  grid_publish_esdf_ = node_->declare_parameter<bool>("sdf_map/grid_publish_esdf", false);
  grid_local_range_ = node_->declare_parameter<double>("sdf_map/grid_local_range", 50.0);
  grid_distance_cap_ = node_->declare_parameter<double>("sdf_map/grid_distance_cap", 10.0);
  grid_snapshot_period_ = node_->declare_parameter<double>("sdf_map/grid_snapshot_period", 5.0);
  std::string local_topic = node_->declare_parameter<std::string>("sdf_map/grid_local_topic", "/local_map");

  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
      !std::isfinite(mp_.obstacles_inflation_) || mp_.obstacles_inflation_ < 0.0 ||
      grid_occupied_threshold_ < 1 || grid_occupied_threshold_ > 100 ||
      grid_visualization_stride_ < 1 || mp_.frame_id_.empty() || topic.empty() || local_topic.empty() ||
      !std::isfinite(grid_local_range_) || grid_local_range_ <= 0.0 ||
      !std::isfinite(grid_distance_cap_) || grid_distance_cap_ <= 0.0 ||
      !std::isfinite(grid_snapshot_period_) || grid_snapshot_period_ <= 0.0)
  {
    throw std::invalid_argument("Invalid sdf_map static grid parameters.");
  }

  // 仅用于地图到来前的接口初始化，不在这里分配大范围三维缓存。
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.map_origin_.setZero();
  mp_.map_size_.setZero();
  mp_.map_voxel_num_.setZero();
  mp_.map_min_boundary_.setZero();
  mp_.map_max_boundary_.setZero();
  mp_.map_min_idx_.setZero();
  mp_.map_max_idx_.setZero();
  md_.esdf_need_update_ = false;
  grid_map_ready_ = false;
  grid_last_window_index_ << -1, -1, 0;

  // 静态地图可能只发布一次，晚启动也要能收到已发布的地图。
  rclcpp::QoS map_qos = rclcpp::QoS(1).reliable().transient_local();
  grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, map_qos, std::bind(&SDFMap::gridMapCallback, this, std::placeholders::_1));
  grid_inflate_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/sdf_map/grid_inflate", map_qos);
  grid_local_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_topic, rclcpp::SensorDataQoS(),
      std::bind(&SDFMap::localGridCallback, this, std::placeholders::_1));
  grid_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/sdf_map/odom", rclcpp::SensorDataQoS(),
      std::bind(&SDFMap::gridOdometryCallback, this, std::placeholders::_1));
  grid_fused_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/sdf_map/grid_fused", map_qos);
  grid_fused_update_pub_ = node_->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      "/sdf_map/grid_fused_updates", 10);
  grid_inflate_update_pub_ = node_->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      "/sdf_map/grid_inflate_updates", 10);
  grid_last_snapshot_time_ = node_->get_clock()->now();
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 1);

  esdf_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SDFMap::updateESDFCallback, this));
  RCLCPP_INFO(node_->get_logger(), "USV grid input: %s, planning frame: %s",
              topic.c_str(), mp_.frame_id_.c_str());
}

void SDFMap::gridMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
  // 本次统一 map 坐标系，不把 frame_id 改名当作坐标变换。
  if (msg->header.frame_id != mp_.frame_id_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Grid frame '%s' differs from planning frame '%s'; map rejected.",
                 msg->header.frame_id.c_str(), mp_.frame_id_.c_str());
    return;
  }
  const auto& info = msg->info;
  size_t cells = static_cast<size_t>(info.width) * info.height;
  Eigen::Quaterniond q(info.origin.orientation.w, info.origin.orientation.x,
                       info.origin.orientation.y, info.origin.orientation.z);
  if (info.width == 0 || info.height == 0 ||
      cells > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      info.width > static_cast<unsigned int>(std::numeric_limits<int>::max() - 2) ||
      info.height > static_cast<unsigned int>(std::numeric_limits<int>::max() - 2) ||
      msg->data.size() != cells || !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
      !q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-3)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid OccupancyGrid dimensions, origin, resolution or data.");
    return;
  }
  q.normalize();
  Eigen::Matrix3d rotation = q.toRotationMatrix();
  if (std::abs(rotation(2, 2) - 1.0) > 1e-6)
  {
    RCLCPP_ERROR(node_->get_logger(), "USV grid origin must be planar (yaw only).");
    return;
  }
  for (size_t i = 0; i < cells; ++i)
  {
    if (msg->data[i] < -1 || msg->data[i] > 100)
    {
      RCLCPP_ERROR(node_->get_logger(), "OccupancyGrid values must be -1 or 0..100.");
      return;
    }
  }

  bool same_geometry = grid_map_ && info.width == grid_map_->info.width &&
      info.height == grid_map_->info.height && info.resolution == grid_map_->info.resolution &&
      info.origin.position.x == grid_map_->info.origin.position.x &&
      info.origin.position.y == grid_map_->info.origin.position.y &&
      std::abs(rotation(0, 0) - grid_cos_yaw_) < 1e-9 &&
      std::abs(rotation(1, 0) - grid_sin_yaw_) < 1e-9;
  if (same_geometry && msg->data == grid_map_->data)
  {
    return;  // 静态底图内容没变，不因重复发布重新计算全图。
  }
  grid_map_ready_ = false;
  grid_map_ = msg;  // 保存共享消息，不再复制一份大地图。
  grid_cos_yaw_ = rotation(0, 0);
  grid_sin_yaw_ = rotation(1, 0);
  mp_.resolution_ = info.resolution;
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.map_origin_ << info.origin.position.x, info.origin.position.y, 0.0;
  mp_.map_size_ << info.width * mp_.resolution_, info.height * mp_.resolution_, 0.0;
  mp_.map_voxel_num_ << info.width, info.height, 1;
  mp_.map_min_idx_.setZero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  // 旋转栅格的轴对齐包围盒只用于范围接口，索引仍使用真实原点及旋转。
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_;
  for (int x = 0; x <= 1; ++x)
  {
    for (int y = 0; y <= 1; ++y)
    {
      Eigen::Vector3d corner;
      double dx = x * mp_.map_size_(0), dy = y * mp_.map_size_(1);
      corner << mp_.map_origin_(0) + grid_cos_yaw_ * dx - grid_sin_yaw_ * dy,
                mp_.map_origin_(1) + grid_sin_yaw_ * dx + grid_cos_yaw_ * dy, 0.0;
      mp_.map_min_boundary_ = mp_.map_min_boundary_.cwiseMin(corner);
      mp_.map_max_boundary_ = mp_.map_max_boundary_.cwiseMax(corner);
    }
  }
  if (!same_geometry)
  {
    grid_local_values_ = cv::Mat(info.height, info.width, CV_8SC1, cv::Scalar(-1));
  }
  grid_fused_.create(info.height, info.width, CV_8SC1);
  for (int y = 0; y < grid_fused_.rows; ++y)
  {
    for (int x = 0; x < grid_fused_.cols; ++x)
    {
      grid_fused_.at<signed char>(y, x) = fusedGridValue(x, y);
    }
  }
  grid_inflate_.release();
  grid_distance_.release();
  grid_full_update_ = true;
  // 与 cloudCallback 相同：先接收占据数据，再由距离场定时器完成更新。
  md_.esdf_need_update_ = true;
}

void SDFMap::updateESDF2d()
{
  if (!grid_map_)
  {
    return;
  }
  const int width = mp_.map_voxel_num_(0);
  const int height = mp_.map_voxel_num_(1);
  bool full = grid_full_update_;
  cv::Rect core = full ? cv::Rect(0, 0, width, height) : gridWindow();
  if (!full && core.empty())
  {
    return;
  }

  bool changed = false;
  if (grid_has_odom_)
  {
    cv::Rect observed_region = gridWindow();
    while (!grid_local_queue_.empty())
    {
      changed = fuseLocalGrid(*grid_local_queue_.front(), observed_region) || changed;
      grid_local_queue_.pop_front();
    }
  }
  if (!full && !changed && !grid_window_dirty_)
  {
    md_.esdf_need_update_ = false;
    return;
  }

  // 饱和距离10m + 膨胀3m + 两格余量，保证窗口内近场计算能读取窗口外邻近障碍。
  double requested_halo = std::ceil((grid_distance_cap_ + mp_.obstacles_inflation_) /
                                    mp_.resolution_) + 2.0;
  int halo = static_cast<int>(std::min(double(std::max(width, height)), requested_halo));
  int left = std::max(0, core.x - halo), bottom = std::max(0, core.y - halo);
  int right = core.x + core.width + std::min(halo, width - core.x - core.width);
  int top = core.y + core.height + std::min(halo, height - core.y - core.height);
  cv::Rect work(left, bottom, right - left, top - bottom);
  cv::Mat free_mask(work.height, work.width, CV_8UC1);
  for (int y = 0; y < work.height; ++y)
  {
    for (int x = 0; x < work.width; ++x)
    {
      int value = grid_fused_.at<signed char>(work.y + y, work.x + x);
      bool occupied = value >= grid_occupied_threshold_ || (value < 0 && grid_unknown_is_occupied_);
      free_mask.at<unsigned char>(y, x) = occupied ? 0 : 255;
    }
  }

  cv::Mat padded, distance;
  // 局部计算框边缘不是障碍墙；只有真实全局地图外边界补障碍。
  cv::copyMakeBorder(free_mask, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(255));
  if (work.x == 0)
  {
    padded.col(0).setTo(0);
  }
  if (work.y == 0)
  {
    padded.row(0).setTo(0);
  }
  if (right == width)
  {
    padded.col(padded.cols - 1).setTo(0);
  }
  if (top == height)
  {
    padded.row(padded.rows - 1).setTo(0);
  }
  cv::distanceTransform(padded, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::Mat inflated(work.height, work.width, CV_8UC1);
  for (int y = 0; y < work.height; ++y)
  {
    for (int x = 0; x < work.width; ++x)
    {
      double clearance = (distance.at<float>(y + 1, x + 1) - 0.5) * mp_.resolution_;
      bool occupied = free_mask.at<unsigned char>(y, x) == 0 || clearance <= mp_.obstacles_inflation_;
      inflated.at<unsigned char>(y, x) = occupied ? 1 : 0;
      free_mask.at<unsigned char>(y, x) = occupied ? 0 : 255;
    }
  }
  cv::copyMakeBorder(free_mask, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(255));
  if (work.x == 0)
  {
    padded.col(0).setTo(0);
  }
  if (work.y == 0)
  {
    padded.row(0).setTo(0);
  }
  if (right == width)
  {
    padded.col(padded.cols - 1).setTo(0);
  }
  if (top == height)
  {
    padded.row(padded.rows - 1).setTo(0);
  }
  cv::distanceTransform(padded, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  cv::Mat signed_distance(work.height, work.width, CV_32FC1);
  for (int y = 0; y < work.height; ++y)
  {
    for (int x = 0; x < work.width; ++x)
    {
      signed_distance.at<float>(y, x) = std::min(grid_distance_cap_,
          std::max(0.0, (distance.at<float>(y + 1, x + 1) - 0.5) * mp_.resolution_));
    }
  }
  cv::bitwise_not(padded, free_mask);
  cv::distanceTransform(free_mask, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  if (full)
  {
    grid_inflate_.create(height, width, CV_8UC1);
    grid_distance_.create(height, width, CV_32FC1);
  }
  for (int y = core.y; y < core.y + core.height; ++y)
  {
    for (int x = core.x; x < core.x + core.width; ++x)
    {
      if (!full && !insideGridWindow(x, y))
      {
        continue;  // 旋转地图的包围框可大于100m方形，仅写回真正窗口内的格子。
      }
      int wx = x - work.x, wy = y - work.y;
      bool occupied = inflated.at<unsigned char>(wy, wx) != 0;
      double value = signed_distance.at<float>(wy, wx);
      if (occupied)
      {
        value = -std::min(grid_distance_cap_,
            std::max(0.0, (distance.at<float>(wy + 1, wx + 1) - 0.5) * mp_.resolution_));
      }
      grid_inflate_.at<unsigned char>(y, x) = occupied ? 1 : 0;
      grid_distance_.at<float>(y, x) = value;
    }
  }
  md_.esdf_need_update_ = false;
  grid_full_update_ = false;
  grid_window_dirty_ = false;
  grid_map_ready_ = true;
  publishGridMap(full, core);
  if (full)
  {
    RCLCPP_INFO(node_->get_logger(), "USV global grid initialized: %d x %d, %.3f m/cell",
                width, height, mp_.resolution_);
  }
}

Eigen::Vector2d SDFMap::gridCoordinates(const Eigen::Vector3d& pos) const
{
  double dx = pos(0) - mp_.map_origin_(0);
  double dy = pos(1) - mp_.map_origin_(1);
  return Eigen::Vector2d(grid_cos_yaw_ * dx + grid_sin_yaw_ * dy,
                        -grid_sin_yaw_ * dx + grid_cos_yaw_ * dy);
}

double SDFMap::gridDistanceWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) const
{
  grad.setZero();
  if (!grid_map_ready_ || !std::isfinite(pos(0)) || !std::isfinite(pos(1)))
  {
    return 0.0;
  }
  Eigen::Vector2d local = gridCoordinates(pos);
  double width = mp_.map_size_(0), height = mp_.map_size_(1);
  if (local(0) < 0.0 || local(1) < 0.0 || local(0) >= width || local(1) >= height)
  {
    // 地图外返回非正距离和指向地图内部的梯度，不夹到边界假装已知空闲。
    double dx = std::max(0.0, std::min(width, local(0))) - local(0);
    double dy = std::max(0.0, std::min(height, local(1))) - local(1);
    double length = std::hypot(dx, dy);
    if (length > 0.0)
    {
      grad << (grid_cos_yaw_ * dx - grid_sin_yaw_ * dy) / length,
              (grid_sin_yaw_ * dx + grid_cos_yaw_ * dy) / length, 0.0;
    }
    else
    {
      double inward_x = local(0) >= width ? -1.0 : 0.0;
      double inward_y = local(1) >= height ? -1.0 : 0.0;
      double norm = std::hypot(inward_x, inward_y);
      grad << (grid_cos_yaw_ * inward_x - grid_sin_yaw_ * inward_y) / norm,
              (grid_sin_yaw_ * inward_x + grid_cos_yaw_ * inward_y) / norm, 0.0;
    }
    double value = length + mp_.obstacles_inflation_;
    if (value >= grid_distance_cap_)
    {
      grad.setZero();
    }
    return -std::min(value, grid_distance_cap_);
  }

  // 双线性插值，梯度是对同一插值距离求导；第三项始终为 0。
  double gx = local(0) * mp_.resolution_inv_ - 0.5;
  double gy = local(1) * mp_.resolution_inv_ - 0.5;
  double cx = std::max(0.0, std::min(double(grid_distance_.cols - 1), gx));
  double cy = std::max(0.0, std::min(double(grid_distance_.rows - 1), gy));
  int x0 = static_cast<int>(std::floor(cx));
  int y0 = static_cast<int>(std::floor(cy));
  int x1 = std::min(x0 + 1, grid_distance_.cols - 1);
  int y1 = std::min(y0 + 1, grid_distance_.rows - 1);
  double tx = cx - x0, ty = cy - y0;
  double d00 = grid_distance_.at<float>(y0, x0);
  double d10 = grid_distance_.at<float>(y0, x1);
  double d01 = grid_distance_.at<float>(y1, x0);
  double d11 = grid_distance_.at<float>(y1, x1);
  double value = (1.0 - ty) * ((1.0 - tx) * d00 + tx * d10) +
                ty * ((1.0 - tx) * d01 + tx * d11);
  double grad_x = ((1.0 - ty) * (d10 - d00) + ty * (d11 - d01)) * mp_.resolution_inv_;
  double grad_y = ((1.0 - tx) * (d01 - d00) + tx * (d11 - d10)) * mp_.resolution_inv_;
  if (cx != gx)
  {
    grad_x = 0.0;
  }
  if (cy != gy)
  {
    grad_y = 0.0;
  }

  // 栅格最外半格的插值需要补充到物理地图边界的距离。
  double edge = local(0);
  double edge_x = 1.0, edge_y = 0.0;
  if (width - local(0) < edge)
  {
    edge = width - local(0); edge_x = -1.0; edge_y = 0.0;
  }
  if (local(1) < edge)
  {
    edge = local(1); edge_x = 0.0; edge_y = 1.0;
  }
  if (height - local(1) < edge)
  {
    edge = height - local(1); edge_x = 0.0; edge_y = -1.0;
  }
  edge -= mp_.obstacles_inflation_;
  if (edge < value)
  {
    value = edge; grad_x = edge_x; grad_y = edge_y;
  }
  grad << grid_cos_yaw_ * grad_x - grid_sin_yaw_ * grad_y,
          grid_sin_yaw_ * grad_x + grid_cos_yaw_ * grad_y, 0.0;
  return value;
}

