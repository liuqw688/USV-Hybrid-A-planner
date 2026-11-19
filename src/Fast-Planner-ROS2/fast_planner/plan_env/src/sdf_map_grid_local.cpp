// 无人船局部栅格融合：观测写入全局缓存，距离场只刷新随船移动的窗口。
#include <plan_env/sdf_map.h>
#include <algorithm>
#include <limits>

namespace {
// 将旋转的传感器格子裁剪到一个全局格子，返回覆盖面积（全局格面积为1）。
// 障碍采用任意正面积覆盖；空闲必须覆盖整格，避免细障碍或未知区被中心采样漏掉。
double cellOverlapArea(const Eigen::Vector2d corners[4], int x, int y)
{
  Eigen::Vector2d polygon[12], clipped[12];
  int count = 4;
  for (int i = 0; i < 4; ++i)
  {
    polygon[i] = corners[i];
  }
  for (int edge = 0; edge < 4; ++edge)
  {
    int axis = edge < 2 ? 0 : 1;
    bool lower = edge == 0 || edge == 2;
    double bound = axis == 0 ? x : y;
    if (!lower)
    {
      bound += 1.0;
    }
    int next_count = 0;
    for (int i = 0; i < count; ++i)
    {
      Eigen::Vector2d a = polygon[(i + count - 1) % count];
      Eigen::Vector2d b = polygon[i];
      bool inside_a = lower ? a(axis) >= bound : a(axis) <= bound;
      bool inside_b = lower ? b(axis) >= bound : b(axis) <= bound;
      if (inside_a != inside_b)
      {
        double ratio = (bound - a(axis)) / (b(axis) - a(axis));
        clipped[next_count++] = a + ratio * (b - a);
      }
      if (inside_b)
      {
        clipped[next_count++] = b;
      }
    }
    count = next_count;
    if (count == 0)
    {
      return 0.0;
    }
    for (int i = 0; i < count; ++i)
    {
      polygon[i] = clipped[i];
    }
  }
  double area = 0.0;
  for (int i = 0; i < count; ++i)
  {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1) % count];
    area += a(0) * b(1) - b(0) * a(1);
  }
  return 0.5 * std::abs(area);
}
}  // namespace

int SDFMap::fusedGridValue(int x, int y) const
{
  int base = grid_map_->data[static_cast<size_t>(y) * mp_.map_voxel_num_(0) + x];
  if (base >= grid_occupied_threshold_)
  {
    return base;  // 传感器空闲观测不删除已知静态障碍。
  }
  int local = grid_local_values_.at<signed char>(y, x);
  return local >= 0 ? local : base;
}

bool SDFMap::insideGridWindow(int x, int y) const
{
  double gx = (x + 0.5) * mp_.resolution_;
  double gy = (y + 0.5) * mp_.resolution_;
  double world_x = mp_.map_origin_(0) + grid_cos_yaw_ * gx - grid_sin_yaw_ * gy;
  double world_y = mp_.map_origin_(1) + grid_sin_yaw_ * gx + grid_cos_yaw_ * gy;
  return grid_has_odom_ && std::abs(world_x - grid_boat_pos_(0)) <= grid_local_range_ &&
         std::abs(world_y - grid_boat_pos_(1)) <= grid_local_range_;
}

cv::Rect SDFMap::gridWindow() const
{
  if (!grid_map_ || !grid_has_odom_)
  {
    return cv::Rect();
  }
  Eigen::Vector2d min_index = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d max_index = -min_index;
  for (int x = -1; x <= 1; x += 2)
  {
    for (int y = -1; y <= 1; y += 2)
    {
      Eigen::Vector3d corner(grid_boat_pos_(0) + x * grid_local_range_,
                             grid_boat_pos_(1) + y * grid_local_range_, 0.0);
      Eigen::Vector2d id = gridCoordinates(corner) * mp_.resolution_inv_;
      min_index = min_index.cwiseMin(id);
      max_index = max_index.cwiseMax(id);
    }
  }
  double width = mp_.map_voxel_num_(0), height = mp_.map_voxel_num_(1);
  int x0 = static_cast<int>(std::max(0.0, std::min(width, std::floor(min_index(0)))));
  int y0 = static_cast<int>(std::max(0.0, std::min(height, std::floor(min_index(1)))));
  int x1 = static_cast<int>(std::max(0.0, std::min(width, std::ceil(max_index(0)))));
  int y1 = static_cast<int>(std::max(0.0, std::min(height, std::ceil(max_index(1)))));
  return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

void SDFMap::gridOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  if (msg->header.frame_id != mp_.frame_id_ ||
      !std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y))
  {
    grid_has_odom_ = false;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Local grid needs finite odometry in the configured map frame.");
    return;
  }
  bool had_odom = grid_has_odom_;
  grid_boat_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0;
  grid_has_odom_ = true;
  if (!grid_map_)
  {
    return;
  }
  Eigen::Vector3i center;
  posToIndex(grid_boat_pos_, center);
  if (!had_odom || center(0) != grid_last_window_index_(0) || center(1) != grid_last_window_index_(1))
  {
    grid_last_window_index_ = center;
    grid_window_dirty_ = true;
    md_.esdf_need_update_ = true;
  }
}

void SDFMap::localGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
  const auto& info = msg->info;
  size_t cells = static_cast<size_t>(info.width) * info.height;
  Eigen::Quaterniond q(info.origin.orientation.w, info.origin.orientation.x,
                       info.origin.orientation.y, info.origin.orientation.z);
  if (msg->header.frame_id != mp_.frame_id_ || info.width == 0 || info.height == 0 ||
      cells > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      info.width > static_cast<unsigned int>(std::numeric_limits<int>::max() - 2) ||
      info.height > static_cast<unsigned int>(std::numeric_limits<int>::max() - 2) || msg->data.size() != cells ||
      !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y) ||
      !q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-3)
  {
    RCLCPP_WARN(node_->get_logger(), "Invalid local grid or frame mismatch; observation rejected.");
    return;
  }
  q.normalize();
  if (std::abs(q.toRotationMatrix()(2, 2) - 1.0) > 1e-6)
  {
    RCLCPP_WARN(node_->get_logger(), "Local grid origin must have yaw only.");
    return;
  }
  for (size_t i = 0; i < cells; ++i)
  {
    if (msg->data[i] < -1 || msg->data[i] > 100)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Local OccupancyGrid values must be -1 or 0..100.");
      return;
    }
  }
  int64_t stamp = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                  msg->header.stamp.nanosec;
  if (stamp > 0 && grid_last_local_stamp_ > 0 && stamp < grid_last_local_stamp_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Out-of-order local grid rejected.");
    return;
  }
  if (stamp > 0)
  {
    grid_last_local_stamp_ = stamp;
  }
  // 按时间顺序合并短队列，不只保留最后一帧而丢掉此前看到的障碍。
  if (grid_local_queue_.size() >= 10)
  {
    grid_local_queue_.pop_front();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Local grid processing is behind; oldest queued observation dropped.");
  }
  grid_local_queue_.push_back(msg);
}

bool SDFMap::fuseLocalGrid(const nav_msgs::msg::OccupancyGrid& msg, const cv::Rect& region)
{
  if (region.empty())
  {
    return false;
  }
  Eigen::Quaterniond q(msg.info.origin.orientation.w, msg.info.origin.orientation.x,
                       msg.info.origin.orientation.y, msg.info.origin.orientation.z);
  q.normalize();
  Eigen::Matrix3d rotation = q.toRotationMatrix();
  double c = rotation(0, 0), s = rotation(1, 0), res = msg.info.resolution;
  double ox = msg.info.origin.position.x, oy = msg.info.origin.position.y;
  Eigen::Vector2d offset = gridCoordinates(Eigen::Vector3d(ox, oy, 0.0)) * mp_.resolution_inv_;
  // 常见的同分辨率、同朝向且格线对齐情况直接映射，不做逐格多边形裁剪。
  if (std::abs(res - mp_.resolution_) < 1e-9 && std::abs(c - grid_cos_yaw_) < 1e-9 &&
      std::abs(s - grid_sin_yaw_) < 1e-9 && offset.allFinite() &&
      std::abs(offset(0)) < std::numeric_limits<int>::max() / 2.0 &&
      std::abs(offset(1)) < std::numeric_limits<int>::max() / 2.0 &&
      std::abs(offset(0) - std::round(offset(0))) < 1e-4 &&
      std::abs(offset(1) - std::round(offset(1))) < 1e-4)
  {
    int offset_x = static_cast<int>(std::round(offset(0)));
    int offset_y = static_cast<int>(std::round(offset(1)));
    bool changed = false;
    for (int y = region.y; y < region.y + region.height; ++y)
    {
      for (int x = region.x; x < region.x + region.width; ++x)
      {
        int lx = x - offset_x, ly = y - offset_y;
        if (!insideGridWindow(x, y) || lx < 0 || ly < 0 ||
            lx >= static_cast<int>(msg.info.width) || ly >= static_cast<int>(msg.info.height)) continue;
        int value = msg.data[static_cast<size_t>(ly) * msg.info.width + lx];
        if (value < 0)
        {
          continue;
        }
        grid_local_values_.at<signed char>(y, x) = value >= grid_occupied_threshold_ ? value : 0;
        int fused = fusedGridValue(x, y);
        if (fused != grid_fused_.at<signed char>(y, x))
        {
          grid_fused_.at<signed char>(y, x) = fused;
          changed = true;
        }
      }
    }
    return changed;
  }
  // 仅访问与本次窗口重叠的局部图区域。
  Eigen::Vector2d local_min = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d local_max = -local_min;
  for (int x = -1; x <= 1; x += 2)
  {
    for (int y = -1; y <= 1; y += 2)
    {
      double dx = grid_boat_pos_(0) + x * grid_local_range_ - ox;
      double dy = grid_boat_pos_(1) + y * grid_local_range_ - oy;
      Eigen::Vector2d id((c * dx + s * dy) / res, (-s * dx + c * dy) / res);
      local_min = local_min.cwiseMin(id);
      local_max = local_max.cwiseMax(id);
    }
  }
  int lx0 = static_cast<int>(std::max(0.0, std::min(double(msg.info.width), std::floor(local_min(0)))));
  int ly0 = static_cast<int>(std::max(0.0, std::min(double(msg.info.height), std::floor(local_min(1)))));
  int lx1 = static_cast<int>(std::max(0.0, std::min(double(msg.info.width), std::ceil(local_max(0)))));
  int ly1 = static_cast<int>(std::max(0.0, std::min(double(msg.info.height), std::ceil(local_max(1)))));
  cv::Mat occupied(region.height, region.width, CV_8SC1, cv::Scalar(-1));
  cv::Mat free_area(region.height, region.width, CV_32FC1, cv::Scalar(0));
  const int corner_x[4] = {0, 1, 1, 0};
  const int corner_y[4] = {0, 0, 1, 1};
  for (int ly = ly0; ly < ly1; ++ly)
  {
    for (int lx = lx0; lx < lx1; ++lx)
    {
      int value = msg.data[static_cast<size_t>(ly) * msg.info.width + lx];
      if (value < 0)
      {
        continue;  // 未观测部分不清除任何历史数据。
      }
      Eigen::Vector2d corners[4];
      Eigen::Vector2d min_id = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
      Eigen::Vector2d max_id = -min_id;
      for (int i = 0; i < 4; ++i)
      {
        double px = (lx + corner_x[i]) * res, py = (ly + corner_y[i]) * res;
        Eigen::Vector3d world(ox + c * px - s * py, oy + s * px + c * py, 0.0);
        corners[i] = gridCoordinates(world) * mp_.resolution_inv_;
        if (!corners[i].allFinite())
        {
          return false;
        }
        min_id = min_id.cwiseMin(corners[i]);
        max_id = max_id.cwiseMax(corners[i]);
      }
      if (max_id(0) <= region.x || min_id(0) >= region.x + region.width ||
          max_id(1) <= region.y || min_id(1) >= region.y + region.height) continue;
      int gx0 = static_cast<int>(std::max(double(region.x), std::floor(min_id(0))));
      int gy0 = static_cast<int>(std::max(double(region.y), std::floor(min_id(1))));
      int gx1 = static_cast<int>(std::min(double(region.x + region.width), std::ceil(max_id(0))));
      int gy1 = static_cast<int>(std::min(double(region.y + region.height), std::ceil(max_id(1))));
      for (int gy = gy0; gy < gy1; ++gy)
      {
        for (int gx = gx0; gx < gx1; ++gx)
        {
          if (!insideGridWindow(gx, gy))
          {
            continue;
          }
          double area = cellOverlapArea(corners, gx, gy);
          if (area <= 1e-8)
          {
            continue;
          }
          int y = gy - region.y, x = gx - region.x;
          if (value >= grid_occupied_threshold_)
          {
            occupied.at<signed char>(y, x) = std::max(int(occupied.at<signed char>(y, x)), value);
          }
          else
          {
            free_area.at<float>(y, x) += area;
          }
        }
      }
    }
  }
  bool changed = false;
  for (int y = 0; y < region.height; ++y)
  {
    for (int x = 0; x < region.width; ++x)
    {
      int gx = region.x + x, gy = region.y + y;
      if (!insideGridWindow(gx, gy))
      {
        continue;
      }
      int value = occupied.at<signed char>(y, x);
      if (value < 0 && free_area.at<float>(y, x) >= 1.0 - 1e-5)
      {
        value = 0;
      }
      if (value < 0)
      {
        continue;
      }
      grid_local_values_.at<signed char>(gy, gx) = value;
      int fused = fusedGridValue(gx, gy);
      if (fused != grid_fused_.at<signed char>(gy, gx))
      {
        grid_fused_.at<signed char>(gy, gx) = fused;
        changed = true;
      }
    }
  }
  return changed;
}

void SDFMap::publishGridMap(bool full, const cv::Rect& region)
{
  if (!grid_map_ready_)
  {
    return;
  }
  if (full)
  {
    nav_msgs::msg::OccupancyGrid fused, inflated;
    fused.header = grid_map_->header;
    fused.header.stamp = node_->get_clock()->now();
    fused.info = grid_map_->info;
    inflated.header = fused.header;
    inflated.info = fused.info;
    fused.data.resize(grid_map_->data.size());
    inflated.data.resize(grid_map_->data.size());
    for (int y = 0; y < grid_fused_.rows; ++y)
    {
      for (int x = 0; x < grid_fused_.cols; ++x)
      {
        size_t address = static_cast<size_t>(y) * grid_fused_.cols + x;
        fused.data[address] = grid_fused_.at<signed char>(y, x);
        inflated.data[address] = grid_inflate_.at<unsigned char>(y, x) ? 100 : 0;
      }
    }
    grid_fused_pub_->publish(fused);
    grid_inflate_pub_->publish(inflated);
    grid_last_snapshot_time_ = node_->get_clock()->now();
    grid_fused_subscribers_ = grid_fused_pub_->get_subscription_count();
    grid_inflate_subscribers_ = grid_inflate_pub_->get_subscription_count();
  }
  else if (!region.empty() && (grid_fused_update_pub_->get_subscription_count() > 0 ||
                               grid_inflate_update_pub_->get_subscription_count() > 0))
  {
    map_msgs::msg::OccupancyGridUpdate fused, inflated;
    fused.header = grid_map_->header;
    fused.header.stamp = node_->get_clock()->now();
    fused.x = region.x; fused.y = region.y;
    fused.width = region.width; fused.height = region.height;
    inflated.header = fused.header;
    inflated.x = fused.x; inflated.y = fused.y;
    inflated.width = fused.width; inflated.height = fused.height;
    fused.data.resize(static_cast<size_t>(region.width) * region.height);
    inflated.data.resize(fused.data.size());
    for (int y = 0; y < region.height; ++y)
    {
      for (int x = 0; x < region.width; ++x)
      {
        size_t address = static_cast<size_t>(y) * region.width + x;
        fused.data[address] = grid_fused_.at<signed char>(region.y + y, region.x + x);
        inflated.data[address] = grid_inflate_.at<unsigned char>(region.y + y, region.x + x) ? 100 : 0;
      }
    }
    grid_fused_update_pub_->publish(fused);
    grid_inflate_update_pub_->publish(inflated);
  }
  if (!grid_publish_esdf_)
  {
    return;
  }
  cv::Rect shown = full ? cv::Rect(0, 0, grid_distance_.cols, grid_distance_.rows) : region;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int y = shown.y; y < shown.y + shown.height; y += grid_visualization_stride_)
  {
    for (int x = shown.x; x < shown.x + shown.width; x += grid_visualization_stride_)
    {
      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 0), pos);
      pcl::PointXYZI point;
      point.x = pos(0); point.y = pos(1); point.z = 0.0;
      point.intensity = grid_distance_.at<float>(y, x);
      cloud.push_back(point);
    }
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = grid_map_->header;
  msg.header.stamp = node_->get_clock()->now();
  esdf_pub_->publish(msg);
}

void SDFMap::checkGridPublication()
{
  if (!grid_map_ready_)
  {
    return;
  }
  size_t fused_count = grid_fused_pub_->get_subscription_count();
  size_t inflate_count = grid_inflate_pub_->get_subscription_count();
  double elapsed = (node_->get_clock()->now() - grid_last_snapshot_time_).seconds();
  if (fused_count > grid_fused_subscribers_ || inflate_count > grid_inflate_subscribers_ ||
      ((fused_count > 0 || inflate_count > 0) && (elapsed < 0.0 || elapsed >= grid_snapshot_period_)))
  {
    publishGridMap();  // 为晚加入订阅者及补丁接收者提供当前完整快照，不重算距离场。
  }
  grid_fused_subscribers_ = fused_count;
  grid_inflate_subscribers_ = inflate_count;
}
