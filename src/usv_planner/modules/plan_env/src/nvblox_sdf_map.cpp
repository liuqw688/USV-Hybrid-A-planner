/**
 * Nvblox SDF Map Adapter Implementation
 * 
 * This file implements the nvblox adapter for Fast-Planner's SDFMap interface.
 * It subscribes to nvblox topics and provides the same interface as SDFMap.
 */

#ifdef USE_NVBLOX

#include "plan_env/nvblox_sdf_map.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <algorithm>

// Note: SDFMap is not in a namespace, so we'll use the same approach
// For compatibility, we'll use the fast_planner namespace for the adapter
namespace fast_planner {

NvbloxSDFMapAdapter::NvbloxSDFMapAdapter()
  : resolution_(0.15)
  , obstacles_inflation_(0.05)
  , map_origin_(-10.0, -10.0, -1.0)
  , map_size_(20.0, 20.0, 4.0)
  , map_min_boundary_(map_origin_)
  , map_max_boundary_(map_origin_ + map_size_)
  , nvblox_available_(false)
  , has_odom_(false)
  , has_esdf_data_(false)
  , has_occupancy_data_(false)
{
}

NvbloxSDFMapAdapter::~NvbloxSDFMapAdapter()
{
}

bool NvbloxSDFMapAdapter::init(const std::shared_ptr<rclcpp::Node>& nh)
{
  node_ = nh;
  
  // Get parameters
  node_->declare_parameter<double>("sdf_map/resolution", 0.15);
  node_->declare_parameter<double>("sdf_map/map_size_x", 20.0);
  node_->declare_parameter<double>("sdf_map/map_size_y", 20.0);
  node_->declare_parameter<double>("sdf_map/map_size_z", 4.0);
  node_->declare_parameter<double>("sdf_map/obstacles_inflation", 0.05);
  node_->declare_parameter<std::string>("nvblox/esdf_topic", "/nvblox_node/esdf_slice");
  node_->declare_parameter<std::string>("nvblox/occupancy_topic", "/nvblox_node/static_map_slice");
  
  node_->get_parameter("sdf_map/resolution", resolution_);
  double x_size, y_size, z_size;
  node_->get_parameter("sdf_map/map_size_x", x_size);
  node_->get_parameter("sdf_map/map_size_y", y_size);
  node_->get_parameter("sdf_map/map_size_z", z_size);
  node_->get_parameter("sdf_map/obstacles_inflation", obstacles_inflation_);
  
  std::string esdf_topic, occupancy_topic;
  node_->get_parameter("nvblox/esdf_topic", esdf_topic);
  node_->get_parameter("nvblox/occupancy_topic", occupancy_topic);
  
  map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, -1.0);
  map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  map_min_boundary_ = map_origin_;
  map_max_boundary_ = map_origin_ + map_size_;

  // Create subscriptions to nvblox topics with BEST_EFFORT QoS for better performance
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();
  sensor_qos.durability_volatile();
  
  esdf_slice_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    esdf_topic, sensor_qos,
    std::bind(&NvbloxSDFMapAdapter::esdfSliceCallback, this, std::placeholders::_1));
  
  occupancy_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    occupancy_topic, sensor_qos,
    std::bind(&NvbloxSDFMapAdapter::occupancyCallback, this, std::placeholders::_1));
  
  rclcpp::QoS odom_qos(10);
  odom_qos.best_effort();
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/sdf_map/odom", odom_qos,
    std::bind(&NvbloxSDFMapAdapter::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), 
              "[NvbloxAdapter] Initialized. Subscribing to ESDF: %s, Occupancy: %s",
              esdf_topic.c_str(), occupancy_topic.c_str());
  
  // Mark as available (will be updated when data arrives)
  nvblox_available_ = true;
  
  return true;
}

void NvbloxSDFMapAdapter::esdfSliceCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  updateMapFromPointCloud(msg, true);
  has_esdf_data_ = true;
}

void NvbloxSDFMapAdapter::occupancyCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  updateMapFromPointCloud(msg, false);
  has_occupancy_data_ = true;
  inflateOccupancy();
}

void NvbloxSDFMapAdapter::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  camera_pos_(0) = msg->pose.pose.position.x;
  camera_pos_(1) = msg->pose.pose.position.y;
  camera_pos_(2) = msg->pose.pose.position.z;
  has_odom_ = true;
}

void NvbloxSDFMapAdapter::updateMapFromPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, bool is_esdf)
{
  // Convert point cloud to internal format
  // Nvblox publishes point clouds with intensity = distance (for ESDF) or occupancy (for occupancy map)
  // PointCloud2Iterator requires non-const reference, so create a mutable copy
  sensor_msgs::msg::PointCloud2 msg_copy = *msg;
  
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_copy, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_copy, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_copy, "z");
  
  bool has_intensity = false;
  sensor_msgs::PointCloud2Iterator<float> iter_intensity;
  try {
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(msg_copy, "intensity");
    has_intensity = true;
  } catch (...) {
    // No intensity field - try "i" as alternative
    try {
      iter_intensity = sensor_msgs::PointCloud2Iterator<float>(msg_copy, "i");
      has_intensity = true;
    } catch (...) {
      // No intensity field at all
    }
  }
  
  size_t point_count = msg->width * msg->height;
  
  for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;
    
    // Skip invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      if (has_intensity) ++iter_intensity;
      continue;
    }
    
    Eigen::Vector3d pos(x, y, z);
    
    if (!isInMap(pos)) {
      if (has_intensity) ++iter_intensity;
      continue;
    }
    
    uint64_t idx = posToVoxelIndex(pos);
    VoxelData& voxel = voxel_map_[idx];
    
    if (is_esdf && has_intensity) {
      // ESDF: intensity = distance to obstacle
      voxel.distance = *iter_intensity;
      ++iter_intensity;
    } else if (!is_esdf) {
      // Occupancy: intensity > threshold = occupied
      if (has_intensity) {
        voxel.occupied = (*iter_intensity > 0.5);
        ++iter_intensity;
      } else {
        // If no intensity, assume point cloud represents occupied voxels
        voxel.occupied = true;
      }
    }
    
    voxel.timestamp = msg->header.stamp;
  }
}

void NvbloxSDFMapAdapter::inflateOccupancy()
{
  // Apply inflation to occupied voxels
  std::unordered_map<uint64_t, VoxelData> inflated_map = voxel_map_;
  
  int inf_step = static_cast<int>(std::ceil(obstacles_inflation_ / resolution_));
  
  for (const auto& [idx, voxel] : voxel_map_) {
    if (!voxel.occupied) continue;
    
    Eigen::Vector3d pos = voxelIndexToPos(idx);
    
    // Inflate in all directions
    for (int x = -inf_step; x <= inf_step; ++x) {
      for (int y = -inf_step; y <= inf_step; ++y) {
        for (int z = -inf_step; z <= inf_step; ++z) {
          Eigen::Vector3d inf_pos = pos + Eigen::Vector3d(
            x * resolution_, y * resolution_, z * resolution_);
          
          if (!isInMap(inf_pos)) continue;
          
          uint64_t inf_idx = posToVoxelIndex(inf_pos);
          inflated_map[inf_idx].inflated = true;
        }
      }
    }
  }
  
  // Update map with inflated data
  for (auto& [idx, voxel] : voxel_map_) {
    if (inflated_map.find(idx) != inflated_map.end()) {
      voxel.inflated = inflated_map[idx].inflated;
    }
  }
}

uint64_t NvbloxSDFMapAdapter::posToVoxelIndex(const Eigen::Vector3d& pos) const
{
  Eigen::Vector3d rel_pos = pos - map_origin_;
  int x = static_cast<int>(std::floor(rel_pos(0) / resolution_));
  int y = static_cast<int>(std::floor(rel_pos(1) / resolution_));
  int z = static_cast<int>(std::floor(rel_pos(2) / resolution_));
  
  // Pack into 64-bit index (21 bits per axis)
  return (static_cast<uint64_t>(x) << 42) | 
         (static_cast<uint64_t>(y) << 21) | 
         static_cast<uint64_t>(z);
}

Eigen::Vector3d NvbloxSDFMapAdapter::voxelIndexToPos(uint64_t idx) const
{
  int z = static_cast<int>(idx & 0x1FFFFF);
  int y = static_cast<int>((idx >> 21) & 0x1FFFFF);
  int x = static_cast<int>((idx >> 42) & 0x1FFFFF);
  
  // Convert from signed to unsigned if needed
  if (x > 1048576) x -= 2097152;
  if (y > 1048576) y -= 2097152;
  if (z > 1048576) z -= 2097152;
  
  return map_origin_ + Eigen::Vector3d(
    (x + 0.5) * resolution_,
    (y + 0.5) * resolution_,
    (z + 0.5) * resolution_);
}

bool NvbloxSDFMapAdapter::isInMap(const Eigen::Vector3d& pos) const
{
  return (pos(0) >= map_min_boundary_(0) && pos(0) < map_max_boundary_(0) &&
          pos(1) >= map_min_boundary_(1) && pos(1) < map_max_boundary_(1) &&
          pos(2) >= map_min_boundary_(2) && pos(2) < map_max_boundary_(2));
}

double NvbloxSDFMapAdapter::getDistance(const Eigen::Vector3d& pos)
{
  if (!isInMap(pos)) return 10000.0; // Large distance for out-of-map
  
  std::lock_guard<std::mutex> lock(map_mutex_);
  uint64_t idx = posToVoxelIndex(pos);
  
  auto it = voxel_map_.find(idx);
  if (it != voxel_map_.end()) {
    return it->second.distance;
  }
  
  return 10000.0; // Unknown = large distance
}

int NvbloxSDFMapAdapter::getOccupancy(const Eigen::Vector3d& pos)
{
  if (!isInMap(pos)) return -1;
  
  std::lock_guard<std::mutex> lock(map_mutex_);
  uint64_t idx = posToVoxelIndex(pos);
  
  auto it = voxel_map_.find(idx);
  if (it != voxel_map_.end()) {
    return it->second.occupied ? 1 : 0;
  }
  
  return -1; // Unknown
}

int NvbloxSDFMapAdapter::getInflateOccupancy(const Eigen::Vector3d& pos)
{
  if (!isInMap(pos)) return -1;
  
  std::lock_guard<std::mutex> lock(map_mutex_);
  uint64_t idx = posToVoxelIndex(pos);
  
  auto it = voxel_map_.find(idx);
  if (it != voxel_map_.end()) {
    return (it->second.occupied || it->second.inflated) ? 1 : 0;
  }
  
  return -1; // Unknown
}

bool NvbloxSDFMapAdapter::isKnownOccupied(const Eigen::Vector3d& pos)
{
  return getInflateOccupancy(pos) == 1;
}

bool NvbloxSDFMapAdapter::isKnownFree(const Eigen::Vector3d& pos)
{
  int occ = getInflateOccupancy(pos);
  return occ == 0;
}

bool NvbloxSDFMapAdapter::isUnknown(const Eigen::Vector3d& pos)
{
  return getOccupancy(pos) == -1;
}

double NvbloxSDFMapAdapter::getDistWithGradTrilinear(const Eigen::Vector3d& pos, Eigen::Vector3d& grad)
{
  // Simple trilinear interpolation for gradient
  grad.setZero();
  
  if (!isInMap(pos)) return 10000.0;
  
  // Get 8 surrounding voxels
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3d rel_pos = pos_m - map_origin_;
  int x0 = static_cast<int>(std::floor(rel_pos(0) / resolution_));
  int y0 = static_cast<int>(std::floor(rel_pos(1) / resolution_));
  int z0 = static_cast<int>(std::floor(rel_pos(2) / resolution_));
  
  double dx = (rel_pos(0) / resolution_) - x0;
  double dy = (rel_pos(1) / resolution_) - y0;
  double dz = (rel_pos(2) / resolution_) - z0;
  
  double values[2][2][2];
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        Eigen::Vector3d voxel_pos = map_origin_ + Eigen::Vector3d(
          (x0 + i + 0.5) * resolution_,
          (y0 + j + 0.5) * resolution_,
          (z0 + k + 0.5) * resolution_);
        values[i][j][k] = getDistance(voxel_pos);
      }
    }
  }
  
  // Trilinear interpolation
  double v00 = (1 - dx) * values[0][0][0] + dx * values[1][0][0];
  double v01 = (1 - dx) * values[0][0][1] + dx * values[1][0][1];
  double v10 = (1 - dx) * values[0][1][0] + dx * values[1][1][0];
  double v11 = (1 - dx) * values[0][1][1] + dx * values[1][1][1];
  double v0 = (1 - dy) * v00 + dy * v10;
  double v1 = (1 - dy) * v01 + dy * v11;
  double dist = (1 - dz) * v0 + dz * v1;
  
  // Gradient computation
  grad[2] = (v1 - v0) / resolution_;
  grad[1] = ((1 - dz) * (v10 - v00) + dz * (v11 - v01)) / resolution_;
  grad[0] = (1 - dz) * (1 - dy) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - dz) * dy * (values[1][1][0] - values[0][1][0]);
  grad[0] += dz * (1 - dy) * (values[1][0][1] - values[0][0][1]);
  grad[0] += dz * dy * (values[1][1][1] - values[0][1][1]);
  grad[0] /= resolution_;
  
  return dist;
}

} // namespace fast_planner

#endif // USE_NVBLOX

