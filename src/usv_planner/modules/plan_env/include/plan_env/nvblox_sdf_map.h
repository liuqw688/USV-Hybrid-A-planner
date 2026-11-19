/**
 * Nvblox SDF Map Adapter
 * 
 * This adapter provides an nvblox-based backend for Fast-Planner's SDFMap interface.
 * It subscribes to nvblox topics and converts the data to Fast-Planner's expected format.
 * 
 * This is an optional feature - Fast-Planner will fall back to the standard SDF map
 * if nvblox is not available.
 */

#ifndef _NVBLOX_SDF_MAP_H
#define _NVBLOX_SDF_MAP_H

#ifdef USE_NVBLOX

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <mutex>
#include <memory>

namespace fast_planner {

/**
 * NvbloxSDFMapAdapter
 * 
 * Adapter class that interfaces with nvblox to provide ESDF and occupancy data
 * to Fast-Planner. This maintains the same interface as SDFMap but uses nvblox
 * as the backend for GPU-accelerated mapping.
 */
class NvbloxSDFMapAdapter {
public:
  NvbloxSDFMapAdapter();
  ~NvbloxSDFMapAdapter();

  /**
   * Initialize the adapter with ROS node
   * @param nh Shared pointer to ROS node
   * @return true if nvblox topics are available, false otherwise
   */
  bool init(const std::shared_ptr<rclcpp::Node>& nh);

  /**
   * Check if nvblox is available and ready
   */
  bool isAvailable() const { return nvblox_available_; }

  /**
   * Get distance to nearest obstacle at position (ESDF query)
   * @param pos 3D position in world frame
   * @return Distance to obstacle (positive = free space, negative = inside obstacle)
   */
  double getDistance(const Eigen::Vector3d& pos);

  /**
   * Get occupancy status at position
   * @param pos 3D position in world frame
   * @return 1 if occupied, 0 if free, -1 if unknown
   */
  int getOccupancy(const Eigen::Vector3d& pos);

  /**
   * Get inflated occupancy (with safety margin)
   * @param pos 3D position in world frame
   * @return 1 if occupied (including inflation), 0 if free
   */
  int getInflateOccupancy(const Eigen::Vector3d& pos);

  /**
   * Check if position is known to be occupied
   */
  bool isKnownOccupied(const Eigen::Vector3d& pos);

  /**
   * Check if position is known to be free
   */
  bool isKnownFree(const Eigen::Vector3d& pos);

  /**
   * Check if position is unknown (not observed)
   */
  bool isUnknown(const Eigen::Vector3d& pos);

  /**
   * Get distance with gradient (for optimization)
   * @param pos 3D position
   * @param grad Output gradient vector
   * @return Distance to obstacle
   */
  double getDistWithGradTrilinear(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

  /**
   * Get map resolution
   */
  double getResolution() const { return resolution_; }

  /**
   * Get map origin
   */
  Eigen::Vector3d getOrigin() const { return map_origin_; }

  /**
   * Check if odometry is valid
   */
  bool odomValid() const { return has_odom_; }

private:
  // ROS node and subscribers
  std::shared_ptr<rclcpp::Node> node_;
  
  // Nvblox topic subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_slice_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Callbacks
  void esdfSliceCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void occupancyCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  // Map data storage (converted from nvblox sparse format)
  // Using a hash map for sparse voxel storage
  struct VoxelData {
    double distance;      // ESDF distance
    bool occupied;        // Occupancy status
    bool inflated;        // Inflated occupancy
    rclcpp::Time timestamp;
  };
  
  std::unordered_map<uint64_t, VoxelData> voxel_map_;
  std::mutex map_mutex_;
  
  // Map parameters
  double resolution_;
  double obstacles_inflation_;
  Eigen::Vector3d map_origin_;
  Eigen::Vector3d map_size_;
  Eigen::Vector3d map_min_boundary_;
  Eigen::Vector3d map_max_boundary_;

  // State flags
  bool nvblox_available_;
  bool has_odom_;
  bool has_esdf_data_;
  bool has_occupancy_data_;
  
  Eigen::Vector3d camera_pos_;

  // Helper functions
  uint64_t posToVoxelIndex(const Eigen::Vector3d& pos) const;
  Eigen::Vector3d voxelIndexToPos(uint64_t idx) const;
  bool isInMap(const Eigen::Vector3d& pos) const;
  
  // Convert nvblox point cloud to internal format
  void updateMapFromPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, bool is_esdf);
  
  // Apply inflation to occupancy data
  void inflateOccupancy();
};

} // namespace fast_planner

#endif // USE_NVBLOX

#endif // _NVBLOX_SDF_MAP_H



