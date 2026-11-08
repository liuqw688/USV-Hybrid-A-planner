#pragma once
#include <usv_kinodynamic_frontend/waterway_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace usv_kinodynamic_frontend {
// Conversion helper only: no subscriptions, publishers, TF assumptions or changes
// to the existing local_costmap / Hybrid A* / MPC interfaces.
inline WaterwayMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& message) {
  if (message.header.frame_id.empty() || message.info.width>static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      message.info.height>static_cast<uint32_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("occupancy grid requires a frame and bounded dimensions");
  const auto& q=message.info.origin.orientation;
  const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (!std::isfinite(norm) || norm<1e-9)
    throw std::invalid_argument("invalid occupancy grid origin quaternion");
  const double x=q.x/norm,y=q.y/norm,z=q.z/norm,w=q.w/norm;
  if (std::abs(2*(x*z-w*y))>1e-6 || std::abs(2*(y*z+w*x))>1e-6 ||
      1-2*(x*x+y*y)<1-1e-6)
    throw std::invalid_argument("waterway grid origin must be planar");
  WaterwayMap map;
  map.frame_id=message.header.frame_id;
  map.width=static_cast<int>(message.info.width);map.height=static_cast<int>(message.info.height);
  map.resolution=message.info.resolution;
  map.origin_x=message.info.origin.position.x;map.origin_y=message.info.origin.position.y;
  map.origin_yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
  map.cells=message.data;
  map.validate();
  return map;
}
}  // namespace usv_kinodynamic_frontend
