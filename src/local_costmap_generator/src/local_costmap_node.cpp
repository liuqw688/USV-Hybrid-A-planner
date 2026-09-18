#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class LocalCostmapNode : public rclcpp::Node
{
public:
  // [功能与联系] 加载膨胀阈值和距离参数，订阅原图并锁存发布/local_costmap；输入到达才构图，不靠频率反复重建。
  LocalCostmapNode()
  : Node("local_costmap_node")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/local_obstacle_map");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/local_costmap");

    occupied_threshold_ = declare_parameter<int>(
      "occupied_threshold", 90);

    unknown_is_obstacle_ = declare_parameter<bool>(
      "unknown_is_obstacle", true);

    robot_radius_ = declare_parameter<double>(
      "robot_radius", 5.0);

    inflation_radius_ = declare_parameter<double>(
      "inflation_radius", 70.0);

    minimum_cost_ = declare_parameter<int>(
      "minimum_cost", 1);

    // 新增参数：高代价区域阈值和衰减因子
    high_cost_threshold_ = declare_parameter<int>(
      "high_cost_threshold", 60);

    decay_factor_ = declare_parameter<double>(
      "decay_factor", 3.0);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_,
      map_qos,
      std::bind(&LocalCostmapNode::mapCallback, this, std::placeholders::_1));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, map_qos);

    RCLCPP_INFO(
      get_logger(),
      "Local costmap generator started. input=%s output=%s "
      "robot_radius=%.2f inflation_radius=%.2f high_cost_thres=%d decay=%.2f",
      input_topic_.c_str(),
      output_topic_.c_str(),
      robot_radius_,
      inflation_radius_,
      high_cost_threshold_,
      decay_factor_);
  }

private:
  struct QueueCell
  {
    float distance;
    int source_cost; // 记录起点的初始代价
    int x;
    int y;
  };

  struct CompareQueueCell
  {
    bool operator()(const QueueCell & a, const QueueCell & b) const
    {
      return a.distance > b.distance;
    }
  };

  // [功能与联系] 验证输入栅格，保持坐标元数据，调用buildCostmap并发布；给航道管理器和两级规划器提供对齐底图。
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr obstacle_map)
  {
    if (!isValidMap(*obstacle_map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received invalid OccupancyGrid from %s",
        input_topic_.c_str());
      return;
    }

    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header = obstacle_map->header;
    costmap.info = obstacle_map->info;
    costmap.data.assign(obstacle_map->data.size(), 0);

    buildCostmap(*obstacle_map, costmap);

    costmap_pub_->publish(costmap);
  }

  // [功能与联系] 核对非零尺寸、正分辨率及data长度，避免距离变换越界；由mapCallback前置调用。
  bool isValidMap(const nav_msgs::msg::OccupancyGrid & map) const
  {
    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0) {
      return false;
    }

    const std::size_t expected_size =
      static_cast<std::size_t>(map.info.width) *
      static_cast<std::size_t>(map.info.height);

    return map.data.size() == expected_size;
  }

  // [功能与联系] 识别负栅格为未知；当前活动实现始终保留-1，unknown_is_obstacle参数未参与此判定。
  bool isUnknown(const int8_t occupancy) const
  {
    return occupancy < 0;
  }

  // [功能与联系] 将二维格坐标变为数组下标；构图的距离场、源代价和最终输出共用。
  std::size_t indexOf(
    const int x,
    const int y,
    const std::uint32_t width) const
  {
    return static_cast<std::size_t>(y) * width +
      static_cast<std::size_t>(x);
  }

  // [功能与联系] 分别为硬障碍与高代价种子执行8邻域距离传播，再按半径及指数衰减合并；输出梯度不是左右航道标签。
  void buildCostmap(
    const nav_msgs::msg::OccupancyGrid & obstacle_map,
    nav_msgs::msg::OccupancyGrid & costmap)
  {
    const int width = static_cast<int>(obstacle_map.info.width);
    const int height = static_cast<int>(obstacle_map.info.height);
    const float resolution = static_cast<float>(obstacle_map.info.resolution);

    const float infinity = std::numeric_limits<float>::infinity();
    const std::size_t total_size = obstacle_map.data.size();

    // 1. 初始化距离场和代价场
    std::vector<float> obs_dist(total_size, infinity);
    std::vector<int> obs_cost(total_size, 0);
    std::priority_queue<QueueCell, std::vector<QueueCell>, CompareQueueCell> obs_queue;

    std::vector<float> high_dist(total_size, infinity);
    std::vector<int> high_cost(total_size, 0);
    std::priority_queue<QueueCell, std::vector<QueueCell>, CompareQueueCell> high_queue;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = indexOf(x, y, width);
        int8_t occ = obstacle_map.data[index];

        if (isUnknown(occ)) {
          costmap.data[index] = -1; // 未知区域强制为 -1
        } 
        else if (occ >= occupied_threshold_) {
          // 物理障碍物起点
          obs_dist[index] = 0.0F;
          obs_cost[index] = 100;
          obs_queue.push({0.0F, 100, x, y});
          costmap.data[index] = 100;
        } 
        else if (occ >= high_cost_threshold_) {
          // 高代价区域起点
          high_dist[index] = 0.0F;
          high_cost[index] = static_cast<int>(occ);
          high_queue.push({0.0F, static_cast<int>(occ), x, y});
          costmap.data[index] = static_cast<int8_t>(occ);
        } 
        else {
          costmap.data[index] = 0;
        }
      }
    }

    // 2. 障碍物 Dijkstra 距离变换
    constexpr int kNeighborCount = 8;
    const int offset_x[kNeighborCount] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int offset_y[kNeighborCount] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    float obs_max_dist = static_cast<float>(robot_radius_ + inflation_radius_);

    while (!obs_queue.empty()) {
      const QueueCell current = obs_queue.top();
      obs_queue.pop();

      const std::size_t current_index = indexOf(current.x, current.y, width);
      if (current.distance > obs_dist[current_index]) continue;
      if (current.distance > obs_max_dist) continue;

      for (int i = 0; i < kNeighborCount; ++i) {
        const int next_x = current.x + offset_x[i];
        const int next_y = current.y + offset_y[i];

        if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) continue;

        const float step_distance = ((offset_x[i] == 0 || offset_y[i] == 0) ? 1.0F : 1.41421356F) * resolution;
        const float next_distance = current.distance + step_distance;

        if (next_distance > obs_max_dist) continue;

        const std::size_t next_index = indexOf(next_x, next_y, width);
        if (next_distance < obs_dist[next_index]) {
          obs_dist[next_index] = next_distance;
          obs_cost[next_index] = current.source_cost;
          obs_queue.push({next_distance, current.source_cost, next_x, next_y});
        }
      }
    }

    // 3. 高代价区域 Dijkstra 距离变换
    float high_max_dist = static_cast<float>(inflation_radius_);

    while (!high_queue.empty()) {
      const QueueCell current = high_queue.top();
      high_queue.pop();

      const std::size_t current_index = indexOf(current.x, current.y, width);
      if (current.distance > high_dist[current_index]) continue;
      if (current.distance > high_max_dist) continue;

      for (int i = 0; i < kNeighborCount; ++i) {
        const int next_x = current.x + offset_x[i];
        const int next_y = current.y + offset_y[i];

        if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) continue;

        const float step_distance = ((offset_x[i] == 0 || offset_y[i] == 0) ? 1.0F : 1.41421356F) * resolution;
        const float next_distance = current.distance + step_distance;

        if (next_distance > high_max_dist) continue;

        const std::size_t next_index = indexOf(next_x, next_y, width);
        if (next_distance < high_dist[next_index]) {
          high_dist[next_index] = next_distance;
          high_cost[next_index] = current.source_cost;
          high_queue.push({next_distance, current.source_cost, next_x, next_y});
        }
      }
    }

    // 4. 合并计算最终代价
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = indexOf(x, y, width);
        int8_t original_occ = obstacle_map.data[index];

        // 未知区域强制保持为 -1
        if (isUnknown(original_occ)) {
          costmap.data[index] = -1;
          continue;
        }

        int final_cost = 0;

        // 计算障碍物膨胀代价
        float d_obs = obs_dist[index];
        if (std::isfinite(d_obs) && d_obs <= obs_max_dist) {
          int c_obs = 0;
          if (d_obs <= robot_radius_) {
            c_obs = 99; // 船体半径内高代价
          } else {
            float dist_from_robot = d_obs - static_cast<float>(robot_radius_);
            float norm = std::clamp(dist_from_robot / static_cast<float>(inflation_radius_), 0.0F, 1.0F);
            float cf = static_cast<float>(minimum_cost_) + (98.0F - static_cast<float>(minimum_cost_)) * std::exp(-decay_factor_ * norm);
            c_obs = static_cast<int>(std::round(cf));
          }
          final_cost = std::max(final_cost, c_obs);
        }

        // 计算高代价区域膨胀代价
        float d_high = high_dist[index];
        if (std::isfinite(d_high) && d_high <= high_max_dist) {
          int base_high = high_cost[index];
          float norm_high = std::clamp(d_high / static_cast<float>(inflation_radius_), 0.0F, 1.0F);
          float cf_high = static_cast<float>(minimum_cost_) + 
                          (static_cast<float>(base_high) 
                          - static_cast<float>(minimum_cost_)) * std::exp(-decay_factor_ * norm_high);
          int c_high = static_cast<int>(std::round(cf_high));
          final_cost = std::max(final_cost, c_high);
        }

        costmap.data[index] = static_cast<int8_t>(std::clamp(final_cost, 0, 100));
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  int occupied_threshold_{90};
  bool unknown_is_obstacle_{true};
  double robot_radius_{5.0};
  double inflation_radius_{70.0};
  int minimum_cost_{1};
  
  // 新增成员变量
  int high_cost_threshold_{60};
  double decay_factor_{3.0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
