#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

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
      "occupied_threshold", 50);

    unknown_is_obstacle_ = declare_parameter<bool>(
      "unknown_is_obstacle", true);

    // 船体外接半径。若障碍物地图已经完成了船体半径膨胀，设为 0.0。
    robot_radius_ = declare_parameter<double>(
      "robot_radius", 0.6);

    // 障碍物外侧继续生成代价梯度的范围。
    inflation_radius_ = declare_parameter<double>(
      "inflation_radius", 3.0);

    // 代价低于此值时直接视为自由空间。
    minimum_cost_ = declare_parameter<int>(
      "minimum_cost", 1);

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
      "robot_radius=%.2f inflation_radius=%.2f",
      input_topic_.c_str(),
      output_topic_.c_str(),
      robot_radius_,
      inflation_radius_);
  }

private:
  struct QueueCell
  {
    float distance;
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

  // [功能与联系] 历史实现中按占用阈值/未知策略识别膨胀种子；此备份文件不参与当前CMake编译。
  bool isObstacle(const int8_t occupancy) const
  {
    return occupancy >= occupied_threshold_ ||
      (unknown_is_obstacle_ && occupancy < 0);
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
    const float resolution =
      static_cast<float>(obstacle_map.info.resolution);

    const float max_distance = static_cast<float>(
      std::max(0.0, robot_radius_ + inflation_radius_));

    const float infinity = std::numeric_limits<float>::infinity();

    std::vector<float> distance_map(
      obstacle_map.data.size(), infinity);

    std::priority_queue<
      QueueCell,
      std::vector<QueueCell>,
      CompareQueueCell> queue;

    int obstacle_count = 0;

    // 所有占据栅格作为距离场的起点。
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = indexOf(
          x, y, obstacle_map.info.width);

        if (isObstacle(obstacle_map.data[index])) {
          distance_map[index] = 0.0F;
          queue.push({0.0F, x, y});
          ++obstacle_count;

          // 输出中保留致命障碍物。
          costmap.data[index] = 100;
        }
      }
    }

    if (obstacle_count == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "No occupied cells found in %s; publish an all-free costmap",
        input_topic_.c_str());
      return;
    }

    // 8 邻域 Dijkstra 距离变换。
    // 对 300 x 300、0.1m 的局部图计算量很小。
    constexpr int kNeighborCount = 8;
    const int offset_x[kNeighborCount] = {
      -1, 0, 1, -1, 1, -1, 0, 1
    };
    const int offset_y[kNeighborCount] = {
      -1, -1, -1, 0, 0, 1, 1, 1
    };

    while (!queue.empty()) {
      const QueueCell current = queue.top();
      queue.pop();

      const std::size_t current_index = indexOf(
        current.x, current.y, obstacle_map.info.width);

      if (current.distance > distance_map[current_index]) {
        continue;
      }

      if (current.distance > max_distance) {
        continue;
      }

      for (int i = 0; i < kNeighborCount; ++i) {
        const int next_x = current.x + offset_x[i];
        const int next_y = current.y + offset_y[i];

        if (next_x < 0 || next_y < 0 ||
            next_x >= width || next_y >= height) {
          continue;
        }

        const float step_distance =
          ((offset_x[i] == 0 || offset_y[i] == 0) ? 1.0F : 1.41421356F) *
          resolution;

        const float next_distance =
          current.distance + step_distance;

        if (next_distance > max_distance) {
          continue;
        }

        const std::size_t next_index = indexOf(
          next_x, next_y, obstacle_map.info.width);

        if (next_distance < distance_map[next_index]) {
          distance_map[next_index] = next_distance;
          queue.push({next_distance, next_x, next_y});
        }
      }
    }

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = indexOf(
          x, y, obstacle_map.info.width);

        if (isObstacle(obstacle_map.data[index])) {
          costmap.data[index] = 100;
          continue;
        }

        const float distance = distance_map[index];

        if (!std::isfinite(distance) || distance > max_distance) {
          costmap.data[index] = 0;
          continue;
        }

        // 船体半径范围内标记为高代价区。
        if (distance <= robot_radius_) {
          costmap.data[index] = 99;
          continue;
        }

        // 在 robot_radius 到 inflation_radius 外沿之间指数衰减。
        const float inflation_distance =
          static_cast<float>(inflation_radius_);

        if (inflation_distance <= 0.0F) {
          costmap.data[index] = 0;
          continue;
        }

        const float distance_from_robot =
          distance - static_cast<float>(robot_radius_);

        const float normalized =
          std::clamp(distance_from_robot / inflation_distance, 0.0F, 1.0F);

        // normalized=0 时约 98；normalized=1 时约 minimum_cost。
        const float cost_float =
          static_cast<float>(minimum_cost_) +
          (98.0F - static_cast<float>(minimum_cost_)) *
          std::exp(-3.0F * normalized);

        int cost = static_cast<int>(std::round(cost_float));
        cost = std::clamp(cost, minimum_cost_, 98);

        costmap.data[index] = static_cast<int8_t>(cost);
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  int occupied_threshold_{50};
  bool unknown_is_obstacle_{true};
  double robot_radius_{0.6};
  double inflation_radius_{3.0};
  int minimum_cost_{1};

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
