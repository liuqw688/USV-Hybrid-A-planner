#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ChannelAStarNode : public rclcpp::Node
{
public:
  // [功能与联系] 读取全局航点/语义/安全参数，建立TF、输入回调及锁存输出；不创建规划timer，搜索由目标或航道角色变化触发。
  ChannelAStarNode()
  : Node("channel_astar_node")
  {
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    lane_topic_ = declare_parameter("lane_costmap_topic", "/channel/lane_costmap");
    goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    output_topic_ = declare_parameter("output_topic", "/astar_channel_waypoints");
    active_goal_topic_ = declare_parameter("active_goal_topic", "/goal_pose_from_astar");
    spacing_ = declare_parameter("waypoint_spacing", 100.0);
    switch_distance_ = declare_parameter("goal_switch_distance", 8.0);
    mandatory_distance_ = declare_parameter("mandatory_goal_distance", 1.5);
    via_points_ = declare_parameter<std::vector<double>>("via_points", std::vector<double>{});
    map_settle_time_ = declare_parameter("map_settle_time", 0.5);
    minimum_replan_period_ = declare_parameter("minimum_replan_period", 0.5);
    replan_on_lane_role_change_=declare_parameter("replan_on_lane_role_change",true);
    main_lane_value_max_ = declare_parameter("main_lane_value_max", 20);
    max_iterations_ = declare_parameter("max_search_iterations", 200000);
    center_cost_weight_ = declare_parameter("center_cost_weight", 0.08);
    right_channel_only_ = declare_parameter("right_channel_only", true);
    opposite_lane_value_=declare_parameter("opposite_lane_value",75);
    terminal_crossing_distance_=declare_parameter("terminal_crossing_distance",100.0);
    opposite_lane_penalty_=declare_parameter("opposite_lane_penalty",4.0);
    outside_lane_value_=declare_parameter("outside_lane_value",45);
    outside_lane_penalty_=declare_parameter("outside_lane_penalty",1.0);
    safety_threshold_=declare_parameter("safety_cost_threshold",88);
    const auto safety_topic=declare_parameter("safety_costmap_topic",std::string("/local_costmap"));
    if(!std::isfinite(terminal_crossing_distance_) || terminal_crossing_distance_<=0 ||
      !std::isfinite(opposite_lane_penalty_) || opposite_lane_penalty_<0 ||
      opposite_lane_value_<=main_lane_value_max_ || opposite_lane_value_>95 ||
      outside_lane_value_<=main_lane_value_max_ || outside_lane_value_>=opposite_lane_value_ ||
      !std::isfinite(outside_lane_penalty_) || outside_lane_penalty_<0 ||
      safety_threshold_<1 || safety_threshold_>100)
      throw std::invalid_argument("Invalid terminal crossing configuration");
    if(spacing_<=0 || switch_distance_<=0 || switch_distance_>=spacing_ || mandatory_distance_<=0 ||
      map_settle_time_<0 || minimum_replan_period_<0 || via_points_.size()%2!=0 ||
      std::any_of(via_points_.begin(),via_points_.end(),[](double v){return !std::isfinite(v);}))
      throw std::invalid_argument("Invalid spacing/tolerance/period or odd-length via_points");

    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_topic_, latched);
    active_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(active_goal_topic_, latched);
    lane_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      lane_topic_, latched, std::bind(&ChannelAStarNode::laneCallback, this, std::placeholders::_1));
    safety_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(safety_topic,latched,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        if(m->info.resolution<=0 || m->data.size()!=static_cast<std::size_t>(m->info.width)*m->info.height)
          return;
        safety_map_=m;tryPendingGoal();
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&ChannelAStarNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&ChannelAStarNode::odomCallback, this, std::placeholders::_1));
    replan_service_=create_service<std_srvs::srv::Trigger>("/channel_astar/replan",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success=has_final_goal_;
        response->message=has_final_goal_ ? "Replan queued for remaining mission destinations":"No goal received";
        if(!has_final_goal_) return;
        planned_=false;current_path_.poses.clear();has_last_active_goal_=false;
        nav_msgs::msg::Path empty;empty.header=final_goal_.header;empty.header.stamp=now();
        path_pub_->publish(empty);replan_pending_=true;last_input_change_=now();
      });
    RCLCPP_INFO(
      get_logger(), "Channel A* ready: final goal=%s, active Hybrid A* goal=%s",
      goal_topic_.c_str(), active_goal_topic_.c_str());
  }

private:
  struct QueueEntry
  {
    double f{0.0};
    double g{0.0};
    int id{-1};
    bool operator<(const QueueEntry & other) const {return f > other.f;}
  };

  // [功能与联系] 将二维栅格坐标映射为行优先下标；供安全查询、A*邻居扩展及父节点数组共用。
  int index(const int x, const int y) const
  {
    return y * static_cast<int>(lane_map_->info.width) + x;
  }

  // [功能与联系] 按航道图原点、分辨率和原点姿态转换世界坐标并检查边界；搜索前验证起点和目标。
  bool worldToGrid(const double world_x, const double world_y, int & x, int & y) const
  {
    if (!lane_map_ || lane_map_->info.resolution <= 0.0 ||
      !std::isfinite(world_x) || !std::isfinite(world_y)) {return false;}
    const double yaw = tf2::getYaw(lane_map_->info.origin.orientation);
    const double dx = world_x - lane_map_->info.origin.position.x;
    const double dy = world_y - lane_map_->info.origin.position.y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    x = static_cast<int>(std::floor(local_x / lane_map_->info.resolution));
    y = static_cast<int>(std::floor(local_y / lane_map_->info.resolution));
    return x >= 0 && y >= 0 && x < static_cast<int>(lane_map_->info.width) &&
           y < static_cast<int>(lane_map_->info.height);
  }

  // [功能与联系] 将航道格中心还原为世界坐标；用于把A*父链转换为供Hybrid接收的稀疏航点。
  void gridToWorld(const int x, const int y, double & world_x, double & world_y) const
  {
    const double yaw = tf2::getYaw(lane_map_->info.origin.orientation);
    const double local_x = (static_cast<double>(x) + 0.5) * lane_map_->info.resolution;
    const double local_y = (static_cast<double>(y) + 0.5) * lane_map_->info.resolution;
    world_x = lane_map_->info.origin.position.x +
      std::cos(yaw) * local_x - std::sin(yaw) * local_y;
    world_y = lane_map_->info.origin.position.y +
      std::sin(yaw) * local_x + std::cos(yaw) * local_y;
  }

  // [功能与联系] 综合区域类别、末段/起段连接半径与航道外安全阈值筛选格子；searchCells同时用它检查斜向切角。
  bool traversable(const int x, const int y, const int start_id, const int goal_id) const
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(lane_map_->info.width) ||
      y >= static_cast<int>(lane_map_->info.height))
    {
      return false;
    }
    const int id = index(x, y);
    if (id == start_id || id == goal_id) {return true;}
    const int value = lane_map_->data[static_cast<std::size_t>(id)];
    if(value==outside_lane_value_ && !outsideSafe(x,y)) return false;
    if(!right_channel_only_) return value>=0 && value<=95;
    if(value>=0 && value<=main_lane_value_max_) return true;
    // 只对左航道最终目标开放末段连接，不能为抄近路全程借用对向航道。
    if(value!=opposite_lane_value_ && value!=outside_lane_value_) return false;
    const auto width=static_cast<int>(lane_map_->info.width);
    return (terminal_crossing_ && std::hypot(x-goal_id%width,
        y-goal_id/width)*lane_map_->info.resolution<=terminal_crossing_distance_) ||
      (departure_crossing_ && std::hypot(x-start_id%width,y-start_id/width)*lane_map_->info.resolution
        <=terminal_crossing_distance_);
  }

  // [功能与联系] 查询对齐的物理安全图，拒绝航道外未知或高代价格；仅是中心栅格筛选，最终曲线由Hybrid验证。
  bool outsideSafe(int x,int y) const {
    // 航道语义45只表示航道外，不能当作物理可通行证明；叠加真实岸边安全图。
    const int cost=safety_map_->data[static_cast<std::size_t>(index(x,y))];
    return cost>=0 && cost<safety_threshold_;
  }

  // [功能与联系] 缓存航道输入并识别主/对向角色互换；纠偏清空旧路线、保留已完成途经点，随后尝试待处理任务。
  void laneCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if(!std::isfinite(message->info.resolution) || message->info.resolution<=0 ||
      message->info.width==0 || message->info.height==0 ||
      message->data.size()!=static_cast<std::size_t>(message->info.width)*message->info.height) {
      RCLCPP_ERROR(get_logger(),"Invalid lane map dimensions; ignoring update");return;
    }
    // 仅识别同网格主航道与对向航道互换；0..20内部中心梯度变化不触发。
    bool roles_changed=false;
    if(replan_on_lane_role_change_ && lane_map_ &&
      lane_map_->header.frame_id==message->header.frame_id &&
      lane_map_->info.width==message->info.width && lane_map_->info.height==message->info.height &&
      lane_map_->info.resolution==message->info.resolution && lane_map_->info.origin==message->info.origin) {
      for(std::size_t i=0;i<message->data.size();++i) {
        const int old=lane_map_->data[i],updated=message->data[i];
        const bool old_main=old>=0 && old<=main_lane_value_max_;
        const bool new_main=updated>=0 && updated<=main_lane_value_max_;
        if((old_main && updated==opposite_lane_value_) || (old==opposite_lane_value_ && new_main)) {
          roles_changed=true;break;
        }
      }
    }
    lane_map_ = message;
    if(roles_changed && has_final_goal_) {
      // 本任务纠偏不重置已完成途经点；立即撤销旧路线，避免沿错侧航道继续走。
      planned_=false;current_path_.poses.clear();has_last_active_goal_=false;
      nav_msgs::msg::Path empty;empty.header=message->header;empty.header.stamp=now();
      path_pub_->publish(empty);replan_pending_=true;last_input_change_=now();
      RCLCPP_INFO(get_logger(),"Lane roles changed: queued one mission correction");
    }
    tryPendingGoal();
  }

  // [功能与联系] 每次RViz目标都启动新任务，撤销旧路线并重置途经点进度；输入就绪后执行一次全局A*。
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    // 每条目标消息均为一次显式任务请求；重复点击同一位置也重新规划一次。
    final_goal_ = *message;
    has_final_goal_ = true;
    planned_=false;current_path_.poses.clear();has_last_active_goal_=false;
    completed_destinations_=0;
    // 新任务先撤销旧路线。Hybrid的global_route_topic订阅用空Path停车，
    // 避免新目标无解时仍然跟踪旧任务的活动航点。
    nav_msgs::msg::Path empty;empty.header=message->header;empty.header.stamp=now();
    path_pub_->publish(empty);
    replan_pending_ = true;
    last_input_change_=now();
    tryPendingGoal();
  }

  // [功能与联系] 缓存本船位姿，按到达半径推进活动航点并尝试待处理任务；普通移动不重复已完成搜索。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    odom_ = message;
    switchWaypoint();
    tryPendingGoal();
  }

  // [功能与联系] 无timer的待任务入口：确认地图对齐、船位及稳定等待后调用plan；没有待任务时立即返回。
  void tryPendingGoal()
  {
    // 没有规划timer。输入回调只补齐已点击目标的首次规划，pending=false立即返回。
    if (!replan_pending_ || !lane_map_ || !safety_map_ || !odom_ || !has_final_goal_) {return;}
    if(safety_map_->header.frame_id!=lane_map_->header.frame_id ||
      safety_map_->info.width!=lane_map_->info.width || safety_map_->info.height!=lane_map_->info.height ||
      safety_map_->info.resolution!=lane_map_->info.resolution ||
      safety_map_->info.origin!=lane_map_->info.origin) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Waiting for aligned safety/lane maps");return;
    }
    if((now()-last_input_change_).seconds()<map_settle_time_) return;
    if (last_plan_time_.nanoseconds() != 0 &&
      (now() - last_plan_time_).seconds() < minimum_replan_period_)
    {
      return;
    }
    replan_pending_ = false;
    last_plan_time_ = now();
    plan();
  }

  // [功能与联系] 将目标通过TF变换到规划图，逐段经过剩余途经点搜索并按弧长采样；全部成功后一次发布完整Path和当前活动目标。
  void plan()
  {
    if(odom_->header.frame_id!=lane_map_->header.frame_id) {
      RCLCPP_ERROR(get_logger(),"Odom/map frame mismatch; transform odometry to %s",
        lane_map_->header.frame_id.c_str());return;
    }
    auto mission_goal=final_goal_;
    if(mission_goal.header.frame_id!=lane_map_->header.frame_id) {
      try {
        // RViz在map中设置目标，规划图在odom中：转换位置及姿态，不能只改frame_id。
        // 使用最新可用TF生成固定任务快照；保留原始目标用于重复任务判定。
        const auto transform=tf_buffer_->lookupTransform(lane_map_->header.frame_id,
          mission_goal.header.frame_id,tf2::TimePointZero);
        tf2::doTransform(final_goal_,mission_goal,transform);
      } catch(const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Waiting for goal TF: %s",error.what());
        // 静态TF可能晚于目标到达；等待后重试，而不是丢弃此次点击。
        replan_pending_=true;return;
      }
    }
    // 同一快照逐段搜索途经点，整条任务成功才一次发布，不能发布半条任务。
    std::vector<geometry_msgs::msg::PoseStamped> destinations;
    for(std::size_t i=completed_destinations_*2;i<via_points_.size();i+=2) {
      auto via=mission_goal;via.pose.position.x=via_points_[i];via.pose.position.y=via_points_[i+1];
      destinations.push_back(via);
    }
    destinations.push_back(mission_goal);
    nav_msgs::msg::Path path;path.header.frame_id=lane_map_->header.frame_id;path.header.stamp=now();
    std::vector<bool> mandatory;
    geometry_msgs::msg::PoseStamped start;
    start.header=path.header;start.pose=odom_->pose.pose;
    path.poses.push_back(start);mandatory.push_back(false);
    std::size_t leg_index=0;
    for(const auto &destination:destinations) {
      auto cells=searchCells(start.pose.position.x,start.pose.position.y,
        destination.pose.position.x,destination.pose.position.y,++leg_index==destinations.size());
      if(cells.empty()) return;
      double travelled=0,next_sample=spacing_;
      double px=start.pose.position.x,py=start.pose.position.y;
      for(std::size_t i=1;i<=cells.size();++i) {
        double x=destination.pose.position.x,y=destination.pose.position.y;
        if(i<cells.size()) gridToWorld(cells[i]%static_cast<int>(lane_map_->info.width),
          cells[i]/static_cast<int>(lane_map_->info.width),x,y);
        const double length=std::hypot(x-px,y-py);
        while(length>1e-9 && next_sample<travelled+length-1e-6) {
          const double ratio=(next_sample-travelled)/length;
          auto sample=destination;sample.header=path.header;
          sample.pose.position.x=px+ratio*(x-px);sample.pose.position.y=py+ratio*(y-py);
          path.poses.push_back(sample);mandatory.push_back(false);next_sample+=spacing_;
        }
        travelled+=length;px=x;py=y;
      }
      auto pinned=destination;pinned.header=path.header;
      // 途经点及最终目标始终精确保留，不能被100m降采样丢弃。
      path.poses.push_back(pinned);mandatory.push_back(true);start=pinned;
    }
    for(std::size_t i=0;i+1<path.poses.size();++i) {
      const auto &a=path.poses[i].pose.position;const auto &b=path.poses[i+1].pose.position;
      tf2::Quaternion q;q.setRPY(0,0,std::atan2(b.y-a.y,b.x-a.x));
      path.poses[i].pose.orientation=tf2::toMsg(q);
    }
    current_path_=path;mandatory_=mandatory;current_index_=path.poses.size()>1 ? 1:0;
    planned_=true;path_pub_->publish(path);publishActiveGoal(true);
    RCLCPP_INFO(get_logger(),"Published fixed mission route once: %zu waypoints, %zu required destinations",
      path.poses.size(),destinations.size());
  }

  // [功能与联系] 8邻域A*累计中心与区域代价，禁止斜穿障碍角；主体走主航道，起末段允许受限非主航道连接。
  std::vector<int> searchCells(double sx,double sy,double gx,double gy,bool final_leg)
  {
    int start_x = 0, start_y = 0, goal_x = 0, goal_y = 0;
    if (!worldToGrid(sx,sy, start_x, start_y)) {
      RCLCPP_WARN(get_logger(), "A* start is outside /channel/lane_costmap");
      return {};
    }
    if (!worldToGrid(gx,gy, goal_x, goal_y)) {
      RCLCPP_WARN(get_logger(), "A* final goal is outside /channel/lane_costmap");
      return {};
    }
    const int start_id = index(start_x, start_y);
    const int goal_id = index(goal_x, goal_y);
    const int goal_value = lane_map_->data[static_cast<std::size_t>(goal_id)];
    terminal_crossing_=final_leg && (goal_value==opposite_lane_value_ || goal_value==outside_lane_value_);
    if(goal_value==outside_lane_value_ && !outsideSafe(goal_x,goal_y)) {
      RCLCPP_WARN(get_logger(),"Outside goal is in dangerous/unknown safety costmap");return {};
    }
    if (right_channel_only_ && (goal_value < 0 ||
      (goal_value > main_lane_value_max_ && !terminal_crossing_))) {
      RCLCPP_WARN(
        get_logger(), "Final goal is not in the current right-hand lane (cell value=%d); "
        "waiting for corrected lane classification or a valid goal", goal_value);
      return {};
    }
    const int start_value=lane_map_->data[static_cast<std::size_t>(start_id)];
    departure_crossing_=start_value==opposite_lane_value_ || start_value==outside_lane_value_;
    if((start_value==outside_lane_value_ && !outsideSafe(start_x,start_y)) ||
      start_value<0 || (right_channel_only_ && start_value>main_lane_value_max_ && !departure_crossing_) ||
      (!right_channel_only_ && (start_value>95 || goal_value<0 || goal_value>95))) {
      RCLCPP_WARN(get_logger(),"Start/destination outside allowed lane; no unsafe endpoint exemption");
      return {};
    }

    const std::size_t cell_count = lane_map_->data.size();
    std::vector<int> parent(cell_count, -1);
    std::vector<double> best(cell_count, std::numeric_limits<double>::infinity());
    std::priority_queue<QueueEntry> open;
    best[static_cast<std::size_t>(start_id)] = 0.0;
    open.push({std::hypot(start_x - goal_x, start_y - goal_y), 0.0, start_id});
    constexpr int dx[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    constexpr int dy[8] = {1, 0, -1, 1, -1, 1, 0, -1};
    int iterations = 0;
    while (!open.empty() && iterations++ < max_iterations_) {
      const auto current = open.top();
      open.pop();
      if (current.g > best[static_cast<std::size_t>(current.id)] + 1.0e-9) {continue;}
      if (current.id == goal_id) {break;}
      const int x = current.id % static_cast<int>(lane_map_->info.width);
      const int y = current.id / static_cast<int>(lane_map_->info.width);
      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (!traversable(nx, ny, start_id, goal_id)) {continue;}
        if (dx[direction] != 0 && dy[direction] != 0 &&
          (!traversable(x + dx[direction], y, start_id, goal_id) ||
          !traversable(x, y + dy[direction], start_id, goal_id)))
        {
          continue;
        }
        const int next_id = index(nx, ny);
        const int lane_value = std::max(0, static_cast<int>(
          lane_map_->data[static_cast<std::size_t>(next_id)]));
        const double movement = std::hypot(dx[direction], dy[direction]);
        // 保留航道中心代价，额外惩罚对向航道行程，使主体留在右侧、末段才横穿。
        const double crossing_cost=right_channel_only_ && lane_value==opposite_lane_value_ ?
          opposite_lane_penalty_*movement:lane_value==outside_lane_value_ ? outside_lane_penalty_*movement:0.0;
        const double candidate = current.g + movement + center_cost_weight_ * lane_value * movement+
          crossing_cost;
        if (candidate >= best[static_cast<std::size_t>(next_id)]) {continue;}
        best[static_cast<std::size_t>(next_id)] = candidate;
        parent[static_cast<std::size_t>(next_id)] = current.id;
        open.push({candidate + std::hypot(nx - goal_x, ny - goal_y), candidate, next_id});
      }
    }
    if (goal_id != start_id && parent[static_cast<std::size_t>(goal_id)] < 0) {
      RCLCPP_WARN(
        get_logger(), "A* found no connected right-hand-lane route after %d iterations", iterations);
      return {};
    }

    std::vector<int> cells;
    for (int id = goal_id; id != start_id; id = parent[static_cast<std::size_t>(id)]) {
      if (id < 0) {return {};}
      cells.push_back(id);
    }
    cells.push_back(start_id);
    std::reverse(cells.begin(), cells.end());
    return cells;
  }


  // [功能与联系] 依据船位顺序推进普通8m/必经1.5m目标，并记录完成的途经点；最终点交给DWA终点停车。
  void switchWaypoint()
  {
    if (!odom_ || current_path_.poses.empty() || current_index_ >= current_path_.poses.size() ||
      odom_->header.frame_id!=current_path_.header.frame_id) {return;}
    std::size_t next_index = current_index_;
    while (next_index + 1U < current_path_.poses.size()) {
      const auto & waypoint = current_path_.poses[next_index];
      if (std::hypot(
          odom_->pose.pose.position.x - waypoint.pose.position.x,
          odom_->pose.pose.position.y - waypoint.pose.position.y) >=
          (mandatory_[next_index] ? mandatory_distance_:switch_distance_))
      {
        break;
      }
      if(mandatory_[next_index]) ++completed_destinations_;
      ++next_index;
    }
    if (next_index != current_index_) {
      current_index_ = next_index;
      publishActiveGoal(false);
    }
  }

  // [功能与联系] 发布当前一个PoseStamped给Hybrid；不能把整表同时当目标发送，否则只剩最后一个目标。
  void publishActiveGoal(const bool force)
  {
    if (current_path_.poses.empty() || current_index_ >= current_path_.poses.size()) {return;}
    const auto & goal = current_path_.poses[current_index_];
    if (!force && has_last_active_goal_ &&
      std::hypot(
        goal.pose.position.x - last_active_goal_.pose.position.x,
        goal.pose.position.y - last_active_goal_.pose.position.y) < 1.0e-3)
    {
      return;
    }
    active_goal_pub_->publish(goal);
    last_active_goal_ = goal;
    has_last_active_goal_ = true;
  }

  std::string lane_topic_, goal_topic_, odom_topic_, output_topic_, active_goal_topic_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double spacing_{100.0}, switch_distance_{8.0},mandatory_distance_{1.5},map_settle_time_{0.5};
  std::vector<double> via_points_;
  std::vector<bool> mandatory_;
  bool planned_{false};
  bool replan_on_lane_role_change_{true};
  bool terminal_crossing_{false};
  bool departure_crossing_{false};
  int opposite_lane_value_{75};
  int outside_lane_value_{45},safety_threshold_{88};
  double outside_lane_penalty_{1.0};
  double terminal_crossing_distance_{100.0},opposite_lane_penalty_{4.0};
  rclcpp::Time last_input_change_{0,0,RCL_ROS_TIME};
  double minimum_replan_period_{0.5}, center_cost_weight_{0.08};
  int main_lane_value_max_{20}, max_iterations_{200000};
  bool right_channel_only_{true}, has_final_goal_{false}, replan_pending_{false};
  bool has_last_active_goal_{false};
  std::size_t completed_destinations_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  std::size_t current_index_{0};
  geometry_msgs::msg::PoseStamped final_goal_, last_active_goal_;
  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr lane_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr safety_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr safety_sub_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr lane_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChannelAStarNode>());
  rclcpp::shutdown();
  return 0;
}
