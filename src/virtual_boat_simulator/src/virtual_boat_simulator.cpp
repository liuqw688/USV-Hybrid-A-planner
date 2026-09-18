#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 引入自定义消息
#include "virtual_boat_simulator/msg/boat_status.hpp" 

using namespace std::chrono_literals;

class VirtualBoatSimulator : public rclcpp::Node
{
public:
    // [功能与联系] 加载目标船身份、位置、速度与显示尺寸，订阅独立指令，启动更新循环并发布目标odom/状态/TF。
    VirtualBoatSimulator() : Node("virtual_boat_simulator")
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("vessel_name", "Virtual-Vessel-01");
        this->declare_parameter<std::string>("vessel_type", "VIRTUAL_SIM");
        this->declare_parameter<double>("length", 1.0);
        this->declare_parameter<double>("width", 1.5);
        this->declare_parameter<double>("initial_speed", 2.0);
        this->declare_parameter<double>("update_rate", 20.0);
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_yaw", 0.0);
        this->declare_parameter<std::string>("command_topic", "/target_boat/command");
        this->declare_parameter<std::string>("odom_topic", "/target_boat/odom");
        this->declare_parameter<std::string>("status_topic", "/target_boat/status");
        this->declare_parameter<std::string>("markers_topic", "/target_boat/markers");
        this->declare_parameter<std::string>("base_frame", "target_boat/base_link");

        vessel_name_ = this->get_parameter("vessel_name").as_string();
        vessel_type_ = this->get_parameter("vessel_type").as_string();
        length_ = this->get_parameter("length").as_double();
        width_ = this->get_parameter("width").as_double();
        double initial_speed = this->get_parameter("initial_speed").as_double();
        double update_rate = this->get_parameter("update_rate").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        
        dt_ = 1.0 / update_rate;

        // 初始状态设定
        x_ = this->get_parameter("initial_x").as_double();
        y_ = this->get_parameter("initial_y").as_double();
        yaw_ = this->get_parameter("initial_yaw").as_double();
        v_ = initial_speed; // 赋予初始速度 2m/s
        w_ = 0.0;
        cmd_v_ = initial_speed;
        cmd_w_ = 0.0;

        // 订阅控制指令和 RViz 初始位姿
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("command_topic").as_string(), 10,
            std::bind(&VirtualBoatSimulator::cmdCallback, this, std::placeholders::_1));

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&VirtualBoatSimulator::initialPoseCallback, this, std::placeholders::_1));

        // 发布各类话题
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 10);
        status_pub_ = this->create_publisher<virtual_boat_simulator::msg::BoatStatus>(
            this->get_parameter("status_topic").as_string(), 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/boat/imu", 10);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/boat/gps", 10);
        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->get_parameter("markers_topic").as_string(), 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 启动仿真定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&VirtualBoatSimulator::updateLoop, this));

        RCLCPP_INFO(this->get_logger(), "Target boat started: %s at [%.1f, %.1f, %.2f], speed %.2f m/s", 
                    vessel_name_.c_str(), x_, y_, yaw_, initial_speed);
    }

private:
    // 接收速度指令
  // [功能与联系] 缓存目标船独立Twist供updateLoop直接积分；不修改本船命令，也没有本船惯性模型。
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_v_ = msg->linear.x;
        cmd_w_ = msg->angular.z;
    }

    // 接收 RViz 2D Pose Estimate 重置位置
    // [功能与联系] 接收RViz初始位姿重设虚拟船位置航向；当前不做TF转换，需使用与odom一致的数值坐标。
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw_ = yaw;

        RCLCPP_INFO(this->get_logger(), "Virtual Boat Pose Reset via RViz: [%.2f, %.2f, %.2f rad]", x_, y_, yaw_);
    }

    // 主仿真循环
    // [功能与联系] 将虚拟船cmd_v/cmd_w直接作为实际速率积分，随后发布TF、odom、状态和显示；没有本船惯性模型。
    void updateLoop()
    {
        // 1. 运动学积分 (简单一阶模型)
        v_ = cmd_v_; 
        w_ = cmd_w_;

        x_ += v_ * std::cos(yaw_) * dt_;
        y_ += v_ * std::sin(yaw_) * dt_;
        yaw_ += w_ * dt_;

        // 角度归一化
        while (yaw_ > M_PI) yaw_ -= 2 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2 * M_PI;

        // 2. 发布数据
        publishTf();
        publishOdometry();
        publishStatus();
        publishImu();
        publishGps();
        publishMarkers();
    }

    // [功能与联系] 广播odom到目标船专用base_frame，避免与本船base_link冲突。
    void publishTf()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = base_frame_;

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }

    // [功能与联系] 发布目标运动状态供Hybrid/DWA进行匀速预测；船体显示长宽不决定规划半径。
    void publishOdometry()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = v_;
        odom.twist.twist.angular.z = w_;

        odom_pub_->publish(odom);
    }

  // [功能与联系] 发布目标船身份、显示尺寸、速度和COG诊断；规划预测主要使用目标Odometry。
    void publishStatus()
    {
        virtual_boat_simulator::msg::BoatStatus status;
        status.header.stamp = this->now();
        status.header.frame_id = base_frame_;
        status.vessel_name = vessel_name_;
        status.vessel_type = vessel_type_;
        status.length = length_;
        status.width = width_;
        status.speed_over_ground = std::abs(v_);
        
        // 数学角度转航海角度 (0-360度，北偏角)
        double cog_rad = M_PI_2 - yaw_; 
        while (cog_rad < 0) cog_rad += 2 * M_PI;
        while (cog_rad >= 2 * M_PI) cog_rad -= 2 * M_PI;
        status.course_over_ground = cog_rad * 180.0 / M_PI;

        status_pub_->publish(status);
    }

    // [功能与联系] 发布简单姿态/角速度IMU样本；不包含真实传感噪声或完整动力学。
    void publishImu()
    {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = this->now();
        imu.header.frame_id = base_frame_;
        imu.angular_velocity.z = w_;
        imu.linear_acceleration.z = 9.81; // 模拟重力
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        imu.orientation = tf2::toMsg(q);
        imu_pub_->publish(imu);
    }

    // [功能与联系] 将平面坐标近似转换为GPS演示数据，假设赤道原点；不能用作真实河图地理定位。
    void publishGps()
    {
        sensor_msgs::msg::NavSatFix gps;
        gps.header.stamp = this->now();
        gps.header.frame_id = base_frame_;
        gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        
        // 简单平面转球面模拟 (假设原点在赤道)
        gps.latitude = y_ / 111320.0;
        gps.longitude = x_ / (111320.0 * std::cos(gps.latitude * M_PI / 180.0));
        gps.altitude = 0.0;
        gps_pub_->publish(gps);
    }

  // [功能与联系] 发布目标船船体、速度箭头及名称；显示长宽不改变规划配置的目标碰撞半径。
    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray markers;

        // 1. 船体模型 (蓝色半透明立方体)
        visualization_msgs::msg::Marker hull;
        hull.header.frame_id = base_frame_;
        hull.header.stamp = this->now();
        hull.ns = "virtual_boat_hull";
        hull.id = 0;
        hull.type = visualization_msgs::msg::Marker::CUBE;
        hull.action = visualization_msgs::msg::Marker::ADD;
        hull.pose.position.z = 0.5;
        hull.pose.orientation.w = 1.0;
        hull.scale.x = length_;
        hull.scale.y = width_;
        hull.scale.z = 1.0;
        hull.color.a = 0.7; hull.color.r = 0.0; hull.color.g = 0.4; hull.color.b = 1.0;
        markers.markers.push_back(hull);

        // 2. 速度矢量 (黄色箭头，长度随速度缩放)
        visualization_msgs::msg::Marker vel_arrow;
        vel_arrow.header.frame_id = base_frame_;
        vel_arrow.header.stamp = this->now();
        vel_arrow.ns = "virtual_boat_velocity";
        vel_arrow.id = 1;
        vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
        vel_arrow.action = visualization_msgs::msg::Marker::ADD;
        
        geometry_msgs::msg::Point start, end;
        start.x = 0.0; start.y = 0.0; start.z = 1.5;
        end.x = v_ * 2.0; end.y = 0.0; end.z = 1.5; 
        vel_arrow.points.push_back(start);
        vel_arrow.points.push_back(end);
        vel_arrow.scale.x = 0.3; vel_arrow.scale.y = 0.6; vel_arrow.scale.z = 0.0;
        vel_arrow.color.a = 1.0; vel_arrow.color.r = 1.0; vel_arrow.color.g = 1.0; vel_arrow.color.b = 0.0;
        markers.markers.push_back(vel_arrow);

        // 3. 船名文字标识
        visualization_msgs::msg::Marker text;
        text.header.frame_id = base_frame_;
        text.header.stamp = this->now();
        text.ns = "virtual_boat_name";
        text.id = 2;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.z = 2.5;
        text.pose.orientation.w = 1.0;
        text.scale.z = 1.2; 
        text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0;
        text.text = vessel_name_;
        markers.markers.push_back(text);

        markers_pub_->publish(markers);
    }

    // 状态变量
    double x_, y_, yaw_, v_, w_, cmd_v_, cmd_w_, dt_;
    std::string vessel_name_, vessel_type_, base_frame_;
    double length_, width_;

    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<virtual_boat_simulator::msg::BoatStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualBoatSimulator>());
    rclcpp::shutdown();
    return 0;
}
