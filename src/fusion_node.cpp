#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "sensor_fusion_lite/fusion_core.hpp"

using namespace std::chrono_literals;
using sensor_fusion_lite::FusionCore;
using sensor_fusion_lite::FilterType;
using sensor_fusion_lite::State;

class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node"), fusion_core_(FilterType::COMPLEMENTARY, 6, nullptr) {
    // Parameters
    this->declare_parameter<std::string>("filter_type", "complementary");

    // Create publishers
    fused_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fused_state, 10");

    // Create subscribers
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", 50, std::bind(&FusionNode::imu_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 50, std::bind(&FusionNode::odom_callback, this, std::placeholders::_1));
    gps_sub = this->create_subscription<geometry_msgs::msg::Point>("gps", 10, std::bind(&FusionNode::gps_callback, this, std::placeholders::_1));
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10, std::bind(&FusionNode::pose_callback, this, std::placeholders::_1));

    // Diagnostics
    fusion_core_.set_diagnostic_callback([this](const std::string& msg) {
      RCLCPP_INFO(this->get_logger(), "[FusionCore %s]", msg.c_str());
    });

    // Register state callback
    fusion_core_.register_state_callback([this](const State& st) {this->publish_state(st);});

    fusion_core_.initialize();
    fusion_core_.start();

    RCLCPP_INFO(this->get_logger(), "FusionNode started.");
  }

private:
  // ----- Callbacks from ROS topics -----
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sensor_fusion_lite::ImuMeasurement meas;
    meas.linear_accel = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    meas.angular_vel = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    meas.timestamp = std::chrono::steady_clock::now();

    fusion_core_.update_imu(meas);
  }

  

  // ----- Publishing fused state -----

  // ----- Member -----
};

// ----- Main -----
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}