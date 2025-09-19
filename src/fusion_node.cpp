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
    
  }

private:
  // ----- Callbacks from ROS topics -----

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