#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_fusion_lite/fusion_core.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace sensor_fusion_lite {

/**
 * @brief ROS 2 Node for the Sensor Fusion Library.
 *
 * This node bridges the ROS 2 middleware with the agnositc FusionCore library.
 * It subscribes to sensor topics (IMU, Odom, GPS) and publishes the fused
 * state.
 */
class FusionCoreNode : public rclcpp::Node {
public:
  FusionCoreNode() : Node("fusion_core_node") {
    // ---------------- Parameters ----------------
    this->declare_parameter<double>("fusion_rate_hz", 30.0);
    this->declare_parameter<std::string>("filter_type", "complementary");
    this->declare_parameter<int>("state_dim", 6);
    // Complementary params
    this->declare_parameter<double>("complementary.alpha", 0.98);
    // EKF params
    this->declare_parameter<std::vector<double>>("ekf.initial_process_noise",
                                                 std::vector<double>{});

    double rate = this->get_parameter("fusion_rate_hz").as_double();

    // ---------------- Fusion Config ----------------
    FusionConfig config;
    std::string type_str = this->get_parameter("filter_type").as_string();
    if (type_str == "ekf")
      config.filter_type = FilterType::EKF;
    else if (type_str == "ukf")
      config.filter_type = FilterType::UKF;
    else
      config.filter_type = FilterType::COMPLEMENTARY;

    config.state_dim = this->get_parameter("state_dim").as_int();
    config.complementary_config.alpha =
        this->get_parameter("complementary.alpha").as_double();
    config.ekf_config.initial_process_noise =
        this->get_parameter("ekf.initial_process_noise").as_double_array();

    // ---------------- Fusion Engine Initialization ----------------
    fusion_core_ = std::make_unique<FusionCore>();
    fusion_core_->initialize(config);

    // Connect logging
    fusion_core_->set_diagnostic_callback([this](const std::string &msg) {
      RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    });

    // ---------------- Publishers ----------------
    fused_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "fused_pose", 10);
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom_fused", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---------------- Subscriptions ----------------
    // Ensure QoS matches sensor data if needed (BEST_EFFORT vs RELIABLE)
    // Using default sensor data QoS for best effort (often used for high freq
    // IMU)
    auto qos = rclcpp::SensorDataQoS();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", qos,
        std::bind(&FusionCoreNode::imu_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos,
        std::bind(&FusionCoreNode::odom_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "gps_pose", 10,
        std::bind(&FusionCoreNode::gps_callback, this, std::placeholders::_1));

    // ---------------- Timers ----------------
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&FusionCoreNode::publish_state, this));

    RCLCPP_INFO(this->get_logger(),
                "FusionCoreNode initialized with filter: %s", type_str.c_str());
  }

private:
  // ---------------- Callbacks ----------------

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Measurement meas;
    meas.type = MeasurementType::IMU;
    meas.timestamp =
        std::chrono::steady_clock::now(); // Ideally convert ROS time to chrono

    ImuMeasurement imu{};
    imu.linear_accel = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z};
    imu.angular_vel = {msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z};
    imu.orientation = {msg->orientation.x, msg->orientation.y,
                       msg->orientation.z, msg->orientation.w};
    imu.timestamp = meas.timestamp;

    meas.imu = imu;
    fusion_core_->add_measurement(meas);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Measurement meas;
    meas.type = MeasurementType::ODOM;
    meas.timestamp = std::chrono::steady_clock::now();

    OdomMeasurement odom{};
    odom.position = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                     msg->pose.pose.position.z};
    odom.linear_velocity = {msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z};
    // Extract covariance diagonal (simplified)
    for (int i = 0; i < 6; ++i)
      odom.cov_diag[i] = msg->pose.covariance[i * 7]; // 0, 7, 14...
    odom.timestamp = meas.timestamp;

    meas.odom = odom;
    fusion_core_->add_measurement(meas);
  }

  void gps_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Measurement meas;
    meas.type = MeasurementType::GPS;
    meas.timestamp = std::chrono::steady_clock::now();

    GpsMeasurement gps{};
    gps.position = {msg->pose.position.x, msg->pose.position.y,
                    msg->pose.position.z};
    // TODO: Covariance from message if available (PoseStamped doesn't have it)
    gps.timestamp = meas.timestamp;

    meas.gps = gps;
    fusion_core_->add_measurement(meas);
  }

  void publish_state() {
    auto state = fusion_core_->get_state();
    auto now = this->get_clock()->now();

    // 1. Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "map"; // Global frame

    pose_msg.pose.position.x = state.position[0];
    pose_msg.pose.position.y = state.position[1];
    pose_msg.pose.position.z = state.position[2];
    pose_msg.pose.orientation.x = state.orientation[0];
    pose_msg.pose.orientation.y = state.orientation[1];
    pose_msg.pose.orientation.z = state.orientation[2];
    pose_msg.pose.orientation.w = state.orientation[3];

    fused_pub_->publish(pose_msg);

    // 2. Publish Odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link"; // Robot frame

    odom_msg.pose.pose = pose_msg.pose;
    // Velocity
    odom_msg.twist.twist.linear.x = state.velocity[0];
    odom_msg.twist.twist.linear.y = state.velocity[1];
    odom_msg.twist.twist.linear.z = state.velocity[2];

    odom_pub_->publish(odom_msg);

    // 3. Broadcast TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = state.position[0];
    t.transform.translation.y = state.position[1];
    t.transform.translation.z = state.position[2];
    t.transform.rotation = pose_msg.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  // ---------------- Members ----------------
  std::unique_ptr<FusionCore> fusion_core_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sensor_fusion_lite

// ---------------- main() ----------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_fusion_lite::FusionCoreNode>());
  rclcpp::shutdown();
  return 0;
}
