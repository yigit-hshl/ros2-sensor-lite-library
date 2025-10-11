#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_fusion_lite/fusion_core.hpp"

#include <memory>

using namespace std::chrono_literals;

namespace sensor_fusion_lite
{

  class FusionCoreNode : public rclcpp::Node
  {
  public:
    FusionCoreNode() : Node("fusion_core_node")
    {
      // Parameters
      this->declare_parameter<double>("fusion_rate_hz", 30.0);
      double rate = this->get_parameter("fusion_rate_hz").as_double();

      // Initialize FusionCore
      fusion_core_ = std::make_unique<FusionCore>();
      fusion_core_->initialize();
      fusion_core_->set_diagnostic_callback(
          [this](const std::string &msg)
          { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); });

      // Subscriptions
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", 10,
          std::bind(&FusionCoreNode::imu_callback, this, std::placeholders::_1));

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10,
          std::bind(&FusionCoreNode::odom_callback, this, std::placeholders::_1));

      gps_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "gps_pose", 10,
          std::bind(&FusionCoreNode::gps_callback, this, std::placeholders::_1));

      // Publisher
      fused_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("fused_pose", 10);

      // Timer for publishing fused state
      timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / rate),
          std::bind(&FusionCoreNode::publish_state, this));

      RCLCPP_INFO(this->get_logger(), "FusionCoreNode initialized.");
    }

  private:
    // ---------------- Callbacks ----------------

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      Measurement meas;
      meas.type = MeasurementType::IMU;
      meas.timestamp = std::chrono::steady_clock::now();

      ImuMeasurement imu{};
      imu.linear_accel = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
      imu.angular_vel = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
      imu.timestamp = meas.timestamp;

      meas.imu = imu;
      fusion_core_->add_measurement(meas);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      Measurement meas;
      meas.type = MeasurementType::ODOM;
      meas.timestamp = std::chrono::steady_clock::now();

      OdomMeasurement odom{};
      odom.position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
      odom.linear_velocity = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
      odom.timestamp = meas.timestamp;

      meas.odom = odom;
      fusion_core_->add_measurement(meas);
    }

    void gps_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      Measurement meas;
      meas.type = MeasurementType::GPS;
      meas.timestamp = std::chrono::steady_clock::now();

      GpsMeasurement gps{};
      gps.position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      gps.timestamp = meas.timestamp;

      meas.gps = gps;
      fusion_core_->add_measurement(meas);
    }

    void publish_state()
    {
      auto state = fusion_core_->get_state();

      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";
      msg.pose.position.x = state.position[0];
      msg.pose.position.y = state.position[1];
      msg.pose.position.z = state.position[2];
      msg.pose.orientation.x = state.orientation[0];
      msg.pose.orientation.y = state.orientation[1];
      msg.pose.orientation.z = state.orientation[2];
      msg.pose.orientation.w = state.orientation[3];

      fused_pub_->publish(msg);
    }

    // ---------------- Members ----------------
    std::unique_ptr<FusionCore> fusion_core_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

} // namespace sensor_fusion_lite

// ---------------- main() ----------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_fusion_lite::FusionCoreNode>());
  rclcpp::shutdown();
  return 0;
}
