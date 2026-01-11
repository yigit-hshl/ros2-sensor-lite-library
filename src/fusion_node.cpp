#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_fusion_lite/fusion_core.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include <memory>

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
    // Declare and get parameters.
    // fusion_rate_hz: The frequency at which the fused state is published.
    this->declare_parameter<double>("fusion_rate_hz", 30.0);
    double rate = this->get_parameter("fusion_rate_hz").as_double();

    // ---------------- Fusion Engine Initialization ----------------
    // Instantiate the core fusion logic.
    fusion_core_ = std::make_unique<FusionCore>();
    fusion_core_->initialize();

    // Connect the library's logging to ROS 2 logging.
    fusion_core_->set_diagnostic_callback([this](const std::string &msg) {
      RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    });

    // ---------------- Subscriptions ----------------
    // IMU Subscription: Expects standard sensor_msgs/Imu
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10,
        std::bind(&FusionCoreNode::imu_callback, this, std::placeholders::_1));

    // Odometry Subscription: Expects nav_msgs/Odometry (e.g., from wheel
    // encoders)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&FusionCoreNode::odom_callback, this, std::placeholders::_1));

    // GPS/Pose Subscription: Expects geometry_msgs/PoseStamped (e.g., from
    // external localization)
    gps_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "gps_pose", 10,
        std::bind(&FusionCoreNode::gps_callback, this, std::placeholders::_1));

    // ---------------- Publishers ----------------
    // Publish the fused result as a PoseStamped message.
    fused_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "fused_pose", 10);

    // ---------------- Timers ----------------
    // Main loop timer to drive the publication rate.
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&FusionCoreNode::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "FusionCoreNode initialized.");
  }

private:
  // ---------------- Callbacks ----------------

  /**
   * @brief Handle incoming IMU messages.
   * Extracts accel/gyro and passes to FusionCore.
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Measurement meas;
    meas.type = MeasurementType::IMU;
    meas.timestamp = std::chrono::steady_clock::now();

    ImuMeasurement imu{};
    imu.linear_accel = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z};
    imu.angular_vel = {msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z};
    imu.timestamp = meas.timestamp;

    meas.imu = imu;
    fusion_core_->add_measurement(meas);
  }

  /**
   * @brief Handle incoming Odometry messages.
   * Extracts position/velocity and passes to FusionCore.
   */
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
    odom.timestamp = meas.timestamp;

    meas.odom = odom;
    fusion_core_->add_measurement(meas);
  }

  /**
   * @brief Handle incoming GPS/Pose messages.
   * Extracts position and passes to FusionCore.
   */
  void gps_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Measurement meas;
    meas.type = MeasurementType::GPS;
    meas.timestamp = std::chrono::steady_clock::now();

    GpsMeasurement gps{};
    gps.position = {msg->pose.position.x, msg->pose.position.y,
                    msg->pose.position.z};
    gps.timestamp = meas.timestamp;

    meas.gps = gps;
    fusion_core_->add_measurement(meas);
  }

  /**
   * @brief Periodic callback to publish the current state.
   */
  void publish_state() {
    // Retrieve latest state from the library (thread-safe)
    auto state = fusion_core_->get_state();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map"; // Fixed frame for now

    // Populate Position
    msg.pose.position.x = state.position[0];
    msg.pose.position.y = state.position[1];
    msg.pose.position.z = state.position[2];

    // Populate Orientation
    msg.pose.orientation.x = state.orientation[0];
    msg.pose.orientation.y = state.orientation[1];
    msg.pose.orientation.z = state.orientation[2];
    msg.pose.orientation.w = state.orientation[3];

    fused_pub_->publish(msg);
  }

  // ---------------- Members ----------------
  std::unique_ptr<FusionCore>
      fusion_core_; ///< The core fusion logic library instance

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pub_;
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
