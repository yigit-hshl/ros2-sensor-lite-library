# ros2-sensor-lite-library

A lightweight, modular library for combining multiple sensors (IMU, odometry, GPS, LIDAR pose, etc.) into a consistent state estimate without forcing users into heavy configs.

1. Pluggable Filters

Complementary filter → simple IMU fusion (gyro + accel).
1D/2D/3D Kalman filter (basic EKF/UKF) → extendable templates.
Custom filter hook → allow user to drop in their own equations.

2. ROS2 Node Interfaces
Subscribes to:
sensor_msgs/Imu
nav_msgs/Odometry
sensor_msgs/NavSatFix
geometry_msgs/PoseStamped

Publishes:
nav_msgs/Odometry (fused state)
geometry_msgs/PoseWithCovarianceStamped

3. Dynamic Configuration
Allow runtime adjustment of noise covariances (similar to dynamic_reconfigure in ROS1).
Example: change IMU trust level on the fly if it starts drifting.

4. Ease of Use
YAML config file with:
which sensors to fuse
noise parameters
update rates
Predefined templates (IMU+Odometry, IMU+GPS, IMU+LIDAR).

5. Extras
Diagnostics (diagnostic_msgs/DiagnosticArray) → show which sensor is active / dropped.
Visualization plugin for RViz → fused vs. raw trajectories.

## High-Level Node Architecture

 Node: fusion_node

This is the main entrypoint ROS2 node.

Inputs (Subscribed Topics)

IMU data → /imu/data (sensor_msgs/Imu)

Odometry → /wheel/odom (nav_msgs/Odometry)

GPS (optional) → /gps/fix (sensor_msgs/NavSatFix)

Pose estimates (optional) → /lidar/pose (geometry_msgs/PoseStamped)

Outputs (Published Topics)

Fused state → /fusion/odom (nav_msgs/Odometry)

Pose estimate with covariance → /fusion/pose (geometry_msgs/PoseWithCovarianceStamped)

Diagnostics → /fusion/diagnostics (diagnostic_msgs/DiagnosticArray)

Parameters (YAML Configurable)

use_imu (bool, default: true)

use_odom (bool, default: true)

use_gps (bool, default: false)

use_pose (bool, default: false)

filter_type (string: complementary, ekf, ukf)

update_rate (Hz, default: 50)

Noise covariance settings (per sensor):

imu_noise_cov (3x3)

odom_noise_cov (6x6)

gps_noise_cov (3x3)

pose_noise_cov (6x6)

diagnostics_enabled (bool, default: true)

