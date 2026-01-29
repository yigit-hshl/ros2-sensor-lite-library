# ros2-sensor-lite-library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble%2B-blue.svg)](https://docs.ros.org/en/humble/)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

> [!WARNING]
> This project is still under construction and lacks a lot of functionalities and it might not work as expected. Use with caution.

A lightweight, purely C++17 modular sensor fusion library for ROS 2. Designed for ease of use, it provides plug-and-play fusion of IMU, Odometry, and GPS data using swappable filter backends (Complementary, EKF, UKF).

## 🚀 Key Features

*   **Modular Backends**: Switch between `Complementary`, `EKF`, or `UKF` filters via a single parameter.
*   **ROS 2 Standard**: Publishes `nav_msgs/Odometry` and broadcasts `tf2` transforms (`odom` -> `base_link`) out of the box.
*   **No Heavy Configs**: Simple, flat parameter structure. No complex YAML trees required to get started.
*   **Sensor Agnostic**: Accepts standard ROS messages:
    *   `sensor_msgs/Imu`
    *   `nav_msgs/Odometry`
    *   `geometry_msgs/PoseStamped`

## 📦 Installation

This is a standard ROS 2 package. Clone it into your workspace:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/yigit-hshl/ros2-sensor-lite-library.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 🏃 Usage

### Running the Node

You can run the node directly with default settings (Complementary Filter):

```bash
ros2 run sensor_fusion_lite fusion_node
```

### Using Launch Files

A launch file is provided to easily configure parameters:

```bash
ros2 launch sensor_fusion_lite fusion.launch.py filter_type:=ekf fusion_rate_hz:=50.0
```

## ⚙️ Configuration

The node is highly configurable via ROS parameters.

### General Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `fusion_rate_hz` | `double` | `30.0` | Output frequency of fused state. |
| `filter_type` | `string` | `complementary` | Filter strategy: `complementary`, `ekf`, `ukf`. |
| `state_dim` | `int` | `6` | Dimension of the state vector (usually 6: x,y,z,vx,vy,vz). |

### Complementary Filter Stats

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `complementary.alpha` | `double` | `0.98` | Trust factor. Higher = trust prediction/old state, Lower = trust measurement. |

### EKF / UKF Stats

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `ekf.initial_process_noise` | `double[]` | `[]` | Diagonal elements of process noise matrix Q. |

## 📡 Topics

### Subscribed
*   `/imu/data` (`sensor_msgs/msg/Imu`)
*   `/odom` (`nav_msgs/msg/Odometry`)
*   `/gps_pose` (`geometry_msgs/msg/PoseStamped`)

### Published
*   `/fused_pose` (`geometry_msgs/msg/PoseStamped`): The fused position and orientation.
*   `/odom_fused` (`nav_msgs/msg/Odometry`): Standard odometry message with velocity.
*   **TF**: Broadcasts `odom` -> `base_link`.

## 📚 Documentation

Detailed architecture and API documentation is available:
*   [Architecture Design](docs/architecture.md)
*   [Theoretical Background & Math](docs/theory.md)
*   Generate Doxygen: `doxygen docs/Doxyfile`

## 🤝 Contributing

1.  Fork the Project
2.  Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3.  Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4.  Push to the Branch (`git push origin feature/AmazingFeature`)
5.  Open a Pull Request

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.
