# Architecture: Sensor Fusion Lite

## Overview

The **Sensor Fusion Lite** library provides a modular way to fuse sensor data (IMU, Odometry, GPS, Pose) into a consistent state estimate. It is designed to be lightweight and easily extensible with custom filters.

## Data Flow

```mermaid
graph LR
    subgraph Sensors
        IMU(/imu/data)
        Odom(/wheel/odom)
        GPS(/gps/fix)
        Pose(/lidar/pose)
    end

    subgraph FusionNode
        Sync[Synchronizer]
        Filter[Fusion Filter Interface]
        EKF[Extended Kalman Filter]
        UKF[Unscented Kalman Filter]
        Comp[Complementary Filter]
    end

    subgraph Output
        FusedOdom(/fusion/odom)
        FusedPose(/fusion/pose)
    end

    IMU --> Sync
    Odom --> Sync
    GPS --> Sync
    Pose --> Sync

    Sync --> Filter
    Filter -.-> EKF
    Filter -.-> UKF
    Filter -.-> Comp

    Filter --> FusedOdom
    Filter --> FusedPose
```

## Class Design

### Core
- **`FusionNode`**: The ROS 2 node that handles subscriptions, parameter updates, and publishing.
- **`FusionCore`**: The library class that manages the filter state and logic, independent of ROS 2 (mostly).

### Filters
All filters inherit from a base `FilterInterface` (conceptually, or via templates).

- **`ComplementaryFilter`**: Simple weight-based fusion for orientation.
- **`EKF`**: Extended Kalman Filter for non-linear state estimation.
- **`UKF`**: Unscented Kalman Filter for better handling of non-linearities without Jacobians.
