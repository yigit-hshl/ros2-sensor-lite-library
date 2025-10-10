#pragma once

#include "../fusion_core.hpp"
#include <memory>

namespace sensor_fusion_lite
{
  class BaseFilter
  {
  public:
    virtual ~BaseFilter() = default;

    // initialize with given state and dimension
    virtual void initialize(const State &initial_state, int state_dim) = 0;

    // predict state forward by dt (seconds)
    virtual void predict(double dt) = 0;

    // sensor-specific updates
    virtual void update_imu(const ImuMeasurement &imu) = 0;
    virtual void update_odom(const OdomMeasurement &odom) = 0;
    virtual void update_gps(const GpsMeasurement &gps) = 0;
    virtual void update_pose(const PoseMeasurement &pose) = 0;

    // custom linear update: H (m x n), z (m), R (m x m)
    virtual void update_custom(const std::vector<std::vector<double>> &H,
                               const std::vector<double> &z,
                               const std::vector<std::vector<double>> &R,
                               Time timestamp) = 0;

    virtual State get_state() const = 0;
    virtual std::vector<std::vector<double>> get_covariance() const = 0;
    virtual void set_state(const State& s) = 0;
  };

  using BaseFilterPtr = std::unique_ptr<BaseFilter>;
} // namespace sensor_fusion_lite