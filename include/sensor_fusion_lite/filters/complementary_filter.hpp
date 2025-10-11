#pragma once

#include "base_filter.hpp"
#include <mutex>

namespace sensor_fusion_lite
{
  class ComplementaryFilter : public BaseFilter
  {
  public:
    // alpha: blending factor for complementary filter (0..1), default 0.98
    explicit ComplementaryFilter(double alpha = 0.98);

    void initialize(const State &initial_state, int state_dim) override;
    void predict(double dt) override;

    void update_imu(const ImuMeasurement &imu) override;
    void update_odom(const OdomMeasurement &odom) override;
    void update_gps(const GpsMeasurement &gps) override;
    void update_pose(const PoseMeasurement &pose) override;

    void update_custom(const std::vector<std::vector<double>> &H,
                       const std::vector<double> &z,
                       const std::vector<std::vector<double>> &R,
                       Time timestamp) override;

    State get_state() const override;
    std::vector<std::vector<double>> get_covariance() const override;
    void set_state(const State &s) override;

  private:
    mutable std::mutex mtx_;
    State state_;
    std::vector<std::vector<double>> covariance_;
    double alpha_;
  };
} // namespace sensor_fusion_lite