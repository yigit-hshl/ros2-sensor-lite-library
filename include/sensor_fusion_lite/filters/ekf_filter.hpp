#pragma once

#include "base_filter.hpp"
#include <mutex>

namespace sensor_fusion_lite
{
  class ExtendedKalmanFilter : public BaseFilter
  {
  public:
    ExtendedKalmanFilter();

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
    std::vector<std::vector<double>> P_; // covariance
    std::vector<std::vector<double>> Q_; // process noise
    // helper matrix ops (private)
    static std::vector<std::vector<double>> mat_add(const std::vector<std::vector<double>> &A,
                                                    const std::vector<std::vector<double>> &B);
    static std::vector<std::vector<double>> mat_mul(const std::vector<std::vector<double>> &A,
                                                    const std::vector<std::vector<double>> &B);
    static std::vector<std::vector<double>> mat_transpose(const std::vector<std::vector<double>> &A);
    static std::vector<std::vector<double>> mat_identity(size_t n, double val = 1.0);
    static std::vector<double> mat_vec_mul(const std::vector<std::vector<double>> &A,
                                           const std::vector<double> &v);
    static double clamp(double v, double lo, double hi);
  };
} // namespace sensor_fusion_lite