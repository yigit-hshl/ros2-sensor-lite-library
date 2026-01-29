#pragma once

#include "base_filter.hpp"
#include <mutex>

namespace sensor_fusion_lite {

class ExtendedKalmanFilter : public BaseFilter {
public:
  ExtendedKalmanFilter();

  void initialize(const State &initial_state, int state_dim,
                  const FusionConfig &config) override;
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
  static constexpr size_t N = 6;

  mutable std::mutex mtx_;

  // state vector
  std::vector<double> x_;

  // covariance matrix
  std::vector<std::vector<double>> P_;
  // process noise covariance
  std::vector<std::vector<double>> Q_;

  // Last timestamp for dt calculation
  Time x_last_timestamp_;

  // helper matrix ops (private)
  static std::vector<std::vector<double>>
  mat_add(const std::vector<std::vector<double>> &A,
          const std::vector<std::vector<double>> &B);

  static std::vector<std::vector<double>>
  mat_mul(const std::vector<std::vector<double>> &A,
          const std::vector<std::vector<double>> &B);

  static std::vector<double>
  mat_vec_mul(const std::vector<std::vector<double>> &A,
              const std::vector<double> &x);

  static std::vector<std::vector<double>>
  mat_transpose(const std::vector<std::vector<double>> &A);

  static std::vector<std::vector<double>> mat_identity(size_t n);

  static std::vector<std::vector<double>>
  mat_inverse_3x3(const std::vector<std::vector<double>> &A);

  static double clamp(double v, double lo, double hi);

  void ekf_update(const std::vector<double> &z,
                  const std::vector<std::vector<double>> &H,
                  const std::vector<std::vector<double>> &R, Time timestamp);
};
} // namespace sensor_fusion_lite