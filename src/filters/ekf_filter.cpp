#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include <cassert>
#include <cmath>


namespace sensor_fusion_lite {

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::initialize(const State &s, int) {
  std::scoped_lock lock(mtx_);

  x_.assign(N, 0.0);
  x_[0] = s.position[0];
  x_[1] = s.position[1];
  x_[2] = s.position[2];
  x_[3] = s.velocity[0];
  x_[4] = s.velocity[1];
  x_[5] = s.velocity[2];

  P_ = mat_identity(N);
  Q_ = mat_identity(N);

  for (size_t i = 0; i < N; ++i) {
    P_[i][i] = 0.1;
    Q_[i][i] = 0.01;
  }
}

void ExtendedKalmanFilter::predict(double dt) {
  std::scoped_lock lock(mtx_);

  // State transition
  x_[0] += x_[3] * dt;
  x_[1] += x_[4] * dt;
  x_[2] += x_[5] * dt;

  // Build F
  auto F = mat_identity(N);
  F[0][3] = dt;
  F[1][4] = dt;
  F[2][5] = dt;

  // P = F*P*(F_transpose) + Q
  P_ = mat_add(mat_mul(mat_mul(F, P_), mat_transpose(F)), Q_);
}

void ExtendedKalmanFilter::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(mtx_);

  // IMU accel integrates velocity
  constexpr double dt = 0.01;
  x_[3] += imu.linear_accel[0] * dt;
  x_[4] += imu.linear_accel[1] * dt;
  x_[5] += imu.linear_accel[2] * dt;
}

} // namespace sensor_fusion_lite