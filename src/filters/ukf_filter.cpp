#include "sensor_fusion_lite/filters/ukf_filter.hpp"
#include "sensor_fusion_lite/filters/ekf_filter.hpp" // reuse EKF functionality
#include <cstddef>
#include <iostream>

namespace sensor_fusion_lite {
UnscentedKalmanFilter::UnscentedKalmanFilter() {}
/**
 * @brief Initialize the filter with an initial state and state dimension.
 *
 * @param initial_state The initial state of the system.
 * @param state_dim The dimension of the state vector.
 */
void UnscentedKalmanFilter::initialize(const State &initial_state,
                                       int state_dim) {
  std::scoped_lock lock(mtx_);
  state_ = initial_state;
  P_.assign(state_dim, std::vector<double>(state_dim, 0.0));
  /**
   * @brief Initialize the covariance matrix with a small diagonal value.
   *
   * @param state_dim The dimension of the state vector.
   */
  for (int i = 0; i < std::min(state_dim, 6); ++i)
    P_[i][i] = 1e-3;
}

// Temporarily copy EKF functionality
void UnscentedKalmanFilter::predict(double dt) {
  std::scoped_lock lock(mtx_);
  for (int i = 0; i < 3; ++i)
    state_.position[i] += state_.velocity[i] * dt;

  if (!P_.empty()) {
    double q = 1e-3 * dt;
    for (size_t i = 0; i < P_.size(); ++i)
      P_[i][i] += q;
  }
  state_.timestamp = std::chrono::steady_clock::now();
}

void UnscentedKalmanFilter::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(mtx_);
  double dt = 0.01;
  for (int i = 0; i < 3; ++i)
    state_.velocity[i] += imu.linear_accel[i] * dt;
  state_.timestamp = imu.timestamp;
}

void UnscentedKalmanFilter::update_odom(const OdomMeasurement &odom) {
  std::scoped_lock lock(mtx_);
  for (int i = 0; i < 3; ++i) {
    double K = P_[i][i] / (P_[i][i] + 0.05);
    state_.position[i] += K * (odom.position[i] - state_.position[i]);
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = odom.timestamp;
}

void UnscentedKalmanFilter::update_gps(const GpsMeasurement &gps) {
  std::scoped_lock lock(mtx_);
  for (int i = 0; i < 3; ++i) {
    double K = P_[i][i] / (P_[i][i] + 3.0);
    state_.position[i] += K * (gps.position[i] - state_.position[i]);
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = gps.timestamp;
}

} // namespace sensor_fusion_lite