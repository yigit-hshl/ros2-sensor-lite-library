#include "sensor_fusion_lite/filters/ukf_filter.hpp"
#include "sensor_fusion_lite/filters/ekf_filter.hpp" // reuse EKF functionality
#include <cstddef>
#include <iostream>
#include <vector>

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

void UnscentedKalmanFilter::update_pose(const PoseMeasurement &pose) {
  std::scoped_lock lock(mtx_);
  for (int i = 0; i < 3; ++i) {
    double K = P_[i][i] / (P_[i][i] + 0.02);
    state_.position[i] += K * (pose.position[i] - state_.position[i]);
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.orientation = pose.orientation;
  state_.timestamp = pose.timestamp;
}

void UnscentedKalmanFilter::update_custom(
    const std::vector<std::vector<double>> &H, const std::vector<double> &z,
    const std::vector<std::vector<double>> &R, Time timestamp) {
  std::scoped_lock lock(mtx_);
  // Simple fallback to diagonal updates (like EKF)
  size_t m = z.size();
  for (size_t i = 0; i < m && i < state_.position.size(); ++i) {
    double K = P_[i][i] /
               (P_[i][i] + ((R.size() > i && R[i].size() > i) ? R[i][i] : 0.1));
    state_.position[i] += K * (z[i] - state_.position[i]);
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = timestamp;
}

State UnscentedKalmanFilter::get_state() const {
  std::scoped_lock lock(mtx_);
  return state_;
}

std::vector<std::vector<double>> UnscentedKalmanFilter::get_covariance() const {
  std::scoped_lock lock(mtx_);
  return P_;
}

void UnscentedKalmanFilter::set_state(const State &s) {
  std::scoped_lock lock(mtx_);
  state_ = s;
}
} // namespace sensor_fusion_lite