#include "sensor_fusion_lite/filters/complementary_filter.hpp"
#include <algorithm>
#include <cmath>

namespace sensor_fusion_lite {

/**
 * @brief Construct a new Complementary Filter object.
 * @param alpha The weighting factor.
 *              alpha ~ 1.0 -> Trust previous state/prediction more.
 *              alpha ~ 0.0 -> Trust new measurement more.
 */
ComplementaryFilter::ComplementaryFilter(double alpha) : alpha_(alpha) {}

void ComplementaryFilter::initialize(const State &initial_state,
                                     int state_dim) {
  std::scoped_lock lock(mtx_);
  state_ = initial_state;
  // Initialize covariance (though mostly unused in pure complementary filter)
  // to avoid singularities
  covariance_.assign(state_dim, std::vector<double>(state_dim, 0.0));
  for (int i = 0; i < std::min(state_dim, 6); ++i) {
    covariance_[i][i] = 1e-3;
  }
}

void ComplementaryFilter::predict(double dt) {
  std::scoped_lock lock(mtx_);
  double dtf = dt;
  // Simple Constant Velocity Model:
  // position[t] = position[t-1] + velocity[t-1] * dt
  for (int i = 0; i < 3; ++i) {
    state_.position[i] += state_.velocity[i] * dtf;
  }

  // artificially increase covariance (process noise)
  if (!covariance_.empty()) {
    double q = 1e-3 * dtf;
    for (size_t i = 0; i < covariance_.size(); ++i) {
      covariance_[i][i] += q;
    }
  }
  state_.timestamp = std::chrono::steady_clock::now();
}

void ComplementaryFilter::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(mtx_);

  // 1. Process Linear Acceleration
  // Basic dead reckoning: v = v + a * dt
  // Note: This drifts significantly without correction.
  double dt = 0.01; // FIXME: Should use actual dt between measurements
  for (int i = 0; i < 3; ++i) {
    state_.velocity[i] += imu.linear_accel[i] * dt;
  }

  // 2. Process Orientation
  // If the IMU provides an absolute orientation estimate (e.g., from on-board
  // sensor fusion), blend it.
  if (imu.orientation[0] != 0.0 || imu.orientation[1] != 0.0 ||
      imu.orientation[2] != 0.0 || imu.orientation[3] != 0.0) {
    // Simple Linear Interpolation (LERP) approximation for quaternions.
    // state.q = alpha * state.q + (1-alpha) * meas.q
    for (int i = 0; i < 4; ++i) {
      state_.orientation[i] =
          alpha_ * state_.orientation[i] + (1.0 - alpha_) * imu.orientation[i];
    }

    // Renormalize quaternion to maintain valid rotation
    double norm = 0.0;
    for (int i = 0; i < 4; ++i) {
      norm += state_.orientation[i] * state_.orientation[i];
    }
    if (norm > 0.0) {
      norm = std::sqrt(norm);
      for (int i = 0; i < 4; ++i) {
        state_.orientation[i] /= norm;
      }
    }
  }
  state_.timestamp = imu.timestamp;
}

void ComplementaryFilter::update_odom(const OdomMeasurement &odom) {
  std::scoped_lock lock(mtx_);
  // Blend Position: TRUST previous state by alpha, TRUST odom by (1-alpha)
  for (int i = 0; i < 3; ++i) {
    state_.position[i] =
        alpha_ * state_.position[i] + (1.0 - alpha_) * odom.position[i];
  }
  // Blend Velocity
  for (int i = 0; i < 3; ++i) {
    state_.velocity[i] =
        alpha_ * state_.velocity[i] + (1.0 - alpha_) * odom.linear_velocity[i];
  }
  state_.timestamp = odom.timestamp;
}

void ComplementaryFilter::update_gps(const GpsMeasurement &gps) {
  std::scoped_lock lock(mtx_);
  // Blend Position with GPS
  // Typically alpha is high (0.9+) to filter out GPS jitter, relying on
  // Odom/IMU for high freq updates
  for (int i = 0; i < 3; ++i) {
    state_.position[i] =
        alpha_ * state_.position[i] + (1.0 - alpha_) * gps.position[i];
  }
  state_.timestamp = gps.timestamp;
}

void ComplementaryFilter::update_pose(const PoseMeasurement &pose) {
  std::scoped_lock lock(mtx_);
  // Blend Position
  for (int i = 0; i < 3; ++i) {
    state_.position[i] =
        alpha_ * state_.position[i] + (1.0 - alpha_) * pose.position[i];
  }
  // Blend Orientation
  for (int i = 0; i < 4; ++i) {
    state_.orientation[i] =
        alpha_ * state_.orientation[i] + (1.0 - alpha_) * pose.orientation[i];
  }
  // Normalize Orientation
  double norm = 0.0;
  for (int i = 0; i < 4; ++i) {
    norm += state_.orientation[i] * state_.orientation[i];
  }
  if (norm > 0.0) {
    norm = std::sqrt(norm);
    for (int i = 0; i < 4; ++i) {
      state_.orientation[i] /= norm;
    }
  }
  state_.timestamp = pose.timestamp;
}

void ComplementaryFilter::update_custom(
    const std::vector<std::vector<double>> &, const std::vector<double> &,
    const std::vector<std::vector<double>> &, Time timestamp) {
  std::scoped_lock lock(mtx_);
  state_.timestamp = timestamp;
}

State ComplementaryFilter::get_state() const {
  std::scoped_lock lock(mtx_);
  return state_;
}

std::vector<std::vector<double>> ComplementaryFilter::get_covariance() const {
  std::scoped_lock lock(mtx_);
  return covariance_;
}

void ComplementaryFilter::set_state(const State &s) {
  std::scoped_lock lock(mtx_);
  state_ = s;
}
} // namespace sensor_fusion_lite