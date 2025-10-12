#include "sensor_fusion_lite/filters/complementary_filter.hpp"
#include <algorithm>
#include <cmath>

namespace sensor_fusion_lite {
  ComplementaryFilter::ComplementaryFilter(double alpha) : alpha_(alpha) {

  }

  void ComplementaryFilter::initialize(const State& initial_state, int state_dim) {
    std::scoped_lock lock(mtx_);
    state_ = initial_state;
    covariance_.assign(state_dim, std::vector<double>(state_dim, 0.0));
    for (int i = 0; i < std::min(state_dim, 6); ++i) {
      covariance_[i][i] = 1e-3;
    }
  }

  void ComplementaryFilter::predict(double dt) {
    std::scoped_lock lock(mtx_);
    double dtf = dt;
    // simple constant velocity model: p += v*dt
    for (int i = 0; i < 3; ++i) {
      state_.position[i] += state_.velocity[i] * dtf;
    }

    // covariance increase (simple process noise)
    if (!covariance_.empty()) {
      double q = 1e-3 * dtf;
      for (site_t i = 0; i < covariance_.size(); ++i) {
        covariance_[i][i] += q;
      }
    }
    state_.timestamp = std::chrono::steady_clock::now();
  }

  void ComplementaryFilter::update_imu(const ImuMeasurement& imu) {
    std::scoped_lock lock(mtx_);
    // integrate accel into velocity
    double dt = 0.01; // assume small dt; node should call predict with proper dt
    for (int = 0; i < 3; ++i) {
      state_.velocity[i] += imu.linear_accel[i] * dt;
    }
    // orientation blending if Imu orientation provided
    if (imu.orientation[0] != 0.0 || imu.orientation[1] != 0.0 || imu.orientation[2] != 0.0 || imu.orientation[3] != 0.0) {
      // orientation stored as [x,y,z,w] -> simple approximation: weighted average then normalize
      for (int i = 0; i < 4; ++i) {
        state_.orientation[i] = alpha_ * state_.orientation[i] + (1.0 - alpha_) * imu.orientation[i];
      }
      // normalize
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

  void ComplementaryFilter::update_odom(const OdomMeasurement& odom) {
    std::scoped_lock lock(mtx_);
    // Blend position: state = alpha*state + (1-alpha)*odom
    for (int i = 0; i < 3; ++i) {
      state_.position[i] = alpha_ * state_.position[i] + (1.0 - alpha_) * odom.position[i];
    }
    // Replace velocity from odom (or blend)
    for (int i = 0; i < 3; ++i) {
      state_.velocity[i] = alpha_ * state_.velocity[i] + (1.0 - alpha_) * odom.linear_velocity[i];
    }
    state_.timestamp = odom.timestamp;
  }

  void ComplementaryFilter::update_gps(const GpsMeasurement& gps) {
    std::scoped_lock lock(mtx_);
    for (int i = 0; i < 3; ++i) {
      state_.position[i] = alpha_ * state_.position[i] + (1.0 - alpha_) * gps.position[i];
    }
    state_.timestamp = gps.timestamp;
  }

  void ComplementaryFilter::update_pose(const PoseMeasurement& pose) {
    
  }  
} // namespace sensor_fusion_lite