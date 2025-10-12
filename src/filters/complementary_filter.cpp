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

  
} // namespace sensor_fusion_lite