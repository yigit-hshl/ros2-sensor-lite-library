#include "sensor_fusion_lite/filters/ukf_filter.hpp"
#include "sensor_fusion_lite/filters/ekf_filter.hpp" // reuse EKF functionality
#include <iostream>

namespace sensor_fusion_lite
{
  UnscentedKalmanFilter::UnscentedKalmanFilter() {}

  void UnscentedKalmanFilter::initialize(const State& initial_state, int state_dim)
  {
    std::scoped_lock lock(mtx_);
    state_ = initial_state;
    P_.assign(state_dim, std::vector<double>(state_dim, 0.0));
    for (int i = 0; i < std::min(state_dim, 6); ++i) P_[i][i] = 1e-3;
  }
} // namespace sensor_fusion_lite