#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include <cmat>
#include <algorithm>

namespace sensor_fusion_lite
{
  ExtendedKalmanFilter::ExtendedKalmanFilter()
  {
  }

  void ExtendedKalmanFilter::initialize(const State &initial_state, int state_dim)
  {
    std::scoped_lock lock(mtx_);
    state_ = initial_state;
    P_.assign(state_dim, std::vector<double>(state_dim, 0.0));
    Q_.assign(state_dim, std::vector<double>(state_dim, 0.0));
    for (int i = 0; i < std::min(state_dim, 6); ++i)
      P_[i][i] = 1e-3;
    for (int i = 0; i < std::min(state_dim, 6); ++i)
      Q_[i][i] = 1e-4;
  }

  void ExtendedKalmanFilter::predict(double dt)
  {
    std::scoped_lock lock(mtx_);
    // state vector assumed [px, py, pz, vx, vy, vz]
    // px += vx*dt; py += vy*dt; pz += vz*dt
    for (int i = 0; i < 3; ++i)
      state_.position[i] += state_.velocity[i] * dt;

    // P = P + Q*dt (simple approximation)
    for (size_t i = 0; i < P_.size(); ++i)
    {
      P_[i][i] += Q_.[i][i] * dt;
    }
    state_.timestamp = std::chrono::steady_clock::now();
  }

  void ExtendedKalmanFilter::update_imu(const ImuMeasurement &imu)
  {
    std::scoped_lock lock(mtx_);
    // use accel to adjust velocity (simple correction)
    double dt = 0.01;
    for (int i = 0; i < 3; ++i)
      state_.velocity[i] += imu.linear_accel[i] * dt;
    state_.timestamp = imu.timestamp;
  }

  void ExtendedKalmanFilter::update_odom(const OdomMeasurement &odom)
  {
    std::scoped_lock lock(mtx_);
    // measurement z = position (3)
    std::vector<double> z = {odom.position[0], odom.position[1], odom.position[2]};
    // H = [I3, 03] mapping state to position
    // compute innovation y = z - H*x
    std::vector<double> hx = {state_.position[0], state_.position[1], state_.position[2]};
    std::vector<double> y(3);
    for (int i = 0; i < 3; ++i)
      y[i] = z[i] - hx[i];

    // Kalman gain using diagonal P and R approximated from odom.cov_diag if present
    double Rdiag = 0.05;
    for (int i = 0; i < 3; ++i)
    {
      double K = P_[i][i] / (P_[i][i] + Rdiag);
      state_.position[i] += K * y[i];
      // reduce P
      P_[i][i] = (1 - K) * P_[i][i];
    }
    state_.timestamp = odom.timestamp;
  }

  void ExtendedKalmanFilter::update_gps(const GpsMeasurement &gps)
  {
    std::scoped_lock lock(mtx_);
    std::vector<double> z = {gps.position[0], gps.position[1], gps.position[2]};
    double Rdiag = 3.0;
    for (int i = 0; i < 3; ++i)
    {
      double hx = state_.position[i];
      double y = z[i] - hx;
      double K = P_[i][i] / (P_[i][i] + Rdiag);
      state_.position[i] += K * y;
      P_[i][i] = (1 - K) * P_[i][i];
    }
    state_.timestamp = gps.timestamp;
  }

  void ExtendedKalmanFilter::update_pose(const PoseMeasurement &pose)
  {
    std::scoped_lock lock(mtx_);
    std::vector<double> z = {pose.position[0], pose.position[1], pose.position[2]};
    double Rdiag = 0.02;
    for (int i = 0; i < 3; ++i)
    {
      double y = z[i] - state_.position[i];
      double K = P_[i][i] / (P_[i][i] + Rdiag);
      state_.position[i] += K * y;
      P_[i][i] = (1 - K) * P_[i][i];
    }
    // update orientation with direct assignment
    state_.orientation = pose.orientation;
    state_.timestamp = pose.timestamp;
  }

  void ExtendedKalmanFilter::update_custom(const std::vector<std::vector<double>> &H,
                                           const std::vector<double> &z,
                                           const std::vector<std::vector<double>> &R,
                                           Time timestamp)
  {
    std::scoped_lock lock(mtx_);
    // if H maps to first m states, do a simple diagonal gain update
    size_t m = z.size();
    for (size_t i = 0; i < m && i < state_.position.size(); ++i)
    {
      double hx = state_.position[i];
      double y = z[i] - hx;
      double Rdiag = (R.size() > i && R[i].size() > i) ? R[i][i] : 0.1;
      double K = P_[i][i] / (P_[i][i] + Rdiag);
      state_.position[i] += K * y;
      P_[i][i] = (1 - K) * P_[i][i];
    }
    state_.timestamp = timestamp;
  }

  State ExtendedKalmanFilter::get_state() const
  {
    std::scoped_lock lock(mtx_);
    return state_;
  }

  std::vector<std::vector<double>> ExtendedKalmanFilter::get_covariance() const
  {
    std::scoped_lock lock(mtx_);
    return P_;
  }

  void ExtendedKalmanFilter::set_state(const State &s)
  {
    std::scoped_lock lock(mtx_);
    state_ = s;
  }

  std::vector<std::vector<double>> ExtendedKalmanFilter::mat_add(
      const std::vector<std::vector<double>> &A,
      const std::vector<std::vector<double>> &B)
  {
    size_t n = A.size();
    std::vector<std::vector<double>> C = A;
    for (size_t i = 0; i < n; ++i)
      for (size_t j = 0; j < A[i].size(); ++j)
        C[i][j] += B[i][j];

    return C;
  }
} // namespace sensor_fusion_lite