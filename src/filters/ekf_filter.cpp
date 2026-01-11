#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include <algorithm>
#include <cmat>


namespace sensor_fusion_lite {

/**
 * @brief Construct a new Extended Kalman Filter object.
 */
ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::initialize(const State &initial_state,
                                      int state_dim) {
  std::scoped_lock lock(mtx_);
  state_ = initial_state;
  // P = initial State Covariance
  P_.assign(state_dim, std::vector<double>(state_dim, 0.0));
  // Q = Process Noise Covariance
  Q_.assign(state_dim, std::vector<double>(state_dim, 0.0));
  // Initialize P with small uncertainty
  for (int i = 0; i < std::min(state_dim, 6); ++i)
    P_[i][i] = 1e-3;
  // Initialize Q with small process noise
  for (int i = 0; i < std::min(state_dim, 6); ++i)
    Q_[i][i] = 1e-4;
}

void ExtendedKalmanFilter::predict(double dt) {
  std::scoped_lock lock(mtx_);
  // ---------------- Predict Step ----------------
  // x_k|k-1 = f(x_k-1, u_k)
  // Here we use a simple linear Kinematic model: position += velocity * dt
  // If the model were non-linear, we would compute the Jacobian F here.

  // state vector assumed [px, py, pz, vx, vy, vz]
  // px += vx*dt; py += vy*dt; pz += vz*dt
  for (int i = 0; i < 3; ++i)
    state_.position[i] += state_.velocity[i] * dt;

  // P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
  // For this simple linear model, F ~ I (mostly).
  // We simplify P update: P = P + Q*dt (approximation for time scaling)
  for (size_t i = 0; i < P_.size(); ++i) {
    P_[i][i] += Q_[i][i] * dt;
  }
  state_.timestamp = std::chrono::steady_clock::now();
}

void ExtendedKalmanFilter::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(mtx_);
  // In a full EKF, IMU accel would be an input u to the prediction step or a
  // measurement of acceleration. Here, we treat it as a direct velocity
  // correction (simplification).

  double dt = 0.01; // FIXME: Use actual delta
  for (int i = 0; i < 3; ++i)
    state_.velocity[i] += imu.linear_accel[i] * dt;

  state_.timestamp = imu.timestamp;
}

void ExtendedKalmanFilter::update_odom(const OdomMeasurement &odom) {
  std::scoped_lock lock(mtx_);
  // ---------------- Update Step ----------------
  // z = Measurement Vector (Position)
  std::vector<double> z = {odom.position[0], odom.position[1],
                           odom.position[2]};

  // H = Observation Matrix (d_h / d_x)
  // Maps state [px, py, pz, ...] to measurement [px, py, pz]
  // H = [I3, 03]

  // y = Innovation (Residual) = z - h(x)
  std::vector<double> hx = {state_.position[0], state_.position[1],
                            state_.position[2]};
  std::vector<double> y(3);
  for (int i = 0; i < 3; ++i)
    y[i] = z[i] - hx[i];

  // R = Measurement Noise Covariance
  double Rdiag = 0.05; // Standard deviation or variance
  // Standard Kalman Gain Calculation (simplified diagonal)
  // K = P * H^T * (H * P * H^T + R)^-1
  for (int i = 0; i < 3; ++i) {
    double K = P_[i][i] / (P_[i][i] + Rdiag);
    // Correction: x = x + K * y
    state_.position[i] += K * y[i];
    // Covariance Update: P = (I - K * H) * P
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = odom.timestamp;
}

void ExtendedKalmanFilter::update_gps(const GpsMeasurement &gps) {
  std::scoped_lock lock(mtx_);
  // Similar to Odom Update, but with higher uncertainty R
  std::vector<double> z = {gps.position[0], gps.position[1], gps.position[2]};
  double Rdiag = 3.0; // Higher noise for GPS

  for (int i = 0; i < 3; ++i) {
    double hx = state_.position[i];
    double y = z[i] - hx;
    double K = P_[i][i] / (P_[i][i] + Rdiag);
    state_.position[i] += K * y;
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = gps.timestamp;
}

void ExtendedKalmanFilter::update_pose(const PoseMeasurement &pose) {
  std::scoped_lock lock(mtx_);
  // Positional Update
  std::vector<double> z = {pose.position[0], pose.position[1],
                           pose.position[2]};
  double Rdiag = 0.02; // Trusted pose source
  for (int i = 0; i < 3; ++i) {
    double y = z[i] - state_.position[i];
    double K = P_[i][i] / (P_[i][i] + Rdiag);
    state_.position[i] += K * y;
    P_[i][i] = (1 - K) * P_[i][i];
  }
  // Simple orientation assignment (EKF for quaternion is complex, omitted for
  // 'lite')
  state_.orientation = pose.orientation;
  state_.timestamp = pose.timestamp;
}

void ExtendedKalmanFilter::update_custom(
    const std::vector<std::vector<double>> &H, const std::vector<double> &z,
    const std::vector<std::vector<double>> &R, Time timestamp) {
  std::scoped_lock lock(mtx_);
  // Helper for generic linear update if H is aligned with state
  size_t m = z.size();
  for (size_t i = 0; i < m && i < state_.position.size(); ++i) {
    double hx = state_.position[i];
    double y = z[i] - hx;
    double Rdiag = (R.size() > i && R[i].size() > i) ? R[i][i] : 0.1;
    double K = P_[i][i] / (P_[i][i] + Rdiag);
    state_.position[i] += K * y;
    P_[i][i] = (1 - K) * P_[i][i];
  }
  state_.timestamp = timestamp;
}

State ExtendedKalmanFilter::get_state() const {
  std::scoped_lock lock(mtx_);
  return state_;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::get_covariance() const {
  std::scoped_lock lock(mtx_);
  return P_;
}

void ExtendedKalmanFilter::set_state(const State &s) {
  std::scoped_lock lock(mtx_);
  state_ = s;
}

// Helper Matrix Operations
std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_add(const std::vector<std::vector<double>> &A,
                              const std::vector<std::vector<double>> &B) {
  size_t n = A.size();
  std::vector<std::vector<double>> C = A;
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < A[i].size(); ++j)
      C[i][j] += B[i][j];

  return C;
}

std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_mul(const std::vector<std::vector<double>> &A,
                              const std::vector<std::vector<double>> &B) {
  size_t n = A.size();
  size_t m = B[0].size();
  std::vector<std::vector<double>> C(n, std::vector<double>(m, 0.0));
  for (size_t i = 0; i < n; ++i)
    for (size_t k = 0; k < B.size(); ++k)
      for (size_t j = 0; j < m; ++j)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_transpose(const std::vector<std::vector<double>> &A) {
  size_t n = A.size(), m = A[0].size();
  std::vector<std::vector<double>> T(m, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < m; ++j)
      T[j][i] = A[i][j];
  return T;
}

std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_identity(size_t n, double val) {
  std::vector<std::vector<double>> I(n, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i)
    I[i][i] = val;
  return I;
}

std::vector<double>
ExtendedKalmanFilter::mat_vec_mul(const std::vector<std::vector<double>> &A,
                                  const std::vector<double> &v) {
  std::vector<double> out(A.size(), 0.0);
  for (size_t i = 0; i < A.size(); ++i)
    for (size_t j = 0; j < v.size(); ++j)
      out[i] += A[i][j] * v[j];
  return out;
}

double ExtendedKalmanFilter::clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
} // namespace sensor_fusion_lite