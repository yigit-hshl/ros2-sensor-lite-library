#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

namespace sensor_fusion_lite {

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::initialize(const State &s, int,
                                      const FusionConfig &config) {
  std::scoped_lock lock(mtx_);

  x_.assign(N, 0.0);
  x_[0] = s.position[0];
  x_[1] = s.position[1];
  x_[2] = s.position[2];
  x_[3] = s.velocity[0];
  x_[4] = s.velocity[1];
  x_[5] = s.velocity[2];

  P_ = mat_identity(N);

  // Initialize Q from config if available, else default
  if (config.ekf_config.initial_process_noise.size() == N) {
    Q_ = mat_identity(N);
    for (size_t i = 0; i < N; ++i)
      Q_[i][i] = config.ekf_config.initial_process_noise[i];
  } else {
    Q_ = mat_identity(N);
    for (size_t i = 0; i < N; ++i)
      Q_[i][i] = 0.01;
  }
}

void ExtendedKalmanFilter::predict(double dt) {
  std::scoped_lock lock(mtx_);

  // State transition (Constant Velocity model)
  // x = x + vx * dt
  // y = y + vy * dt
  // z = z + vz * dt
  x_[0] += x_[3] * dt;
  x_[1] += x_[4] * dt;
  x_[2] += x_[5] * dt;

  // Build Jacobian F
  auto F = mat_identity(N);
  F[0][3] = dt;
  F[1][4] = dt;
  F[2][5] = dt;

  // P = F*P*F^T + Q
  P_ = mat_add(mat_mul(mat_mul(F, P_), mat_transpose(F)), Q_);
}

void ExtendedKalmanFilter::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(mtx_);
  // Simple integration for velocity update (naive)
  // In a full EKF, this would be part of prediction or a measurement update for
  // acceleration Here we treat it as a direct control input or simplistic
  // update

  // Note: dt should ideally be calculated. Using a placeholder if not provided.
  // We don't track measuring time interval here perfectly without state
  // timestamp diff. Assuming small dt  // Calculate dt
  double dt =
      std::chrono::duration<double>(imu.timestamp - x_last_timestamp_).count();
  if (dt <= 0.0)
    dt = 0.01;
  x_last_timestamp_ = imu.timestamp;

  x_[3] += imu.linear_accel[0] * dt;
  x_[4] += imu.linear_accel[1] * dt;
  x_[5] += imu.linear_accel[2] * dt;
}

void ExtendedKalmanFilter::update_odom(const OdomMeasurement &odom) {
  std::scoped_lock lock(mtx_);

  // Measurement vector z: [x, y, z, vx, vy, vz]
  std::vector<double> z(6);
  z[0] = odom.position[0];
  z[1] = odom.position[1];
  z[2] = odom.position[2];
  z[3] = odom.linear_velocity[0];
  z[4] = odom.linear_velocity[1];
  z[5] = odom.linear_velocity[2];

  // Measurement matrix H: Identity (direct observation of state)
  auto H = mat_identity(N);

  // Measurement noise R
  auto R = mat_identity(N);
  for (size_t i = 0; i < 6; ++i)
    R[i][i] = odom.cov_diag[i] > 0 ? odom.cov_diag[i] : 0.1;

  ekf_update(z, H, R, odom.timestamp);
}

void ExtendedKalmanFilter::update_gps(const GpsMeasurement &gps) {
  std::scoped_lock lock(mtx_);

  // Measurement vector z: [x, y, z]
  std::vector<double> z(3);
  z[0] = gps.position[0];
  z[1] = gps.position[1];
  z[2] = gps.position[2];

  // H: Observes first 3 states
  std::vector<std::vector<double>> H(3, std::vector<double>(N, 0.0));
  H[0][0] = 1.0;
  H[1][1] = 1.0;
  H[2][2] = 1.0;

  // R from measurement
  auto R = gps.cov;

  ekf_update(z, H, R, gps.timestamp);
}

void ExtendedKalmanFilter::update_pose(const PoseMeasurement &pose) {
  std::scoped_lock lock(mtx_);
  // Similar to Odom but only Position + Orientation (Orientation ignored in
  // this 6D KF for now)

  // Measurement z: [x, y, z]
  std::vector<double> z(3);
  z[0] = pose.position[0];
  z[1] = pose.position[1];
  z[2] = pose.position[2];

  // H
  std::vector<std::vector<double>> H(3, std::vector<double>(N, 0.0));
  H[0][0] = 1.0;
  H[1][1] = 1.0;
  H[2][2] = 1.0;

  // R
  auto R = mat_identity(3);
  for (int i = 0; i < 3; ++i)
    R[i][i] = pose.cov_diag[i] > 0 ? pose.cov_diag[i] : 0.5;

  ekf_update(z, H, R, pose.timestamp);
}

void ExtendedKalmanFilter::update_custom(
    const std::vector<std::vector<double>> &H, const std::vector<double> &z,
    const std::vector<std::vector<double>> &R, Time timestamp) {
  std::scoped_lock lock(mtx_);
  ekf_update(z, H, R, timestamp);
}

void ExtendedKalmanFilter::ekf_update(const std::vector<double> &z,
                                      const std::vector<std::vector<double>> &H,
                                      const std::vector<std::vector<double>> &R,
                                      Time) {
  // 1. Innovation y = z - Hx
  // 2. Innovation covariance S = H P H^T + R
  // 3. Kalman Gain K = P H^T S^-1
  // 4. Update x = x + Ky
  // 5. Update P = (I - KH) P

  size_t m = z.size(); // measurement dimension

  // y = z - Hx
  std::vector<double> Hx = mat_vec_mul(H, x_);
  std::vector<double> y(m);
  for (size_t i = 0; i < m; ++i)
    y[i] = z[i] - Hx[i];

  // S = H P H^T + R
  auto HP = mat_mul(H, P_);
  auto HPHt = mat_mul(HP, mat_transpose(H));
  auto S = mat_add(HPHt, R);

  // K = P H^T S^-1
  // Simplification: assume S is diagonal for inversion if large, or implement
  // 3x3 inv For general sizing, we need a solver. Here we implement a simple
  // inversion for small matrices (up to 3x3 or diagonal). Fallback: Diagonal
  // inversion
  auto S_inv = mat_identity(m);
  // TODO: Full matrix inversion. For now, assuming diagonal dominate or small.
  // Implementing 3x3 inverse if m=3
  if (m == 3) {
    S_inv = mat_inverse_3x3(S);
  } else {
    // Diagonal inverse fallback
    for (size_t i = 0; i < m; ++i)
      S_inv[i][i] = 1.0 / (S[i][i] + 1e-9);
  }

  auto PHt = mat_mul(P_, mat_transpose(H));
  auto K = mat_mul(PHt, S_inv);

  // x = x + Ky
  auto Ky = mat_vec_mul(K, y);
  for (size_t i = 0; i < N; ++i)
    x_[i] += Ky[i];

  // P = (I - KH) P
  auto KH = mat_mul(K, H);
  auto I = mat_identity(N);
  // I - KH
  auto I_KH = I;
  for (size_t i = 0; i < N; ++i)
    for (size_t j = 0; j < N; ++j)
      I_KH[i][j] -= KH[i][j];

  P_ = mat_mul(I_KH, P_);
}

State ExtendedKalmanFilter::get_state() const {
  State s{};
  s.position = {x_[0], x_[1], x_[2]};
  s.velocity = {x_[3], x_[4], x_[5]};
  s.orientation = {0, 0, 0, 1}; // EKF here tracks pos/vel only
  // s.timestamp updated externally or in predict
  return s;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::get_covariance() const {
  return P_;
}

void ExtendedKalmanFilter::set_state(const State &s) {
  x_[0] = s.position[0];
  x_[1] = s.position[1];
  x_[2] = s.position[2];
  x_[3] = s.velocity[0];
  x_[4] = s.velocity[1];
  x_[5] = s.velocity[2];
}

// ---------------- Helpers ----------------
std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_add(const std::vector<std::vector<double>> &A,
                              const std::vector<std::vector<double>> &B) {
  size_t rows = A.size();
  size_t cols = A[0].size();
  std::vector<std::vector<double>> C(rows, std::vector<double>(cols));
  for (size_t i = 0; i < rows; ++i)
    for (size_t j = 0; j < cols; ++j)
      C[i][j] = A[i][j] + B[i][j];
  return C;
}

std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_mul(const std::vector<std::vector<double>> &A,
                              const std::vector<std::vector<double>> &B) {
  size_t rows = A.size();
  size_t cols = B[0].size();
  size_t inner = B.size();
  std::vector<std::vector<double>> C(rows, std::vector<double>(cols, 0.0));
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      for (size_t k = 0; k < inner; ++k) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

std::vector<double>
ExtendedKalmanFilter::mat_vec_mul(const std::vector<std::vector<double>> &A,
                                  const std::vector<double> &x) {
  size_t rows = A.size();
  size_t cols = x.size();
  std::vector<double> y(rows, 0.0);
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      y[i] += A[i][j] * x[j];
    }
  }
  return y;
}

std::vector<std::vector<double>>
ExtendedKalmanFilter::mat_transpose(const std::vector<std::vector<double>> &A) {
  size_t rows = A.size();
  size_t cols = A[0].size();
  std::vector<std::vector<double>> At(cols, std::vector<double>(rows));
  for (size_t i = 0; i < rows; ++i)
    for (size_t j = 0; j < cols; ++j)
      At[j][i] = A[i][j];
  return At;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::mat_identity(size_t n) {
  std::vector<std::vector<double>> I(n, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i)
    I[i][i] = 1.0;
  return I;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::mat_inverse_3x3(
    const std::vector<std::vector<double>> &A) {
  // Determinant
  double det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
               A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
               A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  double invDet = 1.0 / (det + 1e-9);

  std::vector<std::vector<double>> B(3, std::vector<double>(3));
  B[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) * invDet;
  B[0][1] =
      (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet; // Transposed cofactor
  B[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

  B[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
  B[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
  B[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * invDet;

  B[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) * invDet;
  B[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) * invDet;
  B[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) * invDet;

  return B;
}

double ExtendedKalmanFilter::clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

} // namespace sensor_fusion_lite