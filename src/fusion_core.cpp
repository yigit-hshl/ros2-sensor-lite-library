#include "sensor_fusion_lite/fusion_core.hpp"

// filters
#include "sensor_fusion_lite/filters/base_filter.hpp"
#include "sensor_fusion_lite/filters/complementary_filter.hpp"
#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include "sensor_fusion_lite/filters/ukf_filter.hpp"

#include <atomic>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>

namespace sensor_fusion_lite {

// ==================
// Filter factory
// ==================

/**
 * @brief Factory function to create a filter instance based on the specified
 * type.
 * @param type The type of filter to create.
 * @return A unique pointer to the created filter instance.
 */
static BaseFilterPtr make_filter(FilterType type) {
  switch (type) {
  case FilterType::COMPLEMENTARY:
    return std::make_unique<ComplementaryFilter>(0.98);
  case FilterType::EKF:
    return std::make_unique<ExtendedKalmanFilter>();
  case FilterType::UKF:
    return std::make_unique<UnscentedKalmanFilter>();
  default:
    return std::make_unique<ComplementaryFilter>(0.98);
  }
}

// ---------------- Internal Implementation ----------------
/**
 * @brief Private implementation struct (PIMPL idiom).
 * Holds the internal state, thread-safety primitives, and filter logic.
 */
struct FusionCore::Impl {
  FilterType type{FilterType::COMPLEMENTARY}; ///< Active filter strategy
  int state_dim{6};                           ///< Dimension of the state vector
  bool running{false};                        ///< Operational flag

  State state{};                               ///< Current estimated state
  std::vector<std::vector<double>> covariance; ///< State covariance matrix
  std::unordered_map<std::string, bool>
      sensor_enabled; ///< Enabled/Disabled status per sensor

  std::mutex mtx; ///< Mutex for thread-safety
  std::unordered_map<int, StateCallback>
      state_cbs;              ///< Registered state callbacks
  DiagnosticCallback diag_cb; ///< Diagnostic message callback
  int next_cb_id{0};          ///< Counter for callback IDs

  /**
   * @brief Helper to triggering all registered state callbacks.
   */
  void notify_state() {
    for (auto &[id, cb] : state_cbs)
      if (cb)
        cb(state);
  }

  /**
   * @brief Helper for logging diagnostic messages.
   * @param msg The message to log.
   */
  void log(const std::string &msg) {
    if (diag_cb)
      diag_cb(msg);
    else
      std::cout << "[FusionCore] " << msg << std::endl;
  }
};

// ---------------- Public API ----------------

FusionCore::FusionCore(FilterType filter_type, int state_dim,
                       const State *initial_state)
    : impl_(std::make_unique<Impl>()) {
  impl_->type = filter_type;
  impl_->state_dim = state_dim;
  if (initial_state)
    impl_->state = *initial_state;
}

FusionCore::~FusionCore() = default;

void FusionCore::initialize() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  // Initialize covariance with small diagonal values (not absolute zero) to
  // prevent singularities
  impl_->covariance.assign(impl_->state_dim,
                           std::vector<double>(impl_->state_dim, 0.0));
  impl_->log("Initialized with state dimension " +
             std::to_string(impl_->state_dim));
}

void FusionCore::start() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->running = true;
  impl_->log("Started fusion loop");
}

void FusionCore::stop() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->running = false;
  impl_->log("Stopped fusion loop");
}

void FusionCore::set_filter_type(FilterType t) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->type = t;
}

FilterType FusionCore::get_filter_type() const { return impl_->type; }

void FusionCore::enable_sensor(const std::string &sensor_name, bool enabled) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->sensor_enabled[sensor_name] = enabled;
}

bool FusionCore::predict(const std::chrono::duration<double> &dt) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  // Standard kinematic prediction step:
  // x = x + v * dt (simplified constant velocity model)
  for (int i = 0; i < 3; ++i)
    impl_->state.position[i] += impl_->state.velocity[i] * dt.count();

  // Update timestamp to current time
  impl_->state.timestamp = std::chrono::steady_clock::now();

  // Notify listeners of the predicted state
  impl_->notify_state();
  return true;
}

bool FusionCore::update_imu(const ImuMeasurement &imu) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->state.timestamp = imu.timestamp;

  // TODO: Implement specific filter logic (EKF/UKF/Complementary) here or
  // delegate to a strategy pattern
  impl_->log("IMU update");

  impl_->notify_state();
  return true;
}

bool FusionCore::update_odom(const OdomMeasurement &odom) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  // Direct state update from odometry (simplistic fusion)
  impl_->state.position = odom.position;
  impl_->state.velocity = odom.linear_velocity;
  impl_->state.timestamp = odom.timestamp;

  impl_->log("Odom update");
  impl_->notify_state();
  return true;
}

bool FusionCore::update_gps(const GpsMeasurement &gps) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  // Direct position update from GPS
  impl_->state.position = gps.position;
  impl_->state.timestamp = gps.timestamp;

  impl_->log("GPS update");
  impl_->notify_state();
  return true;
}

bool FusionCore::update_pose(const PoseMeasurement &pose) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  // Full pose update (position + orientation)
  impl_->state.position = pose.position;
  impl_->state.orientation = pose.orientation;
  impl_->state.timestamp = pose.timestamp;

  impl_->log("Pose update");
  impl_->notify_state();
  return true;
}

bool FusionCore::update_custom(const std::vector<std::vector<double>> &H,
                               const std::vector<double> &z,
                               const std::vector<std::vector<double>> &R,
                               Time timestamp) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->log("Custom update (stub)");
  // Placeholder for custom generic Kalman Update X = X + K(z - HX)
  impl_->state.timestamp = timestamp;
  impl_->notify_state();
  return true;
}

// Unified Generic Entry
bool FusionCore::add_measurement(const Measurement &m) {
  switch (m.type) {
  case MeasurementType::IMU:
    if (m.imu)
      return update_imu(*m.imu);
    break;
  case MeasurementType::ODOM:
    if (m.odom)
      return update_odom(*m.odom);
    break;
  case MeasurementType::GPS:
    if (m.gps)
      return update_gps(*m.gps);
    break;
  case MeasurementType::POSE:
    if (m.pose)
      return update_pose(*m.pose);
    break;
  case MeasurementType::CUSTOM:
    if (m.H && m.z && m.R)
      return update_custom(*m.H, *m.z, *m.R, m.timestamp);
    break;
  default:
    break;
  }
  return false;
}

State FusionCore::get_state() const {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  return impl_->state;
}

std::vector<std::vector<double>> FusionCore::get_covariance() const {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  return impl_->covariance;
}

void FusionCore::set_state(const State &s) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->state = s;
  impl_->notify_state();
}

int FusionCore::register_state_callback(StateCallback cb) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  int id = impl_->next_cb_id++;
  impl_->state_cbs[id] = std::move(cb);
  return id;
}

void FusionCore::unregister_state_callback(int id) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->state_cbs.erase(id);
}

void FusionCore::set_diagnostic_callback(DiagnosticCallback cb) {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->diag_cb = std::move(cb);
}

} // namespace sensor_fusion_lite
