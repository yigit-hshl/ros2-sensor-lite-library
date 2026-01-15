#include "sensor_fusion_lite/fusion_core.hpp"

// filters
#include "sensor_fusion_lite/filters/base_filter.hpp"
#include "sensor_fusion_lite/filters/complementary_filter.hpp"
#include "sensor_fusion_lite/filters/ekf_filter.hpp"
#include "sensor_fusion_lite/filters/ukf_filter.hpp"

#include <atomic>
#include <iostream>
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
  FilterType filter_type{FilterType::COMPLEMENTARY}; ///< Active filter strategy
  int state_dim{6};        ///< Dimension of the state vector
  bool running{false};     ///< Operational flag
  bool initialized{false}; ///< Filter initialized flag

  State state{};                               ///< Current estimated state
  std::vector<std::vector<double>> covariance; ///< State covariance matrix

  BaseFilterPtr filter; ///< Active filter instance

  std::mutex mtx; ///< Mutex for thread-safety

  std::unordered_map<std::string, bool> sensor_enabled{
      {"imu", true},
      {"odom", true},
      {"gps", true},
      {"pose", true},
  }; ///< Enabled/Disabled status per sensor

  std::unordered_map<int, StateCallback>
      state_callbacks;              ///< Registered state callbacks
  DiagnosticCallback diagnostic_cb; ///< Diagnostic callback
  int next_cb_id{0};                ///< Counter for callback IDs

  /**
   * @brief Helper to triggering all registered state callbacks.
   */
  void notify_state() {
    for (auto &kv : state_callbacks)
      if (kv.second)
        kv.second(state);
  }

  /**
   * @brief Helper for logging diagnostic messages.
   * @param msg The message to log.
   */
  void log(const std::string &msg) {
    if (diagnostic_cb)
      diagnostic_cb(msg);
    else
      std::cout << "[FusionCore] " << msg << std::endl;
  }
};

// ===============================
// FusionCore public API
// ===============================

FusionCore::FusionCore(FilterType filter_type, int state_dim,
                       const State *initial_state)
    : impl_(std::make_unique<Impl>()) {
  impl_->filter_type = filter_type;
  impl_->state_dim = state_dim;
  if (initial_state) {
    impl_->state = *initial_state;
  } else {
    impl_->state.position = {0.0, 0.0, 0.0};
    impl_->state.velocity = {0.0, 0.0, 0.0};
    impl_->state.orientation = {0.0, 0.0, 0.0, 1.0};
    impl_->state.timestamp = std::chrono::steady_clock::now();
  }
}

FusionCore::~FusionCore() = default;

// ===============================
// Lifecycle
// ===============================

void FusionCore::initialize() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->filter = make_filter(impl_->filter_type);
  impl_->initialize(impl_->state, impl_->state_dim);

  impl_->state = impl_->filter->get_state();
  impl_->covariance = impl_->filter->get_covariance();

  impl_->initialized = true;
  impl_->log("FusionCore initialized");
}

void FusionCore::start() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->running = true;
  impl_->log("FusionCore started");
}

void FusionCore::stop() {
  std::lock_guard<std::mutex> lock(impl_->mtx);
  impl_->running = false;
  impl_->log("FusionCore stopped");
}

// ===============================
// Configuration
// ===============================

void FusionCore::set_filter_type(FilterType t) {
  std::scoped_lock lock(impl_->mtx);
  impl_->filter_type = t;

  if (impl_->initialized) {
    impl_->filter = make_filter(t);
    impl_->filter->initialize(impl_->state, impl_->state_dim);
    impl_->log("Filter backend switched");
  }
}

FilterType FusionCore::get_filter_type() const {
  std::scoped_lock lock(impl_->mtx);
  return impl_->filter_type;
}

void FusionCore::enable_sensor(const std::string &sensor_name, bool enabled) {
  std::scoped_lock lock(impl_->mtx);
  impl_->sensor_enabled[sensor_name] = enabled;
}

// ===============================
// Prediction
// ===============================

bool FusionCore::predict(const std::chrono::duration<double> &dt) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->initialized || !impl_->running)
    return false;

  impl_->filter->predict(dt.count());

  impl_->state = impl_->filter->get_state();
  impl_->covariance = impl_->filter->get_covariance();

  // Notify listeners of the predicted state
  impl_->notify_state();
  return true;
}

// ===============================
// Updates (typed)
// ===============================

bool FusionCore::update_imu(const ImuMeasurement &imu) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->sensor_enabled["imu"])
    return false;

  impl_->filter->update_imu(imu);
  impl_->state = impl_->filter->get_state();
  impl_->notify_state();
  return true;
}

bool FusionCore::update_odom(const OdomMeasurement &odom) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->sensor_enabled["odom"])
    return false;

  impl_->filter->update_odom(odom);
  impl_->state = impl_->filter->get_state();
  impl_->notify_state();
  return true;
}

bool FusionCore::update_gps(const GpsMeasurement &gps) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->sensor_enabled["gps"])
    return false;

  impl_->filter->update_gps(gps);
  impl_->state = impl_->filter->get_state();
  impl_->notify_state();
  return true;
}

bool FusionCore::update_pose(const PoseMeasurement &pose) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->sensor_enabled["pose"])
    return false;

  impl_->filter->update_pose(pose);
  impl_->state = impl_->filter->get_state();
  impl_->notify_state();
  return true;
}

bool FusionCore::update_custom(const std::vector<std::vector<double>> &H,
                               const std::vector<double> &z,
                               const std::vector<std::vector<double>> &R,
                               Time timestamp) {
  std::scoped_lock lock(impl_->mtx);
  if (!impl_->sensor_enabled["custom"])
    return false;

  impl_->filter->update_custom(H, z, R, timestamp);
  impl_->state = impl_->filter->get_state();
  impl_->notify_state();
  return true;
}

// ===============================
// Generic measurement entry
// ===============================

bool FusionCore::add_measurement(const Measurement &m) {
  switch (m.type) {
  case MeasurementType::IMU:
    return m.imu ? update_imu(*m.imu) : false;
  case MeasurementType::ODOM:
    return m.odom ? update_odom(*m.odom) : false;
  case MeasurementType::GPS:
    return m.gps ? update_gps(*m.gps) : false;
  case MeasurementType::POSE:
    return m.pose ? update_pose(*m.pose) : false;
  case MeasurementType::CUSTOM:
    return m.H && m.z && m.R ? update_custom(*m.H, *m.z, *m.R, m.timestamp)
                             : false;
  default:
    return false;
  }
}

// ===============================
// Accessors
// ===============================

State FusionCore::get_state() const {
  std::scoped_lock lock(impl_->mtx);
  return impl_->state;
}

std::vector<std::vector<double>> FusionCore::get_covariance() const {
  std::scoped_lock lock(impl_->mtx);
  return impl_->covariance;
}

void FusionCore::set_state(const State &s) {
  std::scoped_lock lock(impl_->mtx);
  impl_->state = s;
  if (impl_->filter) {
    impl_->filter->set_state(s);
  }
}

// ===============================
// Callbacks
// ===============================

int FusionCore::register_state_callback(StateCallback cb) {
  std::scoped_lock lock(impl_->mtx);
  int id = impl_->next_cb_id++;
  impl_->state_callbacks[id] = cb;
  return id;
}

void FusionCore::unregister_state_callback(int id) {
  std::scoped_lock lock(impl_->mtx);
  impl_->state_callbacks.erase(id);
}

void FusionCore::set_diagnostic_callback(DiagnosticCallback cb) {
  std::scoped_lock lock(impl_->mtx);
  impl_->diagnostic_cb = cb;
}
} // namespace sensor_fusion_lite
