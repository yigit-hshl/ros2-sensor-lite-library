#include "sensor_fusion_lite/fusion_core.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <map>

namespace sensor_fusion_lite {
  // ----- Internal Implementation -----
  struct FusionCore::Impl {
    FilterType filter_type;
    int state_dim;
    State state;
    bool initialized{false};
    bool running{false};

    std::mutex mtx;

    // Covariances
    std::vector<double> imu_accel_cov;
    std::vector<double> imu_gyro_cov;
    std::vector<double> odom_cov_diag;
    std::vector<double> gps_cov;
    std::vector<double> pose_cov_diag;

    // Sensor enable map
    std::map<std::string, bool> sensor_enabled;

    // Callbacks
    std::map<int, StateCallback> state_callbacks;
    DiagnosticCallback diag_cb;
    int callback_id_counter{0};

    Impl(FilterType ft, int dim) : filter_type(ft), state_dim(dim) {
      sensor_enabled["imu"] = true;
      sensor_enabled["odom"] = true;
      sensor_enabled["gps"] = true;
      sensor_enabled["pose"] = true;
    }

    void notify_state_update() {
      for (auto &kv : state_callbacks) {
        kv.second(state);
      }
    }

    void notify_diag(const std::string &msg) {
      if (diag_cb) {
        diag_cb(msg);
      }
    }
  };

  // ----- Public API -----
  FusionCore::FusionCore(FilterType filter_type, int state_dim, const State* initial_state) : impl_(std::make_unique<Impl>(filter_type, state_dim)) {
    if (initial_state) {
      impl_->state = *initial_state;
    }
  }

  FusionCore::~FusionCore() = default;

  void FusionCore::initialize() {
    std::scoped_lock lock(impl_->mtx);
    impl_->initialized = true;
    impl_->notify_diag("FusionCore initialized.");
  }

  void FusionCore::start() {
    std::scoped_lock lock(impl_->mtx);
    impl_->running = true;
    impl_->notify_diag("FusionCore started.");
  }

  void FusionCore::stop() {
    std::scoped_lock lock(impl_->mtx);
    impl_->running = false;
    impl_->notify_diag("FusionCore stopped.");
  }

  void FusionCore::set_filter_type(FilterType t) {
    std::scoped_lock lock(impl_->mtx);
    impl_->filter_type = t;
  }

  FilterType FusionCore::get_filter_type() const {
    std::scoped_lock lock(impl_->mtx);
    return impl_->filter_type;
  }

  // ----- Covariance setters (just store diagonals as vectors) -----
  void FusionCore::set_imu_noise(const std::vector<double>& accel_cov, const std::vector<double>& gyro_cov) {
    std::scoped_lock lock(impl_->mtx);
    impl_->imu_accel_cov = accel_cov;
    impl_->imu_gyro_cov = gyro_cov;
  }

  void FusionCore::set_odom_covariance_diag(const std::vector<double>& diag) {
    std::scoped_lock lock(impl_->mtx);
    impl_->odom_cov_diag = diag;
  }

  void FusionCore::set_gps_covariance(const std::vector<double>& cov) {
    std::scoped_lock lock(impl_->mtx);
    impl_->gps_cov = cov;
  }

  void FusionCore::set_pose_cov_diag(const std::vector<double>& diag) {
    std::scoped_lock lock(impl_->mtx);
    impl_->pose_cov_diag = diag;
  }

  void FusionCore::enable_sensor(const std::string& sensor_name, bool enabled) {
    std::scoped_lock lock(impl_->mtx);
    impl_->sensor_enabled[sensor_name] = enabled;
  }

  // ----- Measurement updates (stubs, fusion logic) -----
  bool FusionCore::predict(const std::chrono::duration<double>& dt) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized) return false;

    double dt_sec = dt.count();
    for (int i = 0; i < 3; ++i) {
      impl_->state.position[i] += impl_->state.velocity[i] * dt_sec;
    }
    impl_->state.timestamp += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt);

    impl_->notify_state_update();
    return true;
  }

  bool FusionCore::update_imu(const ImuMeasurement& imu) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized || !impl_->sensor_enabled["imu"]) return false;

    // Simplified: integrate accel into velocity
    for (int i = 0; i < 3; ++i) {
      impl_->state.velocity[i] += imu.linear_accel[i] * 0.01; // assume dt=0.01
    }
    impl_->state.timestamp = imu.timestamp;

    impl_->notify_state_update();
    return true;
  }

  bool FusionCore::update_odom(const OdomMeasurement& odom) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized || !impl_->sensor_enabled["odom"]) return false;

    impl_->state.position = odom.position;
    impl_->state.velocity = odom.linear_velocity;
    impl_->state.timestamp = odom.timestamp;

    impl_->notify_state_update();
    return true;
  }

  bool FusionCore::update_gps(const GpsMeasurement& gps) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized || !impl_->sensor_enabled["gps"]) return false;

    impl_->state.position = gps.position;
    impl_->state.timestamp = gps.timestamp;

    impl_->notify_state_update();
    return true;
  }

  bool FusionCore::update_pose(const PoseMeasurement& pose) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized || !impl_->sensor_enabled["pose"]) return false;

    impl_->state.position = pose.position;
    impl_->state.timestamp = pose.timestamp;

    impl_->notify_state_update();
    return true;
  }

  bool FusionCore::update_custom(const std::vector<std::vector<double>>& H, const std::vector<double>& z, const std::vector<std::vector<double>>& R, Time timestamp) {
    std::scoped_lock lock(impl_->mtx);
    if (!impl_->initialized) return false;

    // Placeholder: just update timestamp
    impl_->state.timestamp = timestamp;
    impl_->notify_state_update();
    return true;
  }

  // ----- Accessors -----
  State FusionCore::get_state() const {
    std::scoped_lock lock(impl_->mtx);
    return impl_->state;
  }

  std::vector<std::vector<double>> FusionCore::get_covariance() const {
    std::scoped_lock lock(impl_->mtx);
    return {{1.0,0.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0}}; // placeholder identity
  }

  void FusionCore::set_state(const State& s) {
    std::scoped_lock lock(impl_->mtx);
    impl_->state = s;
    impl_->notify_state_update();
  }

  // ----- Callbacks -----
} // namespace sensor_fusion_lite