#pragma once

#include <functional>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <optional>
#include <array>
#include <memory>

namespace sensor_fusion_lite {
  using Time = std::chrono::steady_clock::time_point;

  // Minimal quaternion type
  struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
  };

  // Simple state container
  struct State {
    std::array<double,3> position{0,0,0}; // px,py,pz
    std::array<double,3> velocity{0,0,0}; // vx,vy,vz
    Quaternion orientation{}; // optional (unit quaternion)
    std::vector<std::vector<double>> covariance; // NxN covariance corresponding to chosen state_dim
    Time timestamp{};
  };

  // Sensor measurement wrappers (lightweight)
  struct ImuMeasurement {
    std::array<double,3> linear_accel{0,0,0};
    std::array<double,3> angular_vel{0,0,0};
    std::optional<Quaternion> orientation; // Optional from IMU
    Time timestamp{};
  };

  struct OdomMeasurement {
    std::array<double,3> position{0,0,0};
    std::array<double,3> linear_velocity{0,0,0};
    std::array<double,6> cov_diag{};// pose+orient diag for convenience
    Quaternion orientation{};
    Time timestamp{};
  };

  struct GpsMeasurement {
    std::array<double,3> position{0,0,0}; // lat/long translated to ENU or pre-converted XY
    std::array<std::array<double,3>,3> cov{};
    Time timestamp{};
  };

  struct PoseMeasurement {
    std::array<double,3> position{0,0,0};
    std::array<double,6> cov_diag{};
    Quaternion orientation{};
    Time timestamp{};
  };

  // Filter types
  enum class FilterType { COMPLEMENTARY, EKF, UKF };

  // Callback types
  using StateCallback = std::function<void(const State&)>;
  using DiagnosticCallback = std::function<void(const std::string&)>;

  class FusionCore {
  public:
    // Specify filter type and state dimension; optionally initial state
    FusionCore(FilterType filter_type = FilterType::COMPLEMENTARY, int state_dim = 6, const State* initial_state = nullptr);
    ~FusionCore();

    // ----- Lifecycle -----
    // Must be called before using (allocates internal matrices)
    void initialize();

    // Thread-safe: start/stop internal timers (if used)
    void start();
    void stop();

    // ----- Configuration (thread-safe) -----
    void set_filter_type(FilterType t);
    FilterType get_filter_type() const;

    // Set per-sensor noise/covariance (matrix sizes depend on sensor)
    void set_imu_noise(const std::array<std::array<double,3>,3>& accle_cov, const std::array<std::array<double,3>,3>& gyro_cov);
    void set_odom_covariance_diag(const std::array<double,6>& diag);
    void set_gps_covariance(const std::array<std::array<double,3>,3>& cov);
    void set_pose_cov_diag(const std::array<double,6>& diag);

    // Enable/disable sensors at runtime
    void enable_sensor(const std::string& sensor_name, bool enabled);

    // ----- Main API: measurements -----
    // Each of these is thread-safe and returns true if the update was applied
    bool predict(const std::chrono::duration<double>& dt); // Optional explicit predict
    bool update_imu(const ImuMeasurement& imu);
    bool update_odom(const OdomMeasurement& odom);
    bool update_gps(const GpsMeasurement& gps);
    bool update_pose(const PoseMeasurement& pose);

    // Apply a custom measurement: user provides H, z, R (linear Kalman update)
    bool update_custom(const std::vector<std::vector<double>>& H, const std::vector<double>& z, const std::vector<std::vector<double>>& R, Time timestamp);

    // ----- Accessors -----
    State get_state() const; // Returns copy; thread-safe
    std::vector<std::vector<double>> get_covariance() const;

    // Set state directly (careful: use for initialization or reset)
    void set_state(const State& s);

    // ----- Callbacks -----
    // Register a callback to be invoked whenever fused state is updated
    // Returns an id token for deregistration (simple integer)
    int register_state_callback(StateCallback cb);
    void unregister_state_callback(int id);

    // Diagnostics callback
    void set_diagnostic_callback(DiagnosticCallback cb);

  private:
    // Internal helpers (private)
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };
} // namespace sensor_fusion_lite