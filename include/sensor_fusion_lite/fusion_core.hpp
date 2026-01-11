#pragma once

#include <functional>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <optional>
#include <memory>
#include <array>
#include <cmath>

namespace sensor_fusion_lite
{

  using Time = std::chrono::steady_clock::time_point;

  // ---------------- State Container ----------------

  /**
   * @brief Represents the estimated state of the robot.
   */
  struct State
  {
    std::array<double, 3> position{};    ///< Position (x, y, z) in meters
    std::array<double, 3> velocity{};    ///< Linear velocity (vx, vy, vz) in m/s
    std::array<double, 4> orientation{}; ///< Orientation as a quaternion (x, y, z, w)
    std::vector<std::vector<double>> covariance; ///< State covariance matrix
    Time timestamp{}; ///< Timestamp of the state estimate
  };

  /**
   * @brief Raw IMU measurement data.
   */
  struct ImuMeasurement
  {
    std::array<double, 3> linear_accel{}; ///< Linear acceleration (ax, ay, az) in m/s^2
    std::array<double, 3> angular_vel{};  ///< Angular velocity (gx, gy, gz) in rad/s
    std::array<double, 4> orientation{};  ///< Optional pre-computed orientation (quaternion)
    Time timestamp{}; ///< Timestamp of the measurement
  };

  /**
   * @brief Odometry measurement data.
   */
  struct OdomMeasurement
  {
    std::array<double, 3> position{}; ///< Position (x, y, z)
    std::array<double, 4> orientation{}; ///< Orientation (quaternion)
    std::array<double, 3> linear_velocity{}; ///< Linear velocity (vx, vy, vz)
    std::array<double, 6> cov_diag{}; ///< Diagonal of the covariance matrix
    Time timestamp{}; ///< Timestamp of the measurement
  };

  /**
   * @brief GPS measurement data.
   */
  struct GpsMeasurement
  {
    std::array<double, 3> position{}; ///< Position (lat->x, lon->y, alt->z)
    std::array<std::array<double, 3>, 3> cov{}; ///< 3x3 position covariance
    Time timestamp{}; ///< Timestamp of the measurement
  };

  /**
   * @brief Generic 6DOF pose measurement.
   */
  struct PoseMeasurement
  {
    std::array<double, 3> position{}; ///< Position (x, y, z)
    std::array<double, 4> orientation{}; ///< Orientation (quaternion)
    std::array<double, 6> cov_diag{}; ///< Diagonal of the 6DOF covariance
    Time timestamp{}; ///< Timestamp of the measurement
  };

  /**
   * @brief Enumeration of supported measurement types.
   */
  enum class MeasurementType
  {
    IMU,
    ODOM,
    GPS,
    POSE,
    CUSTOM
  };

  /**
   * @brief Unified measurement container for processing queues.
   */
  struct Measurement
  {
    MeasurementType type; ///< Type of the measurement
    Time timestamp; ///< Common timestamp

    std::optional<ImuMeasurement> imu;
    std::optional<OdomMeasurement> odom;
    std::optional<GpsMeasurement> gps;
    std::optional<PoseMeasurement> pose;

    // Custom Generic Update
    std::optional<std::vector<std::vector<double>>> H; ///< Observation model matrix (Custom)
    std::optional<std::vector<double>> z; ///< Measurement vector (Custom)
    std::optional<std::vector<std::vector<double>>> R; ///< Measurement noise covariance (Custom)
  };

  /**
   * @brief Enumeration of supported filter algorithms.
   */
  enum class FilterType
  {
    COMPLEMENTARY, ///< Simple complementary filter for orientation
    EKF,           ///< Extended Kalman Filter
    UKF            ///< Unscented Kalman Filter
  };

  // ---------------- Callbacks ----------------
  using StateCallback = std::function<void(const State &)>;
  using DiagnosticCallback = std::function<void(const std::string &)>;

  /**
   * @brief Core class for Sensor Fusion Logic.
   *
   * This class manages the state estimation independent of the ROS 2 middleware.
   * It handles state prediction and measurement updates from various sensors.
   */
  class FusionCore
  {
  public:
    /**
     * @brief Construct a new Fusion Core object.
     *
     * @param filter_type Type of filter to use (default: COMPLEMENTARY).
     * @param state_dim Dimension of the state vector (default: 6).
     * @param initial_state Optional initial state pointer.
     */
    FusionCore(FilterType filter_type = FilterType::COMPLEMENTARY,
               int state_dim = 6,
               const State *initial_state = nullptr);

    /**
     * @brief Destroy the Fusion Core object.
     */
    ~FusionCore();

    // lifecycle
    
    /**
     * @brief Initialize internal structures and allocators.
     */
    void initialize();

    /**
     * @brief Start the fusion loop (if running async) or prepare for updates.
     */
    void start();

    /**
     * @brief Stop the fusion process.
     */
    void stop();

    // configuration
    
    /**
     * @brief Set the active filter algorithm.
     * @param t The filter type.
     */
    void set_filter_type(FilterType t);

    /**
     * @brief Get the currently active filter type.
     * @return FilterType
     */
    FilterType get_filter_type() const;

    /**
     * @brief Enable or disable a specific sensor source.
     * @param sensor_name Name of the sensor topic or ID.
     * @param enabled True to enable, false to ignore.
     */
    void enable_sensor(const std::string &sensor_name, bool enabled);

    // main API
    
    /**
     * @brief Perform the prediction step of the filter.
     * @param dt Time delta since the last update in seconds.
     * @return true if successful.
     */
    bool predict(const std::chrono::duration<double> &dt);

    /**
     * @brief Process an IMU measurement.
     * @param imu The IMU data.
     * @return true if update was successful.
     */
    bool update_imu(const ImuMeasurement &imu);

    /**
     * @brief Process an Odometry measurement.
     * @param odom The Odometry data.
     * @return true if update was successful.
     */
    bool update_odom(const OdomMeasurement &odom);

    /**
     * @brief Process a GPS measurement.
     * @param gps The GPS data.
     * @return true if update was successful.
     */
    bool update_gps(const GpsMeasurement &gps);

    /**
     * @brief Process a generic Pose measurement.
     * @param pose The Pose data.
     * @return true if update was successful.
     */
    bool update_pose(const PoseMeasurement &pose);

    /**
     * @brief Process a custom generic measurement update.
     *
     * @param H Observation matrix.
     * @param z Measurement vector.
     * @param R Measurement noise covariance matrix.
     * @param timestamp Time of measurement.
     * @return true if update was successful.
     */
    bool update_custom(const std::vector<std::vector<double>> &H,
                       const std::vector<double> &z,
                       const std::vector<std::vector<double>> &R,
                       Time timestamp);

    // unified generic entry point
    /**
     * @brief Unified entry point for any measurement type.
     * @param m The Generic Measurement container.
     * @return true if successful.
     */
    bool add_measurement(const Measurement &m);

    // accessors
    
    /**
     * @brief Get the current estimated state.
     * @return State The state struct.
     */
    State get_state() const;

    /**
     * @brief Get the current State Covariance.
     * @return 2D vector representing the covariance matrix.
     */
    std::vector<std::vector<double>> get_covariance() const;

    /**
     * @brief Manually set the state.
     * @param s New state.
     */
    void set_state(const State &s);

    // callbacks
    
    /**
     * @brief Register a callback to be notified when state changes.
     * @param cb The callback function.
     * @return int Callback ID for unregistering.
     */
    int register_state_callback(StateCallback cb);

    /**
     * @brief Unregister a state callback.
     * @param id The ID returned by register_state_callback.
     */
    void unregister_state_callback(int id);

    /**
     * @brief Set a diagnostic message callback.
     * @param cb The callback function.
     */
    void set_diagnostic_callback(DiagnosticCallback cb);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

} // namespace sensor_fusion_lite
