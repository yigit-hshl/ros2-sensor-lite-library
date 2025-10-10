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

namespace sensor_fusion_lite {

using Time = std::chrono::steady_clock::time_point;

// ---------------- State Container ----------------
struct State {
    std::array<double, 3> position{};    // px, py, pz
    std::array<double, 3> velocity{};    // vx, vy, vz
    std::array<double, 4> orientation{}; // quaternion (x, y, z, w)
    std::vector<std::vector<double>> covariance;
    Time timestamp{};
};

// ---------------- Sensor Measurements ----------------
struct ImuMeasurement {
    std::array<double, 3> linear_accel{};
    std::array<double, 3> angular_vel{};
    std::array<double, 4> orientation{}; // optional
    Time timestamp{};
};

struct OdomMeasurement {
    std::array<double, 3> position{};
    std::array<double, 4> orientation{};
    std::array<double, 3> linear_velocity{};
    std::array<double, 6> cov_diag{};
    Time timestamp{};
};

struct GpsMeasurement {
    std::array<double, 3> position{};
    std::array<std::array<double, 3>, 3> cov{};
    Time timestamp{};
};

struct PoseMeasurement {
    std::array<double, 3> position{};
    std::array<double, 4> orientation{};
    std::array<double, 6> cov_diag{};
    Time timestamp{};
};

// ---------------- Measurement Type ----------------
enum class MeasurementType { IMU, ODOM, GPS, POSE, CUSTOM };

struct Measurement {
    MeasurementType type;
    Time timestamp;

    std::optional<ImuMeasurement> imu;
    std::optional<OdomMeasurement> odom;
    std::optional<GpsMeasurement> gps;
    std::optional<PoseMeasurement> pose;

    std::optional<std::vector<std::vector<double>>> H;
    std::optional<std::vector<double>> z;
    std::optional<std::vector<std::vector<double>>> R;
};

// ---------------- Filter Type ----------------
enum class FilterType { COMPLEMENTARY, EKF, UKF };

// ---------------- Callbacks ----------------
using StateCallback = std::function<void(const State&)>;
using DiagnosticCallback = std::function<void(const std::string&)>;

// ---------------- FusionCore ----------------
class FusionCore {
public:
    FusionCore(FilterType filter_type = FilterType::COMPLEMENTARY,
               int state_dim = 6,
               const State* initial_state = nullptr);

    ~FusionCore();

    // lifecycle
    void initialize();
    void start();
    void stop();

    // configuration
    void set_filter_type(FilterType t);
    FilterType get_filter_type() const;

    void enable_sensor(const std::string& sensor_name, bool enabled);

    // main API
    bool predict(const std::chrono::duration<double>& dt);
    bool update_imu(const ImuMeasurement& imu);
    bool update_odom(const OdomMeasurement& odom);
    bool update_gps(const GpsMeasurement& gps);
    bool update_pose(const PoseMeasurement& pose);
    bool update_custom(const std::vector<std::vector<double>>& H,
                       const std::vector<double>& z,
                       const std::vector<std::vector<double>>& R,
                       Time timestamp);

    // unified generic entry point
    bool add_measurement(const Measurement& m);

    // accessors
    State get_state() const;
    std::vector<std::vector<double>> get_covariance() const;
    void set_state(const State& s);

    // callbacks
    int register_state_callback(StateCallback cb);
    void unregister_state_callback(int id);
    void set_diagnostic_callback(DiagnosticCallback cb);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sensor_fusion_lite
