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
    std::array<double,3> position{0,0,0};
    std::array<double,3> velocity{0,0,0};
    Quaternion orientation{};
    std::vector<std::vector<double>> covariance; // NxN covariance
    Time timestamp{};
  };

  // Sensor measurement wrappers
  struct ImuMeasurement {
    std::array<double,3> linear_accel{0,0,0};
    std::array<double,3> angular_vel{0,0,0};
    std::optional<Quaternion> orientation;
    Time timestamp{};
  };

  struct OdomMeasurement {
    std::array<double,3> position{0,0,0};
    std::array<double,3> linear_velocity{0,0,0};
    std::array<double,6> cov_diag{};
    Quaternion orientation{};
    Time timestamp{};
  };

  struct GpsMeasurement {
    std::array<double,3> position{0,0,0};
    std::array<std::array<double,3>,3> cov{};
    Time timestamp{};
  };

  struct PoseMeasurement {
    std::array<double,3> position{0,0,0};
    std::array<double,6> cov_diag{};
    Quaternion orientation{};
    Time timestamp{};
  };
} // namespace sensor_fusion_lite