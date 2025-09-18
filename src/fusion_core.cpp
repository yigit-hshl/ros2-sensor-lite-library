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
  
} // namespace sensor_fusion_lite