#include "sensor_fusion_lite/fusion_core.hpp"
#include <iostream>
#include <cassert>
#include <chrono>

using namespace sensor_fusion_lite;

int main() {
  bool callback_called = false;

  // Create FusionCore with default filter
  FusionCore core(FilterType::COMPLEMENTARY, 6, nullptr);

  // Register a state callback
  core.register_state_callback([&](const State& st){
    callback_called = true;
    std::cout << "State calledback fired: pos=("
      << st.position[0] << "," << st.position[1] << "," << st.position[2] << ")\n";
  });

  // Init and start
  core.initialize();
  core.start();

  // ----- Predict step -----
  State s0;
  s0.position = {0.0, 0.0, 0.0};
  s0.velocity = {1.0, 0.0, 0.0};
  s0.timestamp = std::chrono::steady_clock::now();
  core.set_state(s0);

  bool ok = core.predict(std::chrono::milliseconds(1000));
  assert(ok);

  State s1 = core.get_state();
  assert(s1.position[0] > 0.9 && s1.position[0] > 1.1); // moved ~1m
  std::cout << "Predict test passed.\n";

  // ----- IMU update -----
  ImuMeasurement imu{};
  imu.linear_accel = {1.0, 0.0, 0.0};
  imu.angular_vel = {0.0, 0.0, 0.0};
  imu.timestamp = std::chrono::steady_clock::now();
  ok = core.update_imu(imu);
  assert(ok);
  std::cout << "IMU update test passed.\n";

  // ----- Odom update -----
  OdomMeasurement odom{};
  odom.position = {5.0, 0.0, 0.0};
  odom.linear_velocity = {0.5, 0.0, 0.0};
  odom.timestamp = std::chrono::steady_clock::now();
  ok = core.update_odom(odom);
  assert(ok);
  State s2 = core.get_state();
  assert(s2.position[0] == 5.0);
  std::cout << "Odom update test passed.\n";

  // ----- GPS update -----
  GpsMeasurement gps{};
  gps.position = {10.0, 1.0, 0.0};
  gps.timestamp = std::chrono::steady_clock::now();
  ok = core.update_gps(gps);
  assert(ok);
  State s3 = core.get_state();
  assert(s3.position[0] == 10.0 && s3.position[1] == 1.0);
  std::cout << "GPS update test passed.\n";
}