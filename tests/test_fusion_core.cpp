#include "sensor_fusion_lite/fusion_core.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <thread>

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

  // ----- Pose update -----
  PoseMeasurement pose{};
  pose.position = {20.0, 2.0, 0.0};
  pose.timestamp = std::chrono::steady_clock::now();
  ok = core.update_pose(pose);
  assert(ok);
  State s4 = core.get_state();
  assert(s4.position[0] == 20.0 && s4.position[1] == 2.0);
  std::cout << "Pose update test passed.\n";
  
  // ----- Callback fired -----
  assert(callback_called);
  std::cout << "Callback test passed.\n";

  FusionCore fusion;

  auto now = std::chrono::steady_clock::now();

  // Test 1: Initial state should be empty
  auto state_opt = fusion.get_state();
  assert(!state_opt.has_value());

  // Test 2: Add one measurement, check whether state exists
  fusion.add_measurement(now, {1.0, 0.0, 0.0});
  state_opt = fusion.get_state();
  assert(state_opt.has_value());
  assert(state_opt->estimate.size() == 3);

  // Test 3: Add multiple measurements and check averaging
  fusion.add_measurement(now + std::chrono::milliseconds(10), {3.0, 0.0, 0.0});
  state_opt = fusion.get_state();
  assert(state_opt.has_value());
  // Expected average: (1 + 3) / 2 = 2
  assert(state_opt->estimate[0] == 2.0);

  // Test 4: Reset clears state
  fusion.reset();
  state_opt = fusion.get_state();
  assert(!state_opt.has_value());

  std::cout << "[OK] All FusionCore unit tests passed.\n";
  return 0;
}