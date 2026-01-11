cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_lite)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Core library
add_library(fusion_core
  src/fusion_core.cpp
)

target_include_directories(fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fusion_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Node executable
add_executable(fusion_node
  src/fusion_node.cpp
)

target_link_libraries(fusion_node fusion_core)

ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Install targets
install(TARGETS fusion_core fusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})


# Unit test
add_executable(test_fusion_core
  test/test_fusion_core.cpp
)
target_link_libraries(test_fusion_core fusion_core)
ament_target_dependencies(test_fusion_core rclcpp)

install(TARGETS test_fusion_core
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(NAME test_fusion_core
    COMMAND test_fusion_core)
endif()

# Ament package macros
ament_package()