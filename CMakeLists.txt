cmake_minimum_required(VERSION 3.8)
project(imu_covariance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(imu_covariance_node src/imu_covariance_node.cpp)
ament_target_dependencies(imu_covariance_node rclcpp sensor_msgs)

install(TARGETS
  imu_covariance_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
