cmake_minimum_required(VERSION 3.8)
project(speed_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(speed_gate src/speed_gate.cpp)
target_compile_features(speed_gate PUBLIC cxx_std_17)
ament_target_dependencies(speed_gate rclcpp geometry_msgs std_msgs)

install(TARGETS speed_gate DESTINATION lib/${PROJECT_NAME})

ament_package()
