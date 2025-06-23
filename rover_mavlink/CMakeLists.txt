cmake_minimum_required(VERSION 3.8)
project(mavlink_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mavlink_bridge src/mavlink_bridge.cpp)
target_include_directories(mavlink_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(mavlink_bridge rclcpp)

install(TARGETS mavlink_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()