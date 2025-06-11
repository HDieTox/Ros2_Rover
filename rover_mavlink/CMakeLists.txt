cmake_minimum_required(VERSION 3.16)
project(rover_autonomy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Installation de MAVSDK
include(FetchContent)
FetchContent_Declare(
  mavsdk
  GIT_REPOSITORY https://github.com/mavlink/MAVSDK.git
  GIT_TAG v2.0.0
)
FetchContent_MakeAvailable(mavsdk)

add_executable(mavlink_bridge src/mavlink_bridge.cpp)
target_link_libraries(mavlink_bridge ${rclcpp_LIBRARIES} mavsdk::mavsdk)

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node ${rclcpp_LIBRARIES} GeographicLib::GeographicLib)

install(TARGETS mavlink_bridge navigation_node serial_bridge
  DESTINATION lib/${PROJECT_NAME})