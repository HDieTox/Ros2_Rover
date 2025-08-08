cmake_minimum_required(VERSION 3.8)
project(rover_autonomous)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/aarch64-linux-gnu)


# Déclaration des exécutables
add_executable(nav_controller src/nav_controller.cpp)
ament_target_dependencies(nav_controller 
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs nlohmann_json sensor_msgs
)

add_executable(serial_stm src/serial_stm.cpp)
ament_target_dependencies(serial_stm 
  rclcpp geometry_msgs sensor_msgs
)

target_include_directories(serial_stm PRIVATE /usr/include/libserial)
target_link_libraries(serial_stm ${LIBSERIAL_LIB})

# Installation
install(TARGETS
  nav_controller
  serial_stm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()