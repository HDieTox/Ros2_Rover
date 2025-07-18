cmake_minimum_required(VERSION 3.8)
project(rover_autonomous)

# Configuration standard ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/aarch64-linux-gnu)


# Déclaration des exécutables
add_executable(main_controller src/main_controller.cpp src/mission_parser.cpp)
target_include_directories(main_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(main_controller 
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros nlohmann_json sensor_msgs
)

add_executable(serial_publisher 
  src/serial_publisher.cpp
)
ament_target_dependencies(serial_publisher 
  rclcpp geometry_msgs
)

target_include_directories(serial_publisher PRIVATE /usr/include/libserial)
target_link_libraries(serial_publisher ${LIBSERIAL_LIB})

# Installation
install(TARGETS
  main_controller
  serial_publisher
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