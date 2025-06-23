cmake_minimum_required(VERSION 3.8)
project(rover_mavlink)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
list(APPEND CMAKE_PREFIX_PATH "/usr/local")



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(mavsdk REQUIRED)


add_executable(mavlink_bridge src/mavlink_bridge.cpp)
ament_target_dependencies(mavlink_bridge
  rclcpp
  mavros_msgs
)
target_include_directories(mavlink_bridge PRIVATE ${mavsdk_INCLUDE_DIRS})
target_link_libraries(mavlink_bridge mavsdk::mavsdk)

add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  geometry_msgs
  mavros_msgs
)
target_link_libraries(navigation_node
${GeographicLib_LIBRARIES}
)

install(TARGETS mavlink_bridge navigation_node 
  DESTINATION lib/${PROJECT_NAME})