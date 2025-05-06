cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/aarch64-linux-gnu)

# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(ppm_converter src/ppm_converter.cpp)
target_include_directories(ppm_converter PRIVATE /usr/include/libserial)
target_link_libraries(ppm_converter ${LIBSERIAL_LIB})
ament_target_dependencies(ppm_converter rclcpp std_msgs geometry_msgs)

# Installation des exécutables C++
install(TARGETS
  ppm_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Python ==========================================
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pwm_decoder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
