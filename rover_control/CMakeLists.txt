cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

target_link_libraries(ppm_converter serial)
target_include_directories(ppm_converter PRIVATE /usr/local/include)

# C++ ============================================
# Executable : ppm_converter
add_executable(ppm_converter src/ppm_converter.cpp)
ament_target_dependencies(ppm_converter rclcpp std_msgs geometry_msgs)

# Installation des exécutables C++
install(TARGETS
  ppm_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Python ==========================================
ament_python_install_package(${PROJECT_NAME})

# Installation des fichiers Python
install(PROGRAMS
  scripts/pwm_decoder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
