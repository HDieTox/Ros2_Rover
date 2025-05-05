cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Trouver la bibliothèque système serial (libserial)
find_library(SERIAL_LIB libserial)

# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable : pwm_decoder
add_executable(pwm_decoder src/pwm_decoder.cpp)
ament_target_dependencies(pwm_decoder rclcpp std_msgs geometry_msgs)
target_link_libraries(pwm_decoder ${PIGPIO_LIBRARY})

# Executable : ppm_converter
add_executable(ppm_converter src/ppm_converter.cpp)
ament_target_dependencies(ppm_converter rclcpp std_msgs geometry_msgs)

# Executable : motor_controller
add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${SERIAL_LIB})

# Installation des exécutables
install(TARGETS
  pwm_decoder
  ppm_converter
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Installation des fichiers de configuration (si tu as un dossier config)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_package()
