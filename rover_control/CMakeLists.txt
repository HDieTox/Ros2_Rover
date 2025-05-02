cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Recherche manuelle de pigpio
find_path(PIGPIO_INCLUDE_DIR pigpiod_if2.h)
find_library(PIGPIO_LIBRARY pigpiod_if2)

if(NOT PIGPIO_INCLUDE_DIR)
  message(FATAL_ERROR "pigpio include directory not found")
endif()

if(NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpio library not found")
endif()

include_directories(${PIGPIO_INCLUDE_DIR})

# Executable : pwm_decoder
add_executable(pwm_decoder src/pwm_decoder.cpp)
ament_target_dependencies(pwm_decoder rclcpp std_msgs geometry_msgs)
target_link_libraries(pwm_decoder ${PIGPIO_LIBRARY})

# Executable : ppm_converter
add_executable(ppm_converter src/ppm_converter.cpp)
ament_target_dependencies(ppm_converter rclcpp std_msgs geometry_msgs)

# Executable : motor_controller
add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp std_msgs geometry_msgs)
target_link_libraries(motor_controller ${PIGPIO_LIBRARY} rclcpp::rclcpp)

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
