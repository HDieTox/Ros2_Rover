cmake_minimum_required(VERSION 3.8)
project(robot_driver)

# Dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Trouver pigpio
find_library(PIGPIO_LIB pigpiod_if2)

# Node PPM Decoder
add_executable(ppm_decoder src/ppm_decoder.cpp)
target_link_libraries(ppm_decoder ${PIGPIO_LIB} ${rclcpp_LIBRARIES})

# Node PPM Converter
add_executable(ppm_converter src/ppm_converter.cpp)
target_link_libraries(ppm_converter ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})

# Node Motor Controller
add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

# Installation
install(TARGETS
  ppm_decoder
  ppm_converter
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
