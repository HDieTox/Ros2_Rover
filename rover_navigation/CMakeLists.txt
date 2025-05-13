cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/aarch64-linux-gnu)


add_library(${MINMEA_LIB} STATIC src/minmea.c)
target_include_directories(minmea PUBLIC include/rover_navigation)


find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)           # Ajouté explicitement
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(nmea_parser src/nmea_parser.cpp)

# Lier les dépendances ROS 2 via ament_target_dependencies
ament_target_dependencies(nmea_parser
  rcl
  rclcpp
  std_msgs
  sensor_msgs
  nmea_msgs
  geographic_msgs
)


target_link_libraries(nmea_parser 
  ${MINMEA_LIB}
)


# Lier les bibliothèques externes

add_executable(serial_nmea_publisher src/serial_nmea_publisher.cpp)

ament_target_dependencies(serial_nmea_publisher
  rclcpp
  std_msgs
)

target_link_libraries(serial_nmea_publisher
  ${LIBSERIAL_LIB}
)

install(TARGETS
  nmea_parser
  serial_nmea_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
