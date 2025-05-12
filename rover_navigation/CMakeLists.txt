cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/aarch64-linux-gnu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

add_executable(nmea_parser src/nmea_parser.cpp)
target_link_libraries(nmea_parser ${rclcpp_LIBRARIES})

add_executable(serial_nmea_publisher src/serial_nmea_publisher.cpp)
target_link_libraries(serial_nmea_publisher ${rclcpp_LIBRARIES} ${LIBSERIAL_LIB})

install(TARGETS
  nmea_parser
  serial_nmea_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
