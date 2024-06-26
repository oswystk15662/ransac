cmake_minimum_required(VERSION 3.8)
project(ransac_osw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(sick_scan_xd REQUIRED)

add_executable(ransac_osw_node src/ransac_osw.cpp)
ament_target_dependencies(ransac_osw_node rclcpp sensor_msgs sick_scan_xd)

install(TARGETS
  ransac_osw_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
