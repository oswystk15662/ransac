cmake_minimum_required(VERSION 3.8)
project(ransac_tester_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_executable(ransac_tester_cpp_node src/tester_node.cpp)
install(TARGETS
  ransac_tester_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
