cmake_minimum_required(VERSION 3.8)
project(workshop_main)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(workshop_interfaces REQUIRED)

include_directories(include)

add_executable(minimal_server src/minimal_server.cpp)
ament_target_dependencies(minimal_server rclcpp workshop_interfaces)

add_executable(minimal_client src/minimal_client.cpp)
ament_target_dependencies(minimal_client rclcpp workshop_interfaces)

install(TARGETS
  minimal_server
  minimal_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()