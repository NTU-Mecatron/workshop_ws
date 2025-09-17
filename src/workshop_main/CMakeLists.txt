cmake_minimum_required(VERSION 3.8)
project(workshop_main)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(workshop_interfaces REQUIRED)

add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp workshop_interfaces)

add_executable(minimal_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(minimal_subscriber rclcpp workshop_interfaces)

add_executable(minimal_server src/minimal_server.cpp)
ament_target_dependencies(minimal_server rclcpp workshop_interfaces)

add_executable(minimal_client src/minimal_client.cpp)
ament_target_dependencies(minimal_client rclcpp workshop_interfaces)

install(TARGETS
  minimal_publisher
  minimal_subscriber
  minimal_server
  minimal_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()