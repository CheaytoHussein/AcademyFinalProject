cmake_minimum_required(VERSION 3.8)
project(academy_robot_pointcloud_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(academy_robot_pointcloud_interfaces REQUIRED)

add_executable(depth_to_pointcloud_server src/depth_to_pointcloud_server.cpp)
ament_target_dependencies(depth_to_pointcloud_server rclcpp sensor_msgs academy_robot_pointcloud_interfaces)

install(TARGETS depth_to_pointcloud_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
