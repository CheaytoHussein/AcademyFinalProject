cmake_minimum_required(VERSION 3.8)
project(academy_robot_pick_place)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Dependencies on our own packages (interfaces + action)
find_package(academy_robot_approach_screw REQUIRED)
find_package(academy_robot_pose_interfaces REQUIRED)

add_executable(pick_place_orchestrator src/pick_place_orchestrator.cpp)
ament_target_dependencies(pick_place_orchestrator
  rclcpp rclcpp_action geometry_msgs tf2_geometry_msgs
  moveit_ros_planning_interface
  academy_robot_approach_screw academy_robot_pose_interfaces)

install(TARGETS pick_place_orchestrator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
