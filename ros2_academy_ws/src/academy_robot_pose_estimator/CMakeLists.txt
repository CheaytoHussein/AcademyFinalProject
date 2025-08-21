cmake_minimum_required(VERSION 3.8)
project(academy_robot_pose_estimator)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(academy_robot_pointcloud_interfaces REQUIRED)
find_package(academy_robot_pose_interfaces REQUIRED)

add_executable(pose_estimator
  src/pose_estimator_node.cpp
  src/registration.cpp
)
add_executable(screw_pose_service src/screw_pose_service.cpp)

ament_target_dependencies(pose_estimator
  rclcpp
  sensor_msgs
  pcl_conversions
  academy_robot_pointcloud_interfaces
  academy_robot_pose_interfaces
)

target_include_directories(pose_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(pose_estimator PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pose_estimator ${PCL_LIBRARIES})
target_compile_definitions(pose_estimator PRIVATE ${PCL_DEFINITIONS})

ament_target_dependencies(screw_pose_service rclcpp geometry_msgs academy_robot_pose_interfaces academy_robot_pose_interfaces)

install(TARGETS screw_pose_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS pose_estimator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
