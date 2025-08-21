cmake_minimum_required(VERSION 3.8)
project(academy_robot_approach_screw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApproachScrew.action"
  DEPENDENCIES builtin_interfaces
)

# Export runtime so others (and this package) can use the generated code
ament_export_dependencies(rosidl_default_runtime)

add_executable(approach_screw_server src/approach_screw_server.cpp)
ament_target_dependencies(approach_screw_server rclcpp rclcpp_action sensor_msgs geometry_msgs)

# Ensure headers are generated before compiling this target
add_dependencies(approach_screw_server ${PROJECT_NAME}__rosidl_generator_cpp)

# Get and link the C++ typesupport for the action
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(approach_screw_server ${cpp_typesupport_target})

# Make the generated headers visible to this target
target_include_directories(approach_screw_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

install(TARGETS approach_screw_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
