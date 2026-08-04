cmake_minimum_required(VERSION 3.8)
project(uniubi_motion_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uniubi REQUIRED)
find_package(uniubi_motion_client REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionStatus.msg"
  "srv/StartMotionAction.srv"
  DEPENDENCIES builtin_interfaces
)

add_executable(uniubi_motion_bridge_node
  src/uniubi_motion_bridge_node.cpp
)
target_compile_features(uniubi_motion_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(uniubi_motion_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  uniubi
  uniubi_motion_client
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(uniubi_motion_bridge_node "${cpp_typesupport_target}")

install(TARGETS uniubi_motion_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
