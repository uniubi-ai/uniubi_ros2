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
  "msg/GpsObserved.msg"
  "msg/UwbObserved.msg"
  "srv/JsonCommand.srv"
  "srv/StartMotionAction.srv"
  DEPENDENCIES builtin_interfaces uniubi
)

add_executable(uniubi_motion_bridge_node
  src/uniubi_motion_bridge_node.cpp
)
target_compile_features(uniubi_motion_bridge_node PUBLIC cxx_std_17)
target_include_directories(uniubi_motion_bridge_node PRIVATE include)
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

if(BUILD_TESTING)
  add_executable(battery_status_test test/battery_status_test.cpp)
  target_include_directories(battery_status_test PRIVATE include)
  ament_target_dependencies(battery_status_test sensor_msgs uniubi_motion_client)
  add_test(NAME battery_status_test COMMAND battery_status_test)
  add_executable(frame_id_test test/frame_id_test.cpp)
  target_include_directories(frame_id_test PRIVATE include)
  add_test(NAME frame_id_test COMMAND frame_id_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME namespace_graph_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/namespace_graph_test.py)
  set_tests_properties(namespace_graph_test PROPERTIES TIMEOUT 45
    ENVIRONMENT "BRIDGE_CONFIG_FILE=${CMAKE_CURRENT_SOURCE_DIR}/config/motion_bridge.yaml;ROS_DOMAIN_ID=175;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  add_test(NAME cere_battery_async_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/cere_battery_async_test.py)
  set_tests_properties(cere_battery_async_test PROPERTIES TIMEOUT 30
    ENVIRONMENT "BRIDGE_NODE_EXECUTABLE=$<TARGET_FILE:uniubi_motion_bridge_node>;ROS_DOMAIN_ID=172;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  add_test(NAME async_motion_status_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/async_motion_status_test.py)
  set_tests_properties(async_motion_status_test PROPERTIES TIMEOUT 35
    ENVIRONMENT "BRIDGE_NODE_EXECUTABLE=$<TARGET_FILE:uniubi_motion_bridge_node>;ROS_DOMAIN_ID=174;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
