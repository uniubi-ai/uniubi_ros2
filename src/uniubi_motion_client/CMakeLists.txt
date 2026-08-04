cmake_minimum_required(VERSION 3.8)
project(uniubi_motion_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uniubi REQUIRED)

add_library(motion_high_level_client
  src/system_rpc_client_base.cpp
  src/motion_high_level_client.cpp
  third_party/jsoncpp/src/lib_json/json_reader.cpp
  third_party/jsoncpp/src/lib_json/json_value.cpp
  third_party/jsoncpp/src/lib_json/json_writer.cpp
)
target_include_directories(motion_high_level_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsoncpp/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motion_high_level_client
  rclcpp
  uniubi
)

add_executable(motion_high_level_client_example src/motion_high_level_client_example.cpp)
ament_target_dependencies(motion_high_level_client_example
  rclcpp
  uniubi
)
target_link_libraries(motion_high_level_client_example motion_high_level_client)

add_executable(motion_odometry_subscriber src/motion_odometry_subscriber.cpp)
ament_target_dependencies(motion_odometry_subscriber
  rclcpp
  uniubi
)

install(TARGETS
  motion_high_level_client_example
  motion_odometry_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS motion_high_level_client
  EXPORT export_motion_high_level_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY third_party/jsoncpp/include/json
  DESTINATION include
)

ament_export_targets(export_motion_high_level_client HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp uniubi)

ament_package()
