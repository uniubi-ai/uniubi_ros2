cmake_minimum_required(VERSION 3.8)
project(uniubi_motion_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uniubi REQUIRED)
find_package(CycloneDDS REQUIRED)

set(native_idl_dir "${uniubi_DIR}/../native_idl")
idlc_generate(TARGET cere_motion_types
  FILES "${native_idl_dir}/MotorState.idl" "${native_idl_dir}/MotionObserved.idl"
        "${native_idl_dir}/RemoteControl.idl" "${native_idl_dir}/SensorObserved.idl"
        "${native_idl_dir}/BrainMotionState.idl"
  INCLUDES "${native_idl_dir}"
)


add_library(motion_high_level_client
  src/cere_sensor_reader.cpp
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
target_link_libraries(motion_high_level_client $<BUILD_INTERFACE:cere_motion_types> CycloneDDS::ddsc)
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

add_executable(motion_high_level_client_cli src/motion_high_level_client_cli.cpp)
ament_target_dependencies(motion_high_level_client_cli
  rclcpp
  uniubi
)
target_link_libraries(motion_high_level_client_cli motion_high_level_client)

add_executable(sensor_observed_subscriber src/sensor_observed_subscriber.cpp)
ament_target_dependencies(sensor_observed_subscriber
  rclcpp
  uniubi
)

install(TARGETS
  motion_high_level_client_example
  motion_high_level_client_cli
  sensor_observed_subscriber
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
ament_export_dependencies(rclcpp uniubi CycloneDDS)

if(BUILD_TESTING)
  enable_testing()
  add_executable(cere_sensor_reader_test test/cere_sensor_reader_test.cpp)
  target_include_directories(cere_sensor_reader_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(cere_sensor_reader_test motion_high_level_client CycloneDDS::ddsc)
  ament_target_dependencies(cere_sensor_reader_test rclcpp uniubi)
  add_executable(client_event_sn_test test/client_event_sn_test.cpp)
  target_link_libraries(client_event_sn_test motion_high_level_client)
  ament_target_dependencies(client_event_sn_test rclcpp uniubi)
  add_test(NAME client_event_sn_test COMMAND client_event_sn_test)
  set_tests_properties(client_event_sn_test PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_DOMAIN_ID=177;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  add_executable(native_robot_topic_test test/native_robot_topic_test.cpp)
  target_include_directories(native_robot_topic_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(native_robot_topic_test cere_motion_types CycloneDDS::ddsc)
  ament_target_dependencies(native_robot_topic_test rclcpp uniubi)
  add_test(NAME native_robot_topic_test COMMAND native_robot_topic_test)
  set_tests_properties(native_robot_topic_test PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_DOMAIN_ID=176;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
  add_test(NAME cere_sensor_reader_test COMMAND cere_sensor_reader_test)
  # Explicit lo binding isolates the test; ROS_LOCALHOST_ONLY=1 would add lo again
  # in Humble rmw_cyclonedds_cpp with CycloneDDS 0.10.x.
  set_tests_properties(cere_sensor_reader_test PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_DOMAIN_ID=173;ROS_LOCALHOST_ONLY=0;CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='lo'/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>")
endif()

ament_package()
