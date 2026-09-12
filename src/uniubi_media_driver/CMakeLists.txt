cmake_minimum_required(VERSION 3.10)
project(uniubi_media_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/AudioFrame.msg" DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(audio_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)
find_package(UniubiRobotSdk CONFIG REQUIRED)

if(NOT TARGET Uniubi::MediaBus)
  message(FATAL_ERROR
    "uniubi_media_driver requires the MediaBus runtime from UniubiRobotSdk")
endif()

add_executable(uniubi_media_driver_node
  src/uniubi_media_driver_node.cpp
)
target_compile_features(uniubi_media_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(uniubi_media_driver_node
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(uniubi_media_driver_node
  Uniubi::RobotMotionSdk
  Uniubi::MediaBus
  "${audio_typesupport}"
)

install(TARGETS uniubi_media_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/play_pcm.py scripts/capture_pcm.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_audio_queue test/test_audio_queue.cpp)
  target_include_directories(test_audio_queue PRIVATE src)
  target_link_libraries(test_audio_queue "${audio_typesupport}")
  ament_add_gtest(test_audio_bridge test/test_audio_bridge.cpp)
  target_compile_features(test_audio_bridge PRIVATE cxx_std_17)
  target_include_directories(test_audio_bridge PRIVATE src)
  ament_target_dependencies(test_audio_bridge rclcpp std_srvs)
  target_link_libraries(test_audio_bridge "${audio_typesupport}" Uniubi::MediaBus)
endif()
ament_package()
