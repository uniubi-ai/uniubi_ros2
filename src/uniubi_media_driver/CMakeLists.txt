cmake_minimum_required(VERSION 3.10)
project(uniubi_media_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
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
)
target_link_libraries(uniubi_media_driver_node
  Uniubi::RobotMotionSdk
  Uniubi::MediaBus
)

install(TARGETS uniubi_media_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
