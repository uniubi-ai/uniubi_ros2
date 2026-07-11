cmake_minimum_required(VERSION 3.8)
project(uniubi_interface_test)

include(FetchContent)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uniubi REQUIRED)

FetchContent_Declare(jsoncpp
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsoncpp
)
set(JSONCPP_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_POST_BUILD_UNITTEST OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_WARNING_AS_ERROR OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_PKGCONFIG_SUPPORT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(jsoncpp)

add_library(motion_high_level_client
  src/system_rpc_client_base.cpp
  src/motion_high_level_client.cpp
)
target_include_directories(motion_high_level_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motion_high_level_client
  rclcpp
  uniubi
)
target_link_libraries(motion_high_level_client jsoncpp_lib)

add_executable(motion_high_level_client_test src/motion_high_level_client_test.cpp)
ament_target_dependencies(motion_high_level_client_test
  rclcpp
  uniubi
)
target_link_libraries(motion_high_level_client_test motion_high_level_client jsoncpp_lib)

install(TARGETS
  motion_high_level_client_test
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

ament_export_targets(export_motion_high_level_client HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp uniubi)

ament_package()
