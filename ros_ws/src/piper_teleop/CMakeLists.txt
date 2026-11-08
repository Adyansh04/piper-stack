cmake_minimum_required(VERSION 3.22)
project(piper_teleop LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(piper_teleop_key_map src/key_map.cpp)
piper_target_defaults(piper_teleop_key_map)
target_include_directories(piper_teleop_key_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(keyboard_jog src/keyboard_jog.cpp)
piper_target_defaults(keyboard_jog)
target_link_libraries(keyboard_jog PRIVATE
  piper_teleop_key_map
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

install(TARGETS piper_teleop_key_map keyboard_jog
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_key_map test/test_key_map.cpp)
  piper_target_defaults(test_key_map)
  target_link_libraries(test_key_map piper_teleop_key_map)
endif()

ament_package()
