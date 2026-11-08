cmake_minimum_required(VERSION 3.22)
project(piper_controllers LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)

add_library(piper_controllers SHARED
  src/joint_ramp.cpp
  src/policy.cpp
  src/policy_controller.cpp
)
piper_target_defaults(piper_controllers)
target_include_directories(piper_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(piper_controllers PRIVATE
  controller_interface::controller_interface
  ${geometry_msgs_TARGETS}
  hardware_interface::hardware_interface
  onnxruntime::onnxruntime
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  urdf::urdf
)

pluginlib_export_plugin_description_file(controller_interface piper_controllers.xml)

install(TARGETS piper_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_ramp test/test_joint_ramp.cpp)
  piper_target_defaults(test_joint_ramp)
  target_link_libraries(test_joint_ramp piper_controllers)
endif()

ament_package()
