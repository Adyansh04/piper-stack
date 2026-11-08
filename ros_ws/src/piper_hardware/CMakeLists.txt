cmake_minimum_required(VERSION 3.22)
project(piper_hardware LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(piper_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(piper_hardware SHARED src/piper_system.cpp)
piper_target_defaults(piper_hardware)
target_include_directories(piper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_hardware PUBLIC
  hardware_interface::hardware_interface
  piper_driver::piper_driver
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface piper_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS piper_hardware EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface piper_driver pluginlib rclcpp rclcpp_lifecycle)
ament_package()
