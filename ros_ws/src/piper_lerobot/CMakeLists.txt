cmake_minimum_required(VERSION 3.22)
project(piper_lerobot LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cppzmq REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(piper_lerobot_protocol src/bridge_protocol.cpp)
piper_target_defaults(piper_lerobot_protocol)
target_include_directories(piper_lerobot_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(piper_lerobot_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(lerobot_bridge_component SHARED src/lerobot_bridge.cpp)
piper_target_defaults(lerobot_bridge_component)
target_link_libraries(lerobot_bridge_component PRIVATE
  piper_lerobot_protocol
  ${control_msgs_TARGETS}
  cppzmq
  ${moveit_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(lerobot_bridge_component
  PLUGIN "piper_lerobot::LeRobotBridge" EXECUTABLE lerobot_bridge)

install(TARGETS lerobot_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/expert_episodes.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_protocol test/test_bridge_protocol.cpp)
  piper_target_defaults(test_bridge_protocol)
  target_link_libraries(test_bridge_protocol piper_lerobot_protocol)

  # The plugins' tests need LeRobot's venv (Python 3.12), and must not see ROS's Python 3.14
  # packages or their pytest plugins.
  set(LEROBOT_PYTHON /opt/lerobot-venv/bin/python)
  if(EXISTS ${LEROBOT_PYTHON})
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_lerobot_plugins lerobot
      PYTHON_EXECUTABLE ${LEROBOT_PYTHON}
      ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/lerobot PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      TIMEOUT 120
    )
  else()
    message(WARNING "piper_lerobot: no ${LEROBOT_PYTHON}, the LeRobot plugins' tests are skipped")
  endif()
endif()

ament_package()
