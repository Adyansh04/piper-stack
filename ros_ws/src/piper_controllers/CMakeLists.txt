cmake_minimum_required(VERSION 3.22)
project(piper_controllers LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)

# The motion law and the ONNX policy, without ROS: the node and the tests share them.
add_library(piper_policy SHARED
  src/joint_ramp.cpp
  src/policy.cpp
)
piper_target_defaults(piper_policy)
target_include_directories(piper_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(piper_policy PRIVATE onnxruntime::onnxruntime)

add_executable(policy_runner src/policy_runner.cpp)
piper_target_defaults(policy_runner)
target_link_libraries(policy_runner PRIVATE
  piper_policy
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  urdf::urdf
)

install(TARGETS piper_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS policy_runner RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_ramp test/test_joint_ramp.cpp)
  piper_target_defaults(test_joint_ramp)
  target_link_libraries(test_joint_ramp piper_policy)
endif()

ament_package()
