cmake_minimum_required(VERSION 3.22)
project(piper_bt LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


# The leaves and their registration, apart from the executor so tests and tools reuse them.
add_library(piper_bt_leaves SHARED
  src/port_types.cpp
  src/registration.cpp
  src/ros_leaf.cpp
  src/skills/find_objects.cpp
  src/skills/move_arm_to_named_pose.cpp
  src/skills/pick_object.cpp
  src/skills/place_object.cpp
  src/skills/place_target.cpp
  src/skills/view_workspace.cpp
)
target_include_directories(piper_bt_leaves PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_bt_leaves PUBLIC
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${piper_msgs_TARGETS}
)
piper_target_defaults(piper_bt_leaves)

add_library(piper_bt_executor SHARED src/bt_executor.cpp src/tree_library.cpp)
target_link_libraries(piper_bt_executor PUBLIC
  piper_bt_leaves
  ament_index_cpp::ament_index_cpp
  ${diagnostic_msgs_TARGETS}
)
piper_target_defaults(piper_bt_executor)

add_executable(bt_executor src/bt_executor_main.cpp)
target_link_libraries(bt_executor PRIVATE piper_bt_executor)
piper_target_defaults(bt_executor)

add_executable(bt_node_model src/bt_node_model_main.cpp)
target_link_libraries(bt_node_model PRIVATE piper_bt_leaves)
piper_target_defaults(bt_node_model)

install(TARGETS piper_bt_leaves piper_bt_executor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS bt_executor bt_node_model DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config groot launch trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # Isolated: each test binary gets a ROS domain of its own, so parallel runs do not meet.
  ament_add_ros_isolated_gtest(test_tree_loads test/test_tree_loads.cpp)
  target_link_libraries(test_tree_loads piper_bt_executor)
  target_compile_definitions(test_tree_loads PRIVATE
    PIPER_BT_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees"
    PIPER_BT_PALETTE="${CMAKE_CURRENT_SOURCE_DIR}/groot/piper_bt_nodes.xml")
  piper_target_defaults(test_tree_loads)

  # Its own main: rclcpp is initialised once for every test.
  ament_add_ros_isolated_gtest(
    test_leaves test/test_leaves.cpp TIMEOUT 180 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_leaves piper_bt_executor)
  target_compile_definitions(test_leaves PRIVATE
    PIPER_BT_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")
  piper_target_defaults(test_leaves)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp diagnostic_msgs geometry_msgs piper_msgs rclcpp rclcpp_action)
ament_package()
