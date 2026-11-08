cmake_minimum_required(VERSION 3.22)
project(piper_skills LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(tinyxml2 REQUIRED)
find_package(yaml-cpp REQUIRED)

# The skills' geometry: plain C++ with no ROS, unit-tested on its own.
add_library(piper_skills_core SHARED
  src/grasp_selection.cpp
  src/object_cloud.cpp
  src/place_geometry.cpp
  src/place_target.cpp
  src/srdf_states.cpp
  src/task_stages.cpp
  src/view_planner.cpp
)
target_include_directories(piper_skills_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_skills_core
  PUBLIC Eigen3::Eigen opencv_core
  PRIVATE opencv_imgproc tinyxml2::tinyxml2 yaml-cpp::yaml-cpp
)
piper_target_defaults(piper_skills_core)
install(TARGETS piper_skills_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_msgs REQUIRED)

# A plugin loaded by the component container or the generated executable: nothing links it.
add_library(skills_component SHARED
  src/arm_motion.cpp
  src/camera_capture.cpp
  src/scene_client.cpp
  src/skills_node.cpp
  src/stage_runner.cpp
)
target_link_libraries(skills_component PRIVATE
  piper_skills_core
  cv_bridge::cv_bridge
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${piper_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
)
piper_target_defaults(skills_component)
# A skill blocks its own thread; the executor keeps serving cancels, TF and the camera meanwhile.
rclcpp_components_register_node(skills_component
  PLUGIN "piper_skills::SkillsNode"
  EXECUTABLE skills_node
  EXECUTOR MultiThreadedExecutor
)
install(TARGETS skills_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# A task's parts in MoveIt's planning scene.
add_library(task_scene_component SHARED src/scene_client.cpp src/task_scene.cpp)
target_include_directories(task_scene_component PRIVATE include)
target_link_libraries(task_scene_component PRIVATE
  geometric_shapes::geometric_shapes
  moveit_core::moveit_collision_detection
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_eigen::tf2_eigen
  yaml-cpp::yaml-cpp
  ${moveit_msgs_TARGETS}
  ${piper_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
)
piper_target_defaults(task_scene_component)
rclcpp_components_register_node(task_scene_component
  PLUGIN "piper_skills::TaskScene"
  EXECUTABLE task_scene
  EXECUTOR MultiThreadedExecutor
)
install(TARGETS task_scene_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_skills_core
    test/test_core.cpp test/test_place_target.cpp test/test_task_stages.cpp)
  target_link_libraries(test_skills_core piper_skills_core opencv_imgproc)
  piper_target_defaults(test_skills_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
