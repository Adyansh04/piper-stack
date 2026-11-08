cmake_minimum_required(VERSION 3.22)
project(piper_estimation LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(piper_estimation src/wrench_estimator.cpp)
piper_target_defaults(piper_estimation)
target_include_directories(piper_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_estimation PUBLIC Eigen3::Eigen kdl_parser::kdl_parser orocos-kdl)

add_library(state_estimator_component SHARED src/state_estimator.cpp)
piper_target_defaults(state_estimator_component)
target_link_libraries(state_estimator_component PRIVATE
  piper_estimation
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
)
rclcpp_components_register_node(state_estimator_component
  PLUGIN "piper_estimation::StateEstimator" EXECUTABLE state_estimator)

install(TARGETS piper_estimation state_estimator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wrench_estimator test/test_wrench_estimator.cpp)
  piper_target_defaults(test_wrench_estimator)
  target_link_libraries(test_wrench_estimator piper_estimation)
endif()

ament_package()
