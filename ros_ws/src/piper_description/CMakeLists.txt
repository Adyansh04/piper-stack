cmake_minimum_required(VERSION 3.22)
project(piper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Every variant expands to valid xacro.
  foreach(variant hardware:=mock hardware:=can wrist_camera:=false "hardware:=can;motion_mode:=mit;collision_level:=3;end_load:=half")
    string(MAKE_C_IDENTIFIER "xacro_${variant}" test_name)
    add_test(NAME ${test_name}
      COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/piper.urdf.xacro ${variant})
    set_tests_properties(${test_name} PROPERTIES
      ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  endforeach()
endif()

ament_package()
