cmake_minimum_required(VERSION 3.22)
project(piper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Every hardware variant expands to valid xacro.
  foreach(hardware mock can)
    add_test(NAME xacro_${hardware}
      COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/piper.urdf.xacro hardware:=${hardware})
    set_tests_properties(xacro_${hardware} PROPERTIES
      ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  endforeach()
endif()

ament_package()
