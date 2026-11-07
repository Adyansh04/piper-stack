cmake_minimum_required(VERSION 3.22)
project(piper_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(piper_driver
  src/can_bus.cpp
  src/arm.cpp
  src/fake_arm.cpp
)
target_compile_features(piper_driver PUBLIC cxx_std_23)
target_compile_options(piper_driver PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_include_directories(piper_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_driver PUBLIC Threads::Threads)
set_target_properties(piper_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(piper_monitor tools/piper_monitor.cpp)
target_link_libraries(piper_monitor PRIVATE piper_driver)
add_executable(piper_fake_arm tools/piper_fake_arm.cpp)
target_link_libraries(piper_fake_arm PRIVATE piper_driver)
add_executable(piper_disable tools/piper_disable.cpp)
target_link_libraries(piper_disable PRIVATE piper_driver)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS piper_driver EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS piper_monitor piper_fake_arm piper_disable DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol piper_driver)
  ament_add_gtest(test_triple_buffer test/test_triple_buffer.cpp)
  target_link_libraries(test_triple_buffer piper_driver)
  # Needs vcan0 up on the host; skips itself without it.
  ament_add_gtest(test_arm_vcan test/test_arm_vcan.cpp TIMEOUT 60)
  target_link_libraries(test_arm_vcan piper_driver)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)
ament_package()
