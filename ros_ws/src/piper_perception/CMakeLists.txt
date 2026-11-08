cmake_minimum_required(VERSION 3.22)
project(piper_perception LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(piper_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ZLIB REQUIRED)
find_package(cppzmq REQUIRED)
# The engine needs CUDA and TensorRT; without them only the CPU parts (tokenizer, tensor types,
# request checks) and their tests build. TensorRT ships no CMake package: its libraries come from
# NVIDIA's apt repository.
find_package(CUDAToolkit QUIET)
find_library(NVINFER_LIBRARY nvinfer)
find_library(NVONNXPARSER_LIBRARY nvonnxparser)
if(CUDAToolkit_FOUND AND NVINFER_LIBRARY AND NVONNXPARSER_LIBRARY)
  set(SAM3_WITH_ENGINE ON)
else()
  set(SAM3_WITH_ENGINE OFF)
  message(WARNING "piper_perception: CUDA or TensorRT not found, building the CPU parts only")
endif()



add_library(piper_perception_cpu SHARED
  src/clip_tokenizer.cpp
  src/request_checks.cpp
  src/tensor_types.cpp
)
target_include_directories(piper_perception_cpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(piper_perception_cpu PRIVATE ZLIB::ZLIB)
piper_target_defaults(piper_perception_cpu)
install(TARGETS piper_perception_cpu
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(SAM3_WITH_ENGINE)
  add_library(piper_perception_core SHARED src/sam3_engine.cpp)
  target_link_libraries(piper_perception_core
    PUBLIC piper_perception_cpu opencv_core
    PRIVATE opencv_imgproc CUDA::cudart ${NVINFER_LIBRARY} ${NVONNXPARSER_LIBRARY}
  )
  piper_target_defaults(piper_perception_core)

  # A plugin loaded by the component container or the generated executable: nothing links it, so
  # its dependencies stay private and it is not exported.
  add_library(sam3_segmentation_component SHARED src/sam3_segmentation_node.cpp)
  target_link_libraries(sam3_segmentation_component PRIVATE
    piper_perception_core
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    rclcpp_components::component
    ${piper_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
  )
  piper_target_defaults(sam3_segmentation_component)
  # Images keep arriving while a request runs.
  rclcpp_components_register_node(sam3_segmentation_component
    PLUGIN "piper_perception::Sam3SegmentationNode"
    EXECUTABLE sam3_segmentation_node
    EXECUTOR MultiThreadedExecutor
  )
  install(TARGETS piper_perception_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS sam3_segmentation_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

# The GraspGenX server's client and the node that puts the server on the ROS graph.
add_library(piper_graspgenx_client STATIC src/graspgenx_client.cpp)
set_target_properties(piper_graspgenx_client PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(piper_graspgenx_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(piper_graspgenx_client PUBLIC cppzmq)
piper_target_defaults(piper_graspgenx_client)

add_library(grasp_generation_component SHARED src/grasp_generation_node.cpp)
target_link_libraries(grasp_generation_component PRIVATE
  piper_graspgenx_client
  rclcpp::rclcpp
  rclcpp_components::component
  ${piper_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)
piper_target_defaults(grasp_generation_component)
# A request blocks its callback for up to timeout_s.
rclcpp_components_register_node(grasp_generation_component
  PLUGIN "piper_perception::GraspGenerationNode"
  EXECUTABLE grasp_generation_node
  EXECUTOR MultiThreadedExecutor
)
install(TARGETS grasp_generation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# The server runs in the GraspGenX venv (/opt/graspgenx-venv), started by its launch file.
install(FILES graspgenx/graspgenx_server.py graspgenx/frames.py
  DESTINATION share/${PROJECT_NAME}/graspgenx)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch resources DESTINATION share/${PROJECT_NAME})
# Documentation of the export: it runs in a venv with torch and Meta's sam3, not in this image.
install(FILES scripts/export_sam3.py DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sam3_cpu test/test_clip_tokenizer.cpp test/test_cpu_parts.cpp)
  target_link_libraries(test_sam3_cpu piper_perception_cpu)
  target_compile_definitions(test_sam3_cpu PRIVATE
    SAM3_VOCABULARY="${CMAKE_CURRENT_SOURCE_DIR}/resources/bpe_simple_vocab_16e6.txt.gz")
  piper_target_defaults(test_sam3_cpu)

  ament_add_gtest(test_graspgenx_client test/test_graspgenx_client.cpp)
  target_link_libraries(test_graspgenx_client piper_graspgenx_client)
  piper_target_defaults(test_graspgenx_client)

  # The server's tests need its venv's Python 3.12 (numpy, scipy, zmq), and must not see ROS's
  # Python 3.14 packages or their pytest plugins.
  set(GRASPGENX_PYTHON /opt/graspgenx-venv/bin/python)
  if(EXISTS ${GRASPGENX_PYTHON})
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_graspgenx_server graspgenx
      PYTHON_EXECUTABLE ${GRASPGENX_PYTHON}
      ENV PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      TIMEOUT 120
    )
  else()
    message(WARNING "piper_perception: no ${GRASPGENX_PYTHON}, the GraspGenX server's tests are skipped")
  endif()

  if(SAM3_WITH_ENGINE)
    # Needs a GPU and the engines: built for manual runs, never registered with ctest.
    add_executable(check_sam3_engine test/check_sam3_engine.cpp)
    target_link_libraries(check_sam3_engine PRIVATE
      piper_perception_core opencv_imgproc opencv_imgcodecs)
    piper_target_defaults(check_sam3_engine)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)
ament_package()
