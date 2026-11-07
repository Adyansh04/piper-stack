cmake_minimum_required(VERSION 3.22)
project(piper_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
