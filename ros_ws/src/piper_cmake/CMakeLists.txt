cmake_minimum_required(VERSION 3.22)
project(piper_cmake NONE)

find_package(ament_cmake REQUIRED)

ament_package(CONFIG_EXTRAS cmake/piper_cmake-extras.cmake)
