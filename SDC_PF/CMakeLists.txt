cmake_minimum_required(VERSION 3.7)
project(SDC_PF)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/main.cpp src/particle_filter.cpp src/particle_filter.h)
add_executable(SDC_PF ${SOURCE_FILES})