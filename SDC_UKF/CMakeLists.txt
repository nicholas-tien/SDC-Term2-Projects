cmake_minimum_required(VERSION 3.7)
project(SDC_UKF)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/main.cpp
        src/UKF.cpp
        src/Tools.cpp
       )
add_executable(SDC_UKF ${SOURCE_FILES})