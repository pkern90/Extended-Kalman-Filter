cmake_minimum_required(VERSION 3.6)
project(Extended_Kalman_Filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
        src/FusionEKF.cpp
        src/kalman_filter.cpp
        src/main.cpp
        src/tools.cpp)
add_executable(Extended_Kalman_Filter ${SOURCE_FILES})

add_definitions(-std=c++0x)
