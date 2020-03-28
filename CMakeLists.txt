cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_BUILD_TYPE Debug)

add_definitions() #-DDEBUG -DVIEW_TABLE

project(optimalcontrol)

add_subdirectory(envs)
add_subdirectory(hw7)