cmake_minimum_required(VERSION 2.8.3)
project(sdk_python_wrapper)

set(PYTHON_EXECUTABLE "/usr/bin/python3")

find_package(yaml-cpp REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../lib/python)

add_subdirectory(third-party/pybind11)

pybind11_add_module(robot_interface robot_interface.cpp)
target_link_libraries(robot_interface motorSDK_Linux64 yaml-cpp)
