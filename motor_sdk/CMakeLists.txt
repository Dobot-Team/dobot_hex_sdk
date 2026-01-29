cmake_minimum_required(VERSION 3.0)
project(motor_sdk)

# add_definitions(-DDBUG)

include_directories(include)

link_directories(lib)

add_executable(motor_read ./example/motor_read.cpp)
add_executable(motor_wave ./example/motor_wave.cpp)

target_link_libraries(motor_read motorSDK_Linux64 yaml-cpp)
target_link_libraries(motor_wave motorSDK_Linux64 yaml-cpp)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INSTALL(FILES lib/libmotorSDK_Linux64.so
  DESTINATION lib
)

INSTALL(DIRECTORY include/
  DESTINATION include/motor_sdk)
