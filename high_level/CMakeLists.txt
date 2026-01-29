cmake_minimum_required(VERSION 3.5)
project(bodyTwistControlPublisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msg REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${custom_msg_INCLUDE_DIRS}
)

add_executable(body_twist src/body_twist.cpp)
target_link_libraries(body_twist
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${custom_msg_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

add_executable(locomotion src/locomotion.cpp)
target_link_libraries(locomotion
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${custom_msg_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

add_executable(robot_state src/robot_state.cpp)
target_link_libraries(robot_state
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${custom_msg_LIBRARIES}
)