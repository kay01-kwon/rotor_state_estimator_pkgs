cmake_minimum_required(VERSION 3.8)
project(rotor_state_estimator)

set(CMAKE_CXX_STANDARD 17)

set(ckf_src
  include/rotor_state_estimator/ckf/constrained_kalman_filter.cpp
  include/rotor_state_estimator/node/ckf_node.cpp
)

set(single_ckf_src
  include/rotor_state_estimator/ckf/constrained_kalman_filter.cpp
  include/rotor_state_estimator/node/single_rotor_ckf_node.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_libcanard_msgs REQUIRED)
find_package(rotor_state_msgs REQUIRED)

include_directories(
  include
)

add_library(ckf_lib
  ${ckf_src}
)

ament_target_dependencies(
  ckf_lib
  rclcpp
  std_msgs
  ros2_libcanard_msgs
  rotor_state_msgs
)

# CKF Node
add_executable(
  ckf_node
  src/ckf_node_main.cpp
)

target_link_libraries(
  ckf_node
  ckf_lib
)

# Single Rotor CKF Library
add_library(single_ckf_lib
  ${single_ckf_src}
)

ament_target_dependencies(
  single_ckf_lib
  rclcpp
  std_msgs
  ros2_libcanard_msgs
  rotor_state_msgs
)

# Single Rotor CKF Node
add_executable(
  single_rotor_ckf_node
  src/single_rotor_ckf_node_main.cpp
)

target_link_libraries(
  single_rotor_ckf_node
  single_ckf_lib
)

install(
  TARGETS ckf_node single_rotor_ckf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
