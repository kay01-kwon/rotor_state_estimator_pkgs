cmake_minimum_required(VERSION 3.8)
project(rotor_state_msgs)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/RotorState.msg"
  "msg/RotorCov.msg"
  DEPENDENCIES std_msgs
)

ament_package()