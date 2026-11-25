# Minimal copy of tf2_msgs (ROS 2 Humble) containing only TFMessage,
# so the micro-ROS library can publish /tf. micro_ros_platformio builds every
# package in extra_packages/ into the firmware's micro-ROS library.
cmake_minimum_required(VERSION 3.5)
project(tf2_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TFMessage.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
