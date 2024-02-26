cmake_minimum_required(VERSION 3.8)
project(ic120_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)

rosidl_generate_interfaces(ic120_msgs
  "ic120_navigation/srv/DumpNav.srv"

  DEPENDENCIES 
  std_msgs
  sensor_msgs
  shape_msgs
  geometry_msgs
)


ament_package()
