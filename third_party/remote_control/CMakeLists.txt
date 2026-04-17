cmake_minimum_required(VERSION 3.20)

project(remote_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(remote_control_node
  src/main.cpp
  src/remote_control_node.cpp
)

target_include_directories(remote_control_node
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(remote_control_node PRIVATE cxx_std_17)
target_link_options(remote_control_node PRIVATE "LINKER:--disable-new-dtags")

ament_target_dependencies(
  remote_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS remote_control_node
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/remote_control
)

install(FILES README.md
  DESTINATION share/remote_control
)
