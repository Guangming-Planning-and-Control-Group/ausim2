cmake_minimum_required(VERSION 3.20)

project(ausim_common_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(_ausim_ros_prefix "/opt/ros/humble")
if(EXISTS "${_ausim_ros_prefix}")
  list(APPEND CMAKE_PREFIX_PATH "${_ausim_ros_prefix}")
  if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    set(ENV{AMENT_PREFIX_PATH} "${_ausim_ros_prefix}:$ENV{AMENT_PREFIX_PATH}")
  else()
    set(ENV{AMENT_PREFIX_PATH} "${_ausim_ros_prefix}")
  endif()
endif()

set(_ausim_ros_python "/opt/ros/humble/lib/python3.10/site-packages")
if(EXISTS "${_ausim_ros_python}")
  if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
    set(ENV{PYTHONPATH} "${_ausim_ros_python}:$ENV{PYTHONPATH}")
  else()
    set(ENV{PYTHONPATH} "${_ausim_ros_python}")
  endif()
endif()

add_library(ausim_common_runtime STATIC
  src/config/quadrotor_config.cpp
  src/converts/ipc/bridge_packets.cpp
  src/manager/ros_bridge_process_manager.cpp
  src/runtime/robot_mode_state_machine.cpp
  src/runtime/data_board_interface.cpp
)

target_include_directories(ausim_common_runtime
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(ausim_common_runtime PUBLIC cxx_std_17)
target_compile_definitions(ausim_common_runtime
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_SOURCE_DIR}/quadrotor"
)

target_link_libraries(ausim_common_runtime
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
)

add_library(ausim_common_ros_bridge_core STATIC
  src/converts/data/clock.cpp
  src/converts/data/cmd_vel.cpp
  src/converts/data/common.cpp
  src/converts/data/image.cpp
  src/converts/data/imu.cpp
  src/converts/data/odom.cpp
  src/converts/data/transform.cpp
  src/ros/publisher/data/clock_data_publisher.cpp
  src/ros/publisher/data/image_data_publisher.cpp
  src/ros/publisher/data/imu_data_publisher.cpp
  src/ros/publisher/data/lidar_data_publisher.cpp
  src/ros/publisher/data/odom_data_publisher.cpp
  src/ros/publisher/data/transform_data_publisher.cpp
  src/ros/ros2_bridge.cpp
  src/ros/subscriber/data/cmd_vel_command_subscriber.cpp
  src/ros/subscriber/data/teleop_event_subscriber.cpp
)

target_include_directories(ausim_common_ros_bridge_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(ausim_common_ros_bridge_core PUBLIC cxx_std_17)
target_link_libraries(ausim_common_ros_bridge_core ausim_common_runtime)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_target_dependencies(
  ausim_common_ros_bridge_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosgraph_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

foreach(_bridge_target ausim_ros_bridge quadrotor_ros_bridge)
  add_executable(${_bridge_target}
    src/ros/ros_bridge_main.cpp
  )
  target_link_options(${_bridge_target} PRIVATE "LINKER:--disable-new-dtags")
  target_link_libraries(${_bridge_target} PRIVATE ausim_common_ros_bridge_core)
endforeach()

set(_ausim_ros_runtime_rpaths)
set(_ausim_ros_libdir "/opt/ros/humble/lib")
if(EXISTS "${_ausim_ros_libdir}")
  list(APPEND _ausim_ros_runtime_rpaths "${_ausim_ros_libdir}")
endif()
if(_ausim_ros_runtime_rpaths)
  set_target_properties(ausim_ros_bridge quadrotor_ros_bridge PROPERTIES
    BUILD_RPATH "${_ausim_ros_runtime_rpaths}"
    INSTALL_RPATH "${_ausim_ros_runtime_rpaths}"
  )
endif()

install(TARGETS
  ausim_common_runtime
  ausim_common_ros_bridge_core
  ausim_ros_bridge
  quadrotor_ros_bridge
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_executable(robot_mode_state_machine_test
  tests/robot_mode_state_machine_test.cpp
)

target_include_directories(robot_mode_state_machine_test
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(robot_mode_state_machine_test PRIVATE cxx_std_17)
target_link_libraries(robot_mode_state_machine_test PRIVATE ausim_common_runtime)

add_executable(joy_interface_config_test
  tests/joy_interface_config_test.cpp
)

target_include_directories(joy_interface_config_test
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(joy_interface_config_test PRIVATE cxx_std_17)
target_link_libraries(joy_interface_config_test PRIVATE ausim_common_runtime)

add_executable(sensor_config_dedup_test
  tests/sensor_config_dedup_test.cpp
)

target_include_directories(sensor_config_dedup_test
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(sensor_config_dedup_test PRIVATE cxx_std_17)
target_link_libraries(sensor_config_dedup_test PRIVATE ausim_common_runtime)
