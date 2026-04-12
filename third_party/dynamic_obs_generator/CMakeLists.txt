cmake_minimum_required(VERSION 3.16)
project(dynamic_obs_generator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Resolve MuJoCo dependency if not already done
if(NOT DEFINED MUJOCO_IMPORTED_TARGET)
  set(MUJOCO_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../mujoco-3.6.0" CACHE PATH
      "Path to the vendored MuJoCo 3.6.0 release prefix.")
  include("${CMAKE_CURRENT_LIST_DIR}/../mujoco_ray_caster/cmake/ResolveMuJoCo.cmake")
  resolve_mujoco_dependency(
    OUT_TARGET MUJOCO_IMPORTED_TARGET
    OUT_INCLUDE_DIR MUJOCO_RESOLVED_INCLUDE_DIR
    OUT_LIBRARY MUJOCO_RESOLVED_LIBRARY
    OUT_LIBRARY_DIR MUJOCO_RESOLVED_LIBRARY_DIR
    OUT_PLUGIN_DIR MUJOCO_RESOLVED_PLUGIN_DIR
  )
endif()

# This library depends on Eigen, yaml-cpp, and MuJoCo
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(dynamic_obs_generator STATIC
  obstacle_config.cpp
  dynamic_obstacle_manager.cpp
)

target_include_directories(dynamic_obs_generator
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MUJOCO_RESOLVED_INCLUDE_DIR}
)

target_compile_features(dynamic_obs_generator PUBLIC cxx_std_17)

target_link_libraries(dynamic_obs_generator
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
    "${MUJOCO_IMPORTED_TARGET}"
)

# Install the obstacle.yaml template
install(FILES obstacle.yaml DESTINATION share/dynamic_obs_generator)
