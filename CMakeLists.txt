cmake_minimum_required(VERSION 3.20)

project(QuadrotorWorkspace LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_SOURCE_DIR "/opt/mujoco" CACHE PATH
    "Path to a MuJoCo source checkout or install prefix. The build will derive install/package paths from here.")

set(MUJOCO_RAY_CASTER_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin/mujoco_plugin" CACHE PATH
    "Directory where external MuJoCo plugins are emitted.")
option(MUJOCO_RAY_CASTER_BUILD_SMOKETEST
       "Build the ray caster plugin smoke test executable." OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path prefix" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

include("${CMAKE_SOURCE_DIR}/third_party/mujoco_ray_caster/cmake/ResolveMuJoCo.cmake")
resolve_mujoco_dependency(
  OUT_TARGET MUJOCO_IMPORTED_TARGET
  OUT_INCLUDE_DIR MUJOCO_RESOLVED_INCLUDE_DIR
  OUT_LIBRARY MUJOCO_RESOLVED_LIBRARY
  OUT_LIBRARY_DIR MUJOCO_RESOLVED_LIBRARY_DIR
  OUT_PLUGIN_DIR MUJOCO_RESOLVED_PLUGIN_DIR
)

add_subdirectory(third_party/mujoco_ray_caster)
add_subdirectory(quadrotor)
add_subdirectory(ground_vehicle)
