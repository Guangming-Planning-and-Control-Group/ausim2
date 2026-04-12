cmake_minimum_required(VERSION 3.20)

project(mujoco_ray_caster LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MUJOCO_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../mujoco-3.6.0" CACHE PATH
    "Path to the vendored MuJoCo 3.6.0 release prefix.")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ResolveMuJoCo.cmake")
resolve_mujoco_dependency(
  OUT_TARGET MUJOCO_IMPORTED_TARGET
  OUT_INCLUDE_DIR MUJOCO_RESOLVED_INCLUDE_DIR
  OUT_LIBRARY MUJOCO_RESOLVED_LIBRARY
  OUT_LIBRARY_DIR MUJOCO_RESOLVED_LIBRARY_DIR
  OUT_PLUGIN_DIR MUJOCO_RESOLVED_PLUGIN_DIR
)

set(MUJOCO_RAY_CASTER_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin/mujoco_plugin" CACHE PATH
    "Directory where the MuJoCo ray caster plugin library is emitted.")
option(MUJOCO_RAY_CASTER_BUILD_SMOKETEST
       "Build the MuJoCo ray caster plugin smoke test executable." OFF)

set(MUJOCO_SENSOR_SRCS
  register.cc
  ray_plugin.cc
  ray_caster_plugin.cc
  ray_caster_camera_plugin.cc
  ray_caster_lidar_plugin.cc
)
file(GLOB RAY_CASTER_CORE_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/raycaster_src/*.cpp")

add_library(sensor_raycaster SHARED
  ${MUJOCO_SENSOR_SRCS}
  ${RAY_CASTER_CORE_SRCS}
)

target_include_directories(sensor_raycaster
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/raycaster_src"
)
target_compile_features(sensor_raycaster PUBLIC cxx_std_17)
target_link_libraries(sensor_raycaster PRIVATE "${MUJOCO_IMPORTED_TARGET}")
set_target_properties(sensor_raycaster PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${MUJOCO_RAY_CASTER_OUTPUT_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${MUJOCO_RAY_CASTER_OUTPUT_DIR}"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

if(MUJOCO_RESOLVED_LIBRARY_DIR)
  set_target_properties(sensor_raycaster PROPERTIES
    BUILD_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
  )
endif()

install(TARGETS sensor_raycaster
  RUNTIME DESTINATION bin/mujoco_plugin
  LIBRARY DESTINATION bin/mujoco_plugin
  ARCHIVE DESTINATION lib
)

if(MUJOCO_RAY_CASTER_BUILD_SMOKETEST)
  add_executable(mujoco_ray_caster_smoketest smoke_test.cc)
  target_compile_features(mujoco_ray_caster_smoketest PUBLIC cxx_std_17)
  target_compile_definitions(mujoco_ray_caster_smoketest PRIVATE
    MUJOCO_RAY_CASTER_PLUGIN_DIR="${MUJOCO_RAY_CASTER_OUTPUT_DIR}"
    MUJOCO_RAY_CASTER_SMOKETEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/model/ray_caster.xml"
  )
  target_link_libraries(mujoco_ray_caster_smoketest PRIVATE "${MUJOCO_IMPORTED_TARGET}")
  if(MUJOCO_RESOLVED_LIBRARY_DIR)
    set_target_properties(mujoco_ray_caster_smoketest PROPERTIES
      BUILD_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
      INSTALL_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
    )
  endif()
endif()
