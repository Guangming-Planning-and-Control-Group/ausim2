cmake_minimum_required(VERSION 3.20)

project(quadrotor_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED MUJOCO_IMPORTED_TARGET)
  message(FATAL_ERROR "MuJoCo dependency was not resolved before entering quadrotor/.")
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

set(QUADROTOR_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
set(QUADROTOR_MUJOCO_SIMULATE_DIR
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco-3.6.0/simulate")

add_library(ausim_mujoco_viewer STATIC
  "${QUADROTOR_MUJOCO_SIMULATE_DIR}/simulate.cc"
  "${QUADROTOR_MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc"
  "${QUADROTOR_MUJOCO_SIMULATE_DIR}/glfw_adapter.cc"
  "${QUADROTOR_MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc"
)

add_library(quadrotor_mujoco_viewer ALIAS ausim_mujoco_viewer)

target_include_directories(ausim_mujoco_viewer
  PUBLIC
    "${QUADROTOR_THIRD_PARTY_DIR}"
)

target_compile_features(ausim_mujoco_viewer PUBLIC cxx_std_17)

target_link_libraries(ausim_mujoco_viewer
  PUBLIC
    glfw
    "${MUJOCO_IMPORTED_TARGET}"
)

add_subdirectory("${QUADROTOR_THIRD_PARTY_DIR}/dynamic_obs_generator" "dynamic_obs_generator_build")

add_library(quadrotor_sim_core STATIC
  src/app/quadrotor_app.cpp
  src/control/motor_mixer.cpp
  src/controller/se3_controller.cpp
  src/math/geometry.cpp
  src/runtime/goal_provider.cpp
  src/runtime/vehicle_runtime.cpp
  src/sim/camera_renderer.cpp
  src/sim/mujoco_actuator_writer.cpp
  src/sim/mujoco_bindings.cpp
  src/sim/mujoco_state_reader.cpp
  src/sim/quadrotor_sim.cpp
)

target_include_directories(quadrotor_sim_core
  PUBLIC
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(quadrotor_sim_core PUBLIC cxx_std_17)
target_compile_definitions(quadrotor_sim_core
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(quadrotor_sim_core
  Eigen3::Eigen
  glfw
  yaml-cpp
  ausim_common_runtime
  ausim_mujoco_viewer
  dynamic_obs_generator
  "${MUJOCO_IMPORTED_TARGET}"
)

add_executable(quadrotor
  src/main.cpp
)

target_link_options(quadrotor PRIVATE "LINKER:--disable-new-dtags")
target_link_libraries(quadrotor PRIVATE quadrotor_sim_core)

add_executable(takeoff_goal_provider_test
  tests/takeoff_goal_provider_test.cpp
)

target_include_directories(takeoff_goal_provider_test
  PRIVATE
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(takeoff_goal_provider_test PRIVATE cxx_std_17)
target_link_libraries(takeoff_goal_provider_test PRIVATE quadrotor_sim_core)

add_executable(reset_simulation_test
  tests/reset_simulation_test.cpp
)

target_include_directories(reset_simulation_test
  PRIVATE
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(reset_simulation_test PRIVATE cxx_std_17)
target_link_libraries(reset_simulation_test PRIVATE quadrotor_sim_core)

add_executable(model_sensor_binding_test
  tests/model_sensor_binding_test.cpp
)

target_include_directories(model_sensor_binding_test
  PRIVATE
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(model_sensor_binding_test PRIVATE cxx_std_17)
target_link_libraries(model_sensor_binding_test PRIVATE quadrotor_sim_core)

set(_quadrotor_runtime_rpaths)
if(MUJOCO_RESOLVED_LIBRARY_DIR)
  list(APPEND _quadrotor_runtime_rpaths "${MUJOCO_RESOLVED_LIBRARY_DIR}")
endif()
if(_quadrotor_runtime_rpaths)
  set_target_properties(quadrotor PROPERTIES
    BUILD_RPATH "${_quadrotor_runtime_rpaths}"
    INSTALL_RPATH "${_quadrotor_runtime_rpaths}"
  )
endif()
if(MUJOCO_RESOLVED_PLUGIN_DIR)
  target_compile_definitions(quadrotor_sim_core
    PUBLIC
      QUADROTOR_MUJOCO_PLUGIN_FALLBACK_DIR="${MUJOCO_RESOLVED_PLUGIN_DIR}"
  )
endif()

install(TARGETS quadrotor quadrotor_sim_core ausim_mujoco_viewer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/assets" DESTINATION share)
install(FILES README.md DESTINATION share/quadrotor)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cfg" DESTINATION share/quadrotor)
