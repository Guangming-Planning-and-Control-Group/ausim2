cmake_minimum_required(VERSION 3.20)

project(ground_vehicle_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED MUJOCO_IMPORTED_TARGET)
  message(FATAL_ERROR "MuJoCo dependency was not resolved before entering ground_vehicle/.")
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

add_library(ground_vehicle_sim_core STATIC
  src/config/scout_config.cpp
  src/control/differential_drive_controller.cpp
  src/sim/scout_sim.cpp
  src/sim/wheel_actuator_writer.cpp
)

target_include_directories(ground_vehicle_sim_core
  PUBLIC
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(ground_vehicle_sim_core PUBLIC cxx_std_17)
target_compile_definitions(ground_vehicle_sim_core
  PUBLIC
    GROUND_VEHICLE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    QUADROTOR_SOURCE_DIR="${CMAKE_SOURCE_DIR}/quadrotor"
)

target_link_libraries(ground_vehicle_sim_core
  Eigen3::Eigen
  glfw
  yaml-cpp
  ausim_common_runtime
  ausim_mujoco_viewer
  "${MUJOCO_IMPORTED_TARGET}"
)

add_executable(scout
  src/main.cpp
)

target_link_options(scout PRIVATE "LINKER:--disable-new-dtags")
target_link_libraries(scout PRIVATE ground_vehicle_sim_core)

if(MUJOCO_RESOLVED_LIBRARY_DIR)
  set_target_properties(scout PROPERTIES
    BUILD_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
    INSTALL_RPATH "${MUJOCO_RESOLVED_LIBRARY_DIR}"
  )
endif()
if(MUJOCO_RESOLVED_PLUGIN_DIR)
  target_compile_definitions(ground_vehicle_sim_core
    PUBLIC
      GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR="${MUJOCO_RESOLVED_PLUGIN_DIR}"
  )
endif()

install(TARGETS scout ground_vehicle_sim_core
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cfg" DESTINATION share/ground_vehicle)
