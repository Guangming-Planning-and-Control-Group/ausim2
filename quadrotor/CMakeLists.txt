cmake_minimum_required(VERSION 3.20)

project(quadrotor_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

find_package(mujoco CONFIG QUIET)
if(NOT mujoco_FOUND)
  set(_quadrotor_mujoco_hints)
  if(DEFINED mujoco_DIR)
    list(APPEND _quadrotor_mujoco_hints "${mujoco_DIR}")
  endif()
  if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND _quadrotor_mujoco_hints "$ENV{MUJOCO_DIR}")
  endif()
  if(DEFINED ENV{MUJOCO_ROOT})
    list(APPEND _quadrotor_mujoco_hints "$ENV{MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}/build")
  endif()
  list(APPEND _quadrotor_mujoco_hints
       "/home/x/mujoco/mujoco-3.3.7/build"
       "/usr/local/lib/cmake/mujoco"
       "/usr/lib/cmake/mujoco")
  find_package(mujoco CONFIG REQUIRED PATHS ${_quadrotor_mujoco_hints})
endif()

set(QUADROTOR_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")

add_library(quadrotor_mujoco_viewer STATIC
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/simulate.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/platform_ui_adapter.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/glfw_adapter.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/glfw_dispatch.cc"
)

target_include_directories(quadrotor_mujoco_viewer
  PUBLIC
    "${QUADROTOR_THIRD_PARTY_DIR}"
)

target_compile_features(quadrotor_mujoco_viewer PUBLIC cxx_std_17)

target_link_libraries(quadrotor_mujoco_viewer
  PUBLIC
    glfw
    mujoco::mujoco
    "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/liblodepng.a"
)

add_library(quadrotor_core STATIC
  src/se3_controller.cpp
  src/motor_mixer.cpp
  src/geometry.cpp
  src/quadrotor_sim.cpp
)

target_include_directories(quadrotor_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_features(quadrotor_core PUBLIC cxx_std_17)
target_compile_definitions(quadrotor_core
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(quadrotor_core
  PUBLIC
    Eigen3::Eigen
    yaml-cpp
    quadrotor_mujoco_viewer
    mujoco::mujoco
)

add_executable(quadrotor
  src/main.cpp
)

target_link_libraries(quadrotor PRIVATE quadrotor_core)

get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_RELEASE)
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_RELWITHDEBINFO)
endif()
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_DEBUG)
endif()
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION)
endif()
if(_quadrotor_mujoco_lib)
  get_filename_component(_quadrotor_mujoco_dir "${_quadrotor_mujoco_lib}" DIRECTORY)
  set_target_properties(quadrotor PROPERTIES
    BUILD_RPATH "${_quadrotor_mujoco_dir}"
    INSTALL_RPATH "${_quadrotor_mujoco_dir}"
  )
endif()

install(TARGETS quadrotor quadrotor_core quadrotor_mujoco_viewer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/assets" DESTINATION share)
install(FILES README.md config.yaml DESTINATION share/quadrotor)
