cmake_minimum_required(VERSION 3.16)
project(gait_core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(GAIT_CORE_TOP_LEVEL ON)
else()
  set(GAIT_CORE_TOP_LEVEL OFF)
endif()

# Eigen is very slow unoptimised, so default to Release when building this repo on its own
get_property(GAIT_CORE_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GAIT_CORE_TOP_LEVEL AND NOT GAIT_CORE_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GAIT_CORE_USE_DOUBLE "Use double instead of float for Scalar (the ROS build turns this on)" OFF)
option(GAIT_CORE_BUNDLED_EIGEN "Always use third_party/eigen instead of looking for a system Eigen" OFF)
option(GAIT_CORE_BUILD_TESTS "Build the GoogleTest suite" ${GAIT_CORE_TOP_LEVEL})

# ---------------------------------------------------------------------------
# Eigen: use the system copy when there is one (ROS, so every package in the process shares one
# Eigen version), otherwise the pinned submodule (Pico SDK, PlatformIO, Windows)
# ---------------------------------------------------------------------------
set(GAIT_CORE_EIGEN_FROM_SYSTEM OFF)
if(NOT TARGET Eigen3::Eigen AND NOT GAIT_CORE_BUNDLED_EIGEN AND NOT CMAKE_CROSSCOMPILING)
  find_package(Eigen3 3.3 QUIET NO_MODULE)
  if(TARGET Eigen3::Eigen)
    set(GAIT_CORE_EIGEN_FROM_SYSTEM ON)
  endif()
endif()

if(NOT TARGET Eigen3::Eigen)
  set(GAIT_CORE_EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
  if(NOT EXISTS ${GAIT_CORE_EIGEN_DIR}/Eigen/Dense)
    message(FATAL_ERROR "Eigen not found. Run: git submodule update --init --recursive")
  endif()
  add_library(gait_core_eigen INTERFACE)
  target_include_directories(gait_core_eigen SYSTEM INTERFACE
    $<BUILD_INTERFACE:${GAIT_CORE_EIGEN_DIR}>
    $<INSTALL_INTERFACE:include/gait_core/third_party/eigen>
  )
  set(GAIT_CORE_EIGEN_TARGET gait_core_eigen)
else()
  set(GAIT_CORE_EIGEN_TARGET Eigen3::Eigen)
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(gait_core STATIC
  src/gait_config.cpp
  src/kinematic_solver.cpp
  src/leg_odometry.cpp
  src/robot_geometry.cpp
  src/trajectory_generator.cpp
)
add_library(gait_core::gait_core ALIAS gait_core)

target_include_directories(gait_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gait_core PUBLIC ${GAIT_CORE_EIGEN_TARGET})
target_compile_features(gait_core PUBLIC cxx_std_17)

if(GAIT_CORE_USE_DOUBLE)
  target_compile_definitions(gait_core PUBLIC GAIT_CORE_SCALAR=double)
endif()

if(MSVC)
  target_compile_definitions(gait_core PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif()

# Linked into ROS shared libraries, so it must be position independent there
if(NOT CMAKE_CROSSCOMPILING)
  set_target_properties(gait_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(GAIT_CORE_TOP_LEVEL)
  if(MSVC)
    target_compile_options(gait_core PRIVATE /W4)
  else()
    target_compile_options(gait_core PRIVATE -Wall -Wextra -Wpedantic -Wdouble-promotion)
  endif()
endif()

# ---------------------------------------------------------------------------
# Install + CMake package config, so colcon/ROS can find_package(gait_core)
# ---------------------------------------------------------------------------
if(GAIT_CORE_TOP_LEVEL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  set(GAIT_CORE_EXPORT_TARGETS gait_core)
  if(TARGET gait_core_eigen)
    list(APPEND GAIT_CORE_EXPORT_TARGETS gait_core_eigen)
    install(DIRECTORY ${GAIT_CORE_EIGEN_DIR}/Eigen
            DESTINATION include/gait_core/third_party/eigen)
  endif()

  install(TARGETS ${GAIT_CORE_EXPORT_TARGETS}
    EXPORT gait_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION include
  )
  install(DIRECTORY include/ DESTINATION include)
  install(EXPORT gait_coreTargets
    NAMESPACE gait_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  configure_package_config_file(cmake/gait_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfigVersion.cmake
    COMPATIBILITY SameMinorVersion
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  install(FILES package.xml DESTINATION share/gait_core)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(GAIT_CORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
