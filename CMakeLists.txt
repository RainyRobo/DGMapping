# Copyright 2026 The DGMapping Authors. Licensed under the Apache License, 2.0.
#
# Build entry point for the DGMapping algorithmic-core release.
# It builds two standalone, self-contained demos:
#
#   * so_icp_demo            - exercises include/dgmapping/so_icp.h
#   * degeneracy_detector_demo - exercises include/dgmapping/degeneracy_detector.h
#
# Both depend ONLY on Eigen 3 and a C++17 compiler.  The patched scan
# matcher in src/real_time_correlative_scan_matcher_2d.cc requires the
# rest of Cartographer to compile and is therefore NOT built by default.
# Pass -DDGMAPPING_BUILD_SCAN_MATCHER=ON together with a full Cartographer
# source tree under cartographer/ to build it.

cmake_minimum_required(VERSION 3.13)
project(dgmapping LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DGMAPPING_BUILD_SCAN_MATCHER
       "Build the patched RealTimeCorrelativeScanMatcher2D (requires Cartographer)"
       OFF)

# ---------------------------------------------------------------------------
# Eigen 3
# ---------------------------------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  # Fallback: Eigen distributions that do not ship a CMake config (e.g. the
  # Debian/Ubuntu libeigen3-dev package on older distros).
  find_path(EIGEN3_INCLUDE_DIR Eigen/Core
            PATHS /usr/include/eigen3 /usr/local/include/eigen3
                  /opt/homebrew/include/eigen3
            DOC "Path to Eigen 3 headers")
  if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR
            "Eigen 3 not found.  Install it via your package manager "
            "(e.g. `sudo apt-get install libeigen3-dev`) or pass "
            "-DEIGEN3_INCLUDE_DIR=/path/to/eigen3.")
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE "${EIGEN3_INCLUDE_DIR}")
endif()

# ---------------------------------------------------------------------------
# Header-only core
# ---------------------------------------------------------------------------
add_library(dgmapping INTERFACE)
target_include_directories(dgmapping INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dgmapping INTERFACE Eigen3::Eigen)
target_compile_features(dgmapping INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# Demos
# ---------------------------------------------------------------------------
add_executable(so_icp_demo src/so_icp_demo.cc)
target_link_libraries(so_icp_demo PRIVATE dgmapping)

add_executable(degeneracy_detector_demo src/degeneracy_detector_demo.cc)
target_link_libraries(degeneracy_detector_demo PRIVATE dgmapping)

if(MSVC)
  target_compile_options(so_icp_demo               PRIVATE /W3)
  target_compile_options(degeneracy_detector_demo  PRIVATE /W3)
else()
  target_compile_options(so_icp_demo               PRIVATE -Wall -Wextra)
  target_compile_options(degeneracy_detector_demo  PRIVATE -Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# Optional: patched scan matcher (needs Cartographer)
# ---------------------------------------------------------------------------
if(DGMAPPING_BUILD_SCAN_MATCHER)
  find_package(cartographer QUIET)
  if(NOT cartographer_FOUND)
    message(FATAL_ERROR
            "DGMAPPING_BUILD_SCAN_MATCHER=ON but cartographer was not found "
            "via find_package.  Install / build Google Cartographer first "
            "(see https://github.com/cartographer-project/cartographer).")
  endif()
  add_library(dgmapping_scan_matcher
              src/real_time_correlative_scan_matcher_2d.cc)
  target_link_libraries(dgmapping_scan_matcher
                        PUBLIC dgmapping cartographer)
endif()

install(DIRECTORY include/dgmapping DESTINATION include)
install(FILES LICENSE README.md DESTINATION share/dgmapping)
