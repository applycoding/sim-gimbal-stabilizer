# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(sim-gimbal-stabilizer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SGS_BUILD_TESTS "Build GoogleTest suite" ON)
option(SGS_BUILD_PYTHON "Build pybind11 module" OFF)
option(SGS_BUILD_ROS "Build ROS 2 wrapper" OFF)
option(SGS_BUILD_WASM "Build Emscripten WASM module" OFF)

if(SGS_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(cpp)
