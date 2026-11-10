# SPDX-License-Identifier: Apache-2.0
add_library(sgs_core STATIC
  src/core/math.cpp
  src/core/pid.cpp
  src/core/defaults.cpp
  src/core/complementary.cpp
  src/core/ekf.cpp
  src/core/controller.cpp
)
target_include_directories(sgs_core PUBLIC include)
target_compile_options(sgs_core PRIVATE -fno-exceptions -fno-rtti)
set_target_properties(sgs_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(sgs_sim STATIC
  src/sim/plant.cpp
  src/sim/sensor.cpp
  src/sim/scenario.cpp
  src/sim/rng.cpp
)
target_include_directories(sgs_sim PUBLIC include)
target_link_libraries(sgs_sim PUBLIC sgs_core)

if(SGS_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG 063de7e9578f82b369302001269680b4b1553359
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(sgs_tests
    test/test_math.cpp
    test/test_pid.cpp
    test/test_complementary.cpp
    test/test_ekf.cpp
    test/test_jacobian.cpp
    test/test_controller.cpp
    test/test_scenario.cpp
  )
  add_test(NAME sgs_gtests COMMAND sgs_tests)
  set_tests_properties(sgs_gtests PROPERTIES TIMEOUT 30)
  target_link_libraries(sgs_tests PRIVATE sgs_core sgs_sim GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(sgs_tests PROPERTIES TIMEOUT 30)

  add_executable(noheap_link tools/noheap_link.cpp)
  target_link_libraries(noheap_link PRIVATE sgs_core)
  target_compile_options(noheap_link PRIVATE -fno-exceptions -fno-rtti)
  add_test(NAME noheap_link COMMAND noheap_link)
  set_tests_properties(noheap_link PROPERTIES TIMEOUT 2)
endif()

if(SGS_BUILD_WASM)
  add_executable(sgs_wasm src/wasm/wasm_api.cpp)
  target_link_libraries(sgs_wasm PRIVATE sgs_core sgs_sim)
  set_target_properties(sgs_wasm PROPERTIES OUTPUT_NAME "sgs")
  target_link_options(sgs_wasm PRIVATE
    "SHELL:-s MODULARIZE=1"
    "SHELL:-s EXPORT_ES6=1"
    "SHELL:-s EXPORT_NAME=createSgsModule"
    "SHELL:-s EXPORTED_FUNCTIONS=['_sgs_reset','_sgs_set_gains','_sgs_step_ms','_sgs_trace_ptr','_sgs_trace_len','_sgs_trace_stride','_sgs_trace_capacity','_sgs_set_estimator','_sgs_pitch_step']"
    "SHELL:-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPF32']"
    "SHELL:-s ALLOW_MEMORY_GROWTH=0"
    "SHELL:-s ENVIRONMENT=web"
  )
endif()

if(SGS_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG 97bf890db679505a14dfe547a5e77bb2bd05dc90
  )
  FetchContent_MakeAvailable(pybind11)
  pybind11_add_module(sgs_core_py src/python/bindings.cpp)
  target_link_libraries(sgs_core_py PRIVATE sgs_core sgs_sim)
  set_target_properties(sgs_core_py PROPERTIES OUTPUT_NAME sgs_core LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_BINARY_DIR}/python")
endif()
