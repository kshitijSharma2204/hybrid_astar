cmake_minimum_required(VERSION 3.16)
project(hybrid_astar CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# I use the variables from omplConfig.cmake, not the ompl::ompl target.
# On some machines the target and the variables point at different OMPL
# versions.
find_package(ompl REQUIRED)

# Sources are listed by hand so the pure pursuit work in src/ is not built.
add_library(hybrid_astar_core STATIC
  src/collision.cpp
  src/config.cpp
  src/metrics.cpp
  src/planner.cpp
  src/reeds_shepp.cpp
  src/scenario.cpp
)
target_include_directories(hybrid_astar_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(hybrid_astar_core PUBLIC ${OMPL_LIBRARIES})

add_executable(hybrid_astar_cli src/cli.cpp)
target_link_libraries(hybrid_astar_cli PRIVATE hybrid_astar_core)

option(BUILD_TESTING "Build the unit tests" ON)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(unit_tests
    tests/test_collision.cpp
    tests/test_config.cpp
    tests/test_geometry.cpp
    tests/test_metrics.cpp
    tests/test_planner.cpp
    tests/test_reeds_shepp.cpp
  )
  target_link_libraries(unit_tests PRIVATE hybrid_astar_core GTest::gtest_main)
  target_compile_definitions(unit_tests PRIVATE HA_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(unit_tests)
endif()

option(BUILD_PYTHON "Build the hybrid_astar Python module" ON)
if(BUILD_PYTHON)
  include(FetchContent)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE
  )
  set(PYBIND11_FINDPYTHON ON)
  FetchContent_MakeAvailable(pybind11)
  pybind11_add_module(hybrid_astar_py python/bindings.cpp)
  set_target_properties(hybrid_astar_py PROPERTIES
    OUTPUT_NAME hybrid_astar
    # next to hybrid_astar_viz, so one path entry finds both
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python
  )
  target_link_libraries(hybrid_astar_py PRIVATE hybrid_astar_core)
endif()
