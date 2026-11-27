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
  src/planner.cpp
  src/scenario.cpp
)
target_include_directories(hybrid_astar_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(hybrid_astar_core PUBLIC ${OMPL_LIBRARIES})

add_executable(hybrid_astar_cli src/cli.cpp)
target_link_libraries(hybrid_astar_cli PRIVATE hybrid_astar_core)
