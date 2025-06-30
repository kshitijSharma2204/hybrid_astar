cmake_minimum_required(VERSION 3.8)
project(hybrid_astar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ompl REQUIRED)

# make headers in include/ visible
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# C++ executable: Hybrid A* planner node
add_executable(hybrid_astar_node 
  src/main.cpp
  src/planner.cpp
  src/hybrid_astar_node.cpp
)

ament_target_dependencies(
  hybrid_astar_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ompl
)

target_link_libraries(
  hybrid_astar_node
  ompl::ompl
)

# Install the C++ node executable
install(
  TARGETS hybrid_astar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python environment publisher/visualizer script
install(
  PROGRAMS 
  scripts/env_publisher.py
  scripts/path_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright checks until you add licenses
  set(ament_cmake_copyright_FOUND TRUE)
  # skip cpplint until you add licenses
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
