cmake_minimum_required(VERSION 3.5)
project(dart_solver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(dart_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Solver node library
add_library(dart_solver SHARED
  src/solver_node.cpp
  src/solver_method.cpp
  src/one_euro_filter.cpp
  src/ballistic_model.cpp
)

ament_target_dependencies(dart_solver
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  dart_interfaces
  dart_utils
  Eigen3
  OpenCV
)

rclcpp_components_register_nodes(dart_solver "dart_solver::SolverNode")
set(node_plugins "${node_plugins}dart_solver::SolverNode;$<TARGET_FILE:dart_solver>\n")

install(TARGETS
  dart_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dart_solver)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  dart_interfaces
  Eigen3
  OpenCV
)

ament_package()
