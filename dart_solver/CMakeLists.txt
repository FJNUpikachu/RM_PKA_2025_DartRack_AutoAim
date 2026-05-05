cmake_minimum_required(VERSION 3.8)
project(dart_solver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 基础依赖 =====
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dart_interfaces REQUIRED)

add_library(dart_solver_component SHARED
  src/solver_node.cpp
  src/solver_basic_method.cpp
  src/filters/solver_one_euro_filter.cpp
  src/filters/solver_ekf_filter.cpp
)

target_include_directories(dart_solver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(dart_solver_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  dart_interfaces
)

target_link_libraries(dart_solver_component
  Ceres::ceres
)

rclcpp_components_register_nodes(dart_solver_component
  "pka::SolverNode"
)

install(TARGETS
  dart_solver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()