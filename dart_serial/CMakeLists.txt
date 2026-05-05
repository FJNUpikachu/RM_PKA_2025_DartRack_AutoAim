cmake_minimum_required(VERSION 3.8)
project(dart_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(dart_utils REQUIRED)

set(SERIAL_SRCS
  src/serial/serial.cc
  src/serial/impl/unix.cc
  src/serial/impl/list_ports/list_ports_linux.cc
)

add_library(${PROJECT_NAME}_component SHARED
  src/uart_node.cpp
  src/uart_node_tool.cpp
  src/protocol/serial_protocol.cpp
  src/protocol/dart_protocol.cpp
  ${SERIAL_SRCS}
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_NAME}_component rt pthread)
endif()

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  dart_interfaces
  dart_utils
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "pka::UARTNode"
)

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  dart_interfaces
  dart_utils
)

ament_package()
