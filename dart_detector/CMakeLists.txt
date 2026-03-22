cmake_minimum_required(VERSION 3.8)
project(dart_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# ROS2
# ======================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dart_interfaces REQUIRED)

# ======================
# ONNX Runtime 手动路径
# ======================

set(ONNXRUNTIME_ROOT /opt/onnxruntime)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_ROOT}/include
)

link_directories(
  ${ONNXRUNTIME_ROOT}/lib
)

# ======================
# library
# ======================

add_library(${PROJECT_NAME} SHARED
  src/dart_detector_node.cpp
  src/onnx_detector.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  OpenCV
  dart_interfaces
)

target_link_libraries(${PROJECT_NAME}
  onnxruntime
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  "pka::DartDetectorNode"
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
