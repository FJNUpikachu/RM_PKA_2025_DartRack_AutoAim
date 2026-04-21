cmake_minimum_required(VERSION 3.8)
project(dart_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 基础依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dart_interfaces REQUIRED)

# ===== ONNX Runtime 路径 =====
# 这里沿用你之前已经能跑起来的 onnxruntime 安装位置
set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "Path to ONNX Runtime installation")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS ${ONNXRUNTIME_LIB_DIR}
  REQUIRED
)

# ===== detector 组件库 =====
# 这里把 ONNX 版和传统版都编进同一个组件库里
add_library(dart_detector_core SHARED
  src/dart_detector_node.cpp
  src/onnx_detector.cpp
  src/traditional_detect_method.cpp
)

target_include_directories(dart_detector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_INCLUDE_DIR}
)

ament_target_dependencies(dart_detector_core
  rclcpp
  rclcpp_components
  ament_index_cpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  dart_interfaces
)

target_link_libraries(dart_detector_core
  ${ONNXRUNTIME_LIB}
)

# ===== 注册两个组件 =====
# 1. 当前 ONNX detector
# 2. 传统识别 detector
rclcpp_components_register_nodes(dart_detector_core
  "pka::DartDetectorNode"
)

# ===== 安装 =====
install(TARGETS
  dart_detector_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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