cmake_minimum_required(VERSION 3.8)
project(dart_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(dart_utils REQUIRED)

# 编译节点库
add_library(dart_detector_core SHARED
  src/dart_detector_node.cpp
  src/detect_method.cpp
)

# 为库目标设置包含目录
target_include_directories(dart_detector_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(dart_detector_core
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  rclcpp_components
  std_srvs
  dart_interfaces
  dart_utils
)

# 注册组件
rclcpp_components_register_nodes(dart_detector_core 
  pka::DartDetectorNode
)

# 测试部分（如果需要）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装规则
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

# 显式安装package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 正确的 ament_package 调用方式
ament_package()