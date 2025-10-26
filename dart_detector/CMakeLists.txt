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

# 包含头文件目录
include_directories(include)

# 编译节点库
add_library(dart_detector_core SHARED
  src/dart_detector_node.cpp
  src/detect_method.cpp
  src/screenshot_manager.cpp
  src/video_recorder.cpp
)
ament_target_dependencies(dart_detector_core
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp_components"
  "std_srvs"
)

# 注册组件
rclcpp_components_register_node(dart_detector_core 
  PLUGIN "dart_detector::DartDetectorNode" 
  EXECUTABLE dart_detector_node
)

# 安装规则
install(TARGETS
  dart_detector_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

# 安装可执行文件
install(TARGETS
  dart_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(dart_detector_core)
ament_export_dependencies(
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV 
  rclcpp_components 
  std_srvs
)

ament_package()
