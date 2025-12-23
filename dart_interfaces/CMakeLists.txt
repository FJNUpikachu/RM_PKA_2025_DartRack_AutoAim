cmake_minimum_required(VERSION 3.10)
project(dart_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Light.msg"
    "msg/SerialReceiveData.msg"
    "msg/SerialSendData.msg"
    "msg/BallisticParams.msg"
    "srv/SetMode.srv"
    DEPENDENCIES 
      builtin_interfaces
      std_msgs
      geometry_msgs
)

# 测试部分必须在 ament_package() 之前
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 正确的 ament_package 调用方式
ament_package()