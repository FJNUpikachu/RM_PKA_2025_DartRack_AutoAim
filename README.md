# 【施工中！】
# 工程详见release

# Pikachu战队2026 DartTrack Vision Project

 本项目为福建师范大学Pikachu战队2025赛季飞镖镖架制导视觉部分。


## 一、项目结构

```
.
│
├── dart_detector : 识别部分
|
├── dart_solver : 解算部分
|
├── dart_bringup : 启动及参数文件
|
├── dart_interfaces : 自定义msg、srv
│
├── dart_serial : 与电控通信串口部分
│
└──dart_utils (工具包) 
```

## 二、环境配置


## 三、编译与运行

rm -rf build install log
colcon build --symlink-install --parallel-workers 2

source install/setup.bash
ros2 launch dart_bringup bringup.launch.py

source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

## 维护者及开源许可证


## 致谢






# 调试专用

## 编译及运行
 colcon build
 source install/setup.bash
 ros2 launch demo_bringup demo.launch.py


## 消息接口测试
colcon build --packages-select dart_interfaces
ros2 interface show dart_interfaces/msg/Light

