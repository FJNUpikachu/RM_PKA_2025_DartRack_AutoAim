# ros2_hik_camera

A ROS2 packge for Hikvision USB3.0 industrial camera

## Usage

```
ros2 launch hik_camera hik_camera.launch.py
```

## Params

- exposure_time
- gain

## 发布话题和订阅话题
```cpp
#include <image_transport/image_transport.hpp>

// 使用 sensor_data_qos 配置
bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;

image_transport::CameraPublisher camera_pub_;
// 发布图像信息
camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
```
```cpp
img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this,
                std::placeholders::_1));
```