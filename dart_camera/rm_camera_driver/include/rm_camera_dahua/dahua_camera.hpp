#ifndef RM_CAMERA_DAHUA__DAHUA_CAMERA_HPP_
#define RM_CAMERA_DAHUA__DAHUA_CAMERA_HPP_

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

// Project
#include "rm_camera_dahua/Camera_Control.h"
#include "dart_utils/heartbeat.hpp"

namespace pka::camera_driver
{

class Dahua_CameraNode : public rclcpp::Node
{
public:
  explicit Dahua_CameraNode(const rclcpp::NodeOptions &options);
  ~Dahua_CameraNode() override;

private:
  void declareParameters();
  void setupCamera();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  // 相机控制对象
  Camera_Control camera;
  
  // ROS2相关成员
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  
  // 心跳
  HeartBeatPublisher::SharedPtr heartbeat_;
  
  // 捕获线程
  std::thread capture_thread_;
  
  // 参数变量
  double exposure_time_;
  int brightness_;
  double gain_;
  int auto_white_balance_;
  double frame_rate_;
  int resolution_width_;
  int resolution_height_;
  int offset_x_;
  int offset_y_;
  
  std::string camera_name_;
};

}  // namespace pka::camera_driver

#endif  // RM_CAMERA_DAHUA__DAHUA_CAMERA_HPP_