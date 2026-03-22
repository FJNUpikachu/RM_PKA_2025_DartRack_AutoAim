#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "dart_interfaces/msg/light.hpp"
#include "dart_detector/onnx_detector.hpp"

namespace pka
{

class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
  // ===== ROS =====
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<dart_interfaces::msg::Light>::SharedPtr light_pub_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher raw_image_pub_;
  image_transport::Publisher result_image_pub_;
  bool image_transport_initialized_;

  // ===== 参数 =====
  std::string image_topic_;

  std::string model_path_;
  std::vector<std::string> class_names_;
  int input_width_;
  int input_height_;
  float conf_threshold_;
  float iou_threshold_;
  bool use_cuda_;
  bool enable_debug_;

  // ===== 防抖参数 =====
  double max_jump_px_;
  double smooth_alpha_;

  // ===== 状态 =====
  bool has_last_;
  float last_x_;
  float last_y_;
  bool has_printed_image_size_;

  // ===== 模型 =====
  OnnxDetector detector_;

  // ===== 方法 =====
  void declareParameters();
  void readParameters();
  void printParameters();

  void initImageTransport();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  void publishImage(
    image_transport::Publisher & pub,
    const cv::Mat & image,
    const rclcpp::Time & stamp,
    const std::string & encoding);
};

}  // namespace pka