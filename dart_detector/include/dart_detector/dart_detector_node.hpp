#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dart_detector/onnx_detector.hpp"
#include "dart_detector/traditional_detect_method.hpp"
#include "dart_interfaces/msg/light.hpp"

namespace pka
{

// detector 节点：
// 通过参数选择 backend（onnx / traditional），对图像进行检测并发布 light_position
class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
  // backend: "onnx" 或 "traditional"
  std::string backend_;

  // ROS2 订阅与发布
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<dart_interfaces::msg::Light>::SharedPtr light_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  // 图像调试发布
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher raw_image_pub_;
  image_transport::Publisher result_image_pub_;
  bool image_transport_initialized_;

  // latest-frame-only 机制
  std::mutex image_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_msg_;
  bool processing_;

  // 参数
  std::string image_topic_;
  bool enable_debug_;

  // ONNX 模式参数
  std::string model_path_;
  std::vector<std::string> class_names_;
  int input_width_;
  int input_height_;
  float conf_threshold_;
  float iou_threshold_;
  bool use_cuda_;

  // Traditional 模式参数
  double green_diff_thresh_;
  double green_abs_thresh_;
  int blur_ksize_;
  int y_min_;
  int y_max_;
  double aspect_ratio_threshold_;
  double circularity_threshold_;
  double min_radius_;
  double max_radius_;

  bool has_printed_image_size_;

  // 运行时统计（调试用）
  size_t frame_count_;
  size_t detect_count_;
  double fps_;
  rclcpp::Time last_frame_time_;
  bool last_detected_;

  // ONNX 检测器
  OnnxDetector detector_;

  // Traditional 检测器
  TraditionalDartDetector traditional_detector_;

  void declareParameters();
  void readParameters();
  void printParameters();

  void initImageTransport();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void processLatestFrame();

  void publishImage(
    image_transport::Publisher & pub,
    const cv::Mat & image,
    const rclcpp::Time & stamp,
    const std::string & encoding);
};

}  // namespace pka