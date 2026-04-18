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
#include "dart_interfaces/msg/light.hpp"

namespace pka
{

// detector 节点：
// 负责接收图像、调用 ONNX 模型、选取当前帧最可信的灯、发布 light_position
class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
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
  std::string model_path_;
  std::vector<std::string> class_names_;
  int input_width_;
  int input_height_;
  float conf_threshold_;
  float iou_threshold_;
  bool use_cuda_;
  bool enable_debug_;

  // ===== 新增：尽量贴近 2025 传统 detector 的约束 =====
  int roi_y_min_;
  int roi_y_max_;
  bool prefer_previous_target_;
  double association_max_distance_px_;

  bool has_printed_image_size_;

  // ONNX 检测器
  OnnxDetector detector_;

  // ===== 新增：上一帧已选目标中心，用于“同一目标连续优先” =====
  bool has_last_target_;
  double last_target_x_;
  double last_target_y_;

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