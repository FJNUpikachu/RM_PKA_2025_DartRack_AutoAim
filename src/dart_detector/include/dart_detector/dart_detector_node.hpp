#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "dart_detector/onnx_detector.hpp"
#include "dart_detector/traditional_detect_method.hpp"
#include "dart_interfaces/msg/light.hpp"

namespace pka
{

// detector 节点：
// 通过参数选择 backend（1=onnx, 2=traditional），对图像进行检测并发布 light_position
class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
  // backend: 1=onnx，2=traditional
  int backend_mode_;

  // ROS2 订阅与发布
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
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

  // ONNX 输出低通滤波参数：只作用于神经网络 backend 的 light_position 输出
  bool onnx_lpf_enable_;
  double onnx_lpf_alpha_;
  double onnx_lpf_max_jump_px_;

  // ONNX 低通滤波状态
  bool onnx_lpf_initialized_;
  double onnx_lpf_x_;
  double onnx_lpf_y_;

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

  // camera_info 主点：用于 UI 中轴线、dx/dy 和多候选框靠近中心的判断
  bool camera_info_received_;
  double camera_cx_;
  double camera_cy_;

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
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void processLatestFrame();

  bool isOnnxBackend() const;
  bool isTraditionalBackend() const;
  std::string backendName() const;

  void publishImage(
    image_transport::Publisher & pub,
    const cv::Mat & image,
    const rclcpp::Time & stamp,
    const std::string & encoding);
};

}  // namespace pka