#include "dart_detector/dart_detector_node.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include <chrono>
#include <cmath>

namespace pka
{

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions & options)
: Node("dart_detector", options),
  image_transport_initialized_(false),
  has_last_(false),
  last_x_(0.0f),
  last_y_(0.0f),
  has_printed_image_size_(false)
{
  declareParameters();
  readParameters();


  if (!detector_.init(
      model_path_,
      class_names_,
      input_width_,
      input_height_,
      conf_threshold_,
      iou_threshold_))
  {
    RCLCPP_FATAL(get_logger(), "ONNX模型加载失败");
    throw std::runtime_error("Detector init failed");
  }

  if (use_cuda_) {
    RCLCPP_WARN(get_logger(), "当前版本固定使用 CPU 推理，use_cuda 参数将被忽略。");
  }

  printParameters();

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    10,
    std::bind(&DartDetectorNode::imageCallback, this, std::placeholders::_1));

  light_pub_ = this->create_publisher<dart_interfaces::msg::Light>(
    "light_position", 10);

  RCLCPP_INFO(get_logger(), "防抖 detector 已启动");
}

void DartDetectorNode::declareParameters()
{
  declare_parameter("image_topic", "image_raw");

  declare_parameter("model_path", "");
  declare_parameter("class_names", std::vector<std::string>{"light"});
  declare_parameter("input_width", 640);
  declare_parameter("input_height", 640);
  declare_parameter("conf_threshold", 0.25);
  declare_parameter("iou_threshold", 0.45);
  declare_parameter("use_cuda", false);
  declare_parameter("enable_debug", true);

  // 防抖参数
  declare_parameter("max_jump_px", 40.0);
  declare_parameter("smooth_alpha", 0.6);
}

void DartDetectorNode::readParameters()
{
  image_topic_ = get_parameter("image_topic").as_string();

  model_path_ = get_parameter("model_path").as_string();
  class_names_ = get_parameter("class_names").as_string_array();
  input_width_ = get_parameter("input_width").as_int();
  input_height_ = get_parameter("input_height").as_int();
  conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
  iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
  use_cuda_ = get_parameter("use_cuda").as_bool();
  enable_debug_ = get_parameter("enable_debug").as_bool();

  max_jump_px_ = get_parameter("max_jump_px").as_double();
  smooth_alpha_ = get_parameter("smooth_alpha").as_double();
}

void DartDetectorNode::printParameters()
{
  RCLCPP_INFO(get_logger(), "===== detector 参数 =====");
  RCLCPP_INFO(get_logger(), "image_topic: %s", image_topic_.c_str());
  RCLCPP_INFO(get_logger(), "model_path: %s", model_path_.c_str());
  RCLCPP_INFO(get_logger(), "input: %d x %d", input_width_, input_height_);
  RCLCPP_INFO(get_logger(), "conf_threshold: %.2f", conf_threshold_);
  RCLCPP_INFO(get_logger(), "iou_threshold: %.2f", iou_threshold_);
  RCLCPP_INFO(get_logger(), "use_cuda: %s (当前版本忽略，固定 CPU)", use_cuda_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "enable_debug: %s", enable_debug_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "max_jump_px: %.1f", max_jump_px_);
  RCLCPP_INFO(get_logger(), "smooth_alpha: %.2f", smooth_alpha_);
  RCLCPP_INFO(get_logger(), "========================");
}

void DartDetectorNode::initImageTransport()
{
  if (!image_transport_initialized_) {
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    raw_image_pub_ = it_->advertise("raw_image", 10);
    result_image_pub_ = it_->advertise("result_img", 10);
    image_transport_initialized_ = true;
  }
}

void DartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!image_transport_initialized_) {
    initImageTransport();
  }

  cv::Mat image;

  try {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
    return;
  }

  if (image.empty()) {
    return;
  }

  if (enable_debug_ && !has_printed_image_size_) {
    RCLCPP_INFO(
      get_logger(),
      "图像尺寸: %d x %d, channels=%d",
      image.cols,
      image.rows,
      image.channels());
    has_printed_image_size_ = true;
  }

  auto detections = detector_.infer(image);

  float center_x = last_x_;
  float center_y = last_y_;
  bool detected = false;

  if (!detections.empty()) {
    const auto & d = detections.front();

    float new_x = static_cast<float>(d.box.x + d.box.width * 0.5f);
    float new_y = static_cast<float>(d.box.y + d.box.height * 0.5f);

    // 跳变限制
    if (has_last_) {
      if (std::abs(new_x - last_x_) > max_jump_px_ ||
          std::abs(new_y - last_y_) > max_jump_px_) {
        if (enable_debug_) {
          RCLCPP_WARN(get_logger(), "检测中心跳变过大，本帧采用上一帧位置");
        }
        new_x = last_x_;
        new_y = last_y_;
      }
    }

    // 平滑
    if (has_last_) {
      center_x = static_cast<float>(smooth_alpha_ * new_x + (1.0 - smooth_alpha_) * last_x_);
      center_y = static_cast<float>(smooth_alpha_ * new_y + (1.0 - smooth_alpha_) * last_y_);
    } else {
      center_x = new_x;
      center_y = new_y;
    }

    last_x_ = center_x;
    last_y_ = center_y;
    has_last_ = true;
    detected = true;
  }

  // 丢灯保持上一帧
  if (!detected && has_last_) {
    center_x = last_x_;
    center_y = last_y_;
  }

  dart_interfaces::msg::Light out;
  out.x = center_x;
  out.y = center_y;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;

  light_pub_->publish(out);

  if (image_transport_initialized_) {
    cv::Mat vis = image.clone();

    if (detected) {
      const auto & d = detections.front();
      cv::rectangle(vis, d.box, cv::Scalar(0, 0, 255), 2);

      std::string label = d.class_name + " " + cv::format("%.2f", d.score);
      cv::putText(
        vis,
        label,
        cv::Point(d.box.x, std::max(0, d.box.y - 5)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 0, 255),
        2);
    }

    cv::circle(
      vis,
      cv::Point(static_cast<int>(center_x), static_cast<int>(center_y)),
      5,
      cv::Scalar(0, 255, 255),
      -1);

    publishImage(result_image_pub_, vis, msg->header.stamp, "bgr8");
  }
}

void DartDetectorNode::publishImage(
  image_transport::Publisher & pub,
  const cv::Mat & image,
  const rclcpp::Time & stamp,
  const std::string & encoding)
{
  if (image.empty()) {
    return;
  }

  cv_bridge::CvImage img;
  img.header.stamp = stamp;
  img.header.frame_id = "camera";
  img.encoding = encoding;
  img.image = image;
  pub.publish(*img.toImageMsg());
}

}  // namespace pka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::DartDetectorNode)