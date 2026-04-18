#include "dart_detector/dart_detector_node.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>

namespace pka
{

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions & options)
: Node("dart_detector", options),
  image_transport_initialized_(false),
  processing_(false),
  roi_y_min_(0),
  roi_y_max_(0),
  prefer_previous_target_(true),
  association_max_distance_px_(80.0),
  has_printed_image_size_(false),
  has_last_target_(false),
  last_target_x_(0.0),
  last_target_y_(0.0)
{
  declareParameters();
  readParameters();

  try {
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
  } catch (const cv::Exception & e) {
    RCLCPP_FATAL(get_logger(), "OpenCV 初始化 detector 失败: %s", e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "detector 初始化失败: %s", e.what());
    throw;
  } catch (...) {
    RCLCPP_FATAL(get_logger(), "detector 初始化失败: 未知异常");
    throw;
  }

  if (use_cuda_) {
    RCLCPP_WARN(get_logger(), "当前版本固定使用 CPU 推理，use_cuda 参数将被忽略。");
  }

  printParameters();

  // 只保留最新一帧，避免积压旧帧
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    image_qos,
    std::bind(&DartDetectorNode::imageCallback, this, std::placeholders::_1));

  light_pub_ = this->create_publisher<dart_interfaces::msg::Light>(
    "light_position", 10);

  // 独立定时器处理最新图像，不在订阅回调里直接推理
  process_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&DartDetectorNode::processLatestFrame, this));

  RCLCPP_INFO(get_logger(), "引导灯 detector 已启动（latest-frame-only 模式）");
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

  // ===== 新增 =====
  declare_parameter("roi_y_min", 0);
  declare_parameter("roi_y_max", 0);
  declare_parameter("prefer_previous_target", true);
  declare_parameter("association_max_distance_px", 80.0);
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

  // ===== 新增 =====
  roi_y_min_ = get_parameter("roi_y_min").as_int();
  roi_y_max_ = get_parameter("roi_y_max").as_int();
  prefer_previous_target_ = get_parameter("prefer_previous_target").as_bool();
  association_max_distance_px_ =
    get_parameter("association_max_distance_px").as_double();
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
  RCLCPP_INFO(get_logger(), "roi_y_min: %d", roi_y_min_);
  RCLCPP_INFO(get_logger(), "roi_y_max: %d", roi_y_max_);
  RCLCPP_INFO(get_logger(), "prefer_previous_target: %s", prefer_previous_target_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "association_max_distance_px: %.1f", association_max_distance_px_);
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
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_msg_ = msg;
}

void DartDetectorNode::processLatestFrame()
{
  if (!image_transport_initialized_) {
    initImageTransport();
  }

  sensor_msgs::msg::Image::ConstSharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (processing_ || !latest_image_msg_) {
      return;
    }
    processing_ = true;
    msg = latest_image_msg_;
    latest_image_msg_.reset();
  }

  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
    std::lock_guard<std::mutex> lock(image_mutex_);
    processing_ = false;
    return;
  }

  if (image.empty()) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    processing_ = false;
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

  if (image_transport_initialized_) {
    publishImage(raw_image_pub_, image, msg->header.stamp, "bgr8");
  }

  std::vector<Detection> detections;
  try {
    detections = detector_.infer(image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "detector 推理失败: %s", e.what());
    std::lock_guard<std::mutex> lock(image_mutex_);
    processing_ = false;
    return;
  }

  const int img_cx = image.cols / 2;
  const int img_cy = image.rows / 2;

  bool detected = false;
  int best_idx = -1;
  double center_x = -1.0;
  double center_y = -1.0;

  // ===== 第一步：ROI 过滤 =====
  std::vector<int> valid_indices;
  valid_indices.reserve(detections.size());

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto & d = detections[i];
    const double cy = d.box.y + d.box.height * 0.5;

    bool pass_roi = true;
    if (roi_y_min_ < roi_y_max_) {
      if (cy < static_cast<double>(roi_y_min_) || cy > static_cast<double>(roi_y_max_)) {
        pass_roi = false;
      }
    }

    if (pass_roi) {
      valid_indices.push_back(static_cast<int>(i));
    }
  }

  // 如果 ROI 过滤后一个都没有，就退回全体 detections，避免参数设坏直接完全失明
  if (valid_indices.empty()) {
    for (size_t i = 0; i < detections.size(); ++i) {
      valid_indices.push_back(static_cast<int>(i));
    }
  }

  // ===== 第二步：如果上一帧已有目标，则优先选“离上一帧最近”的候选 =====
  if (prefer_previous_target_ && has_last_target_) {
    float best_score = -1.0f;
    double best_dist = std::numeric_limits<double>::max();

    for (int idx : valid_indices) {
      const auto & d = detections[idx];
      const double cx = d.box.x + d.box.width * 0.5;
      const double cy = d.box.y + d.box.height * 0.5;
      const double dist = std::hypot(cx - last_target_x_, cy - last_target_y_);

      if (dist <= association_max_distance_px_) {
        if (d.score > best_score + 1e-6f) {
          best_score = d.score;
          best_dist = dist;
          best_idx = idx;
        } else if (std::fabs(d.score - best_score) < 0.02f && dist < best_dist) {
          best_dist = dist;
          best_idx = idx;
        }
      }
    }
  }

  // ===== 第三步：如果没有关联到上一帧目标，就回退到原来的选法 =====
  if (best_idx < 0) {
    float best_score = -1.0f;
    double best_center_dist = std::numeric_limits<double>::max();

    for (int idx : valid_indices) {
      const auto & d = detections[idx];
      const double cx = d.box.x + d.box.width * 0.5;
      const double cy = d.box.y + d.box.height * 0.5;
      const double dist = std::hypot(cx - img_cx, cy - img_cy);

      if (d.score > best_score + 1e-6f) {
        best_score = d.score;
        best_center_dist = dist;
        best_idx = idx;
      } else if (std::fabs(d.score - best_score) < 0.02f && dist < best_center_dist) {
        best_center_dist = dist;
        best_idx = idx;
      }
    }
  }

  if (best_idx >= 0) {
    const auto & d = detections[best_idx];
    center_x = d.box.x + d.box.width * 0.5;
    center_y = d.box.y + d.box.height * 0.5;
    detected = true;

    last_target_x_ = center_x;
    last_target_y_ = center_y;
    has_last_target_ = true;
  }

  // 未检测到时直接发 -1，不做保持
  dart_interfaces::msg::Light out;
  out.x = detected ? center_x : -1.0;
  out.y = detected ? center_y : -1.0;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;
  light_pub_->publish(out);

  if (image_transport_initialized_) {
    try {
      cv::Mat vis = image.clone();

      static auto last_time = std::chrono::steady_clock::now();
      static double fps = 0.0;
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last_time).count();
      if (dt > 1e-6) {
        fps = 1.0 / dt;
      }
      last_time = now;

      // 图像中轴线
      cv::line(vis, cv::Point(img_cx, 0), cv::Point(img_cx, vis.rows - 1), cv::Scalar(0, 0, 255), 2);
      cv::line(vis, cv::Point(0, img_cy), cv::Point(vis.cols - 1, img_cy), cv::Scalar(0, 0, 255), 2);

      cv::putText(
        vis,
        "Center",
        cv::Point(std::min(img_cx + 8, vis.cols - 80), std::max(img_cy - 8, 20)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        2);

      cv::putText(
        vis,
        "FPS: " + cv::format("%.0f", fps),
        cv::Point(std::max(10, vis.cols - 110), 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(255, 255, 255),
        2);

      if (roi_y_min_ < roi_y_max_) {
        cv::rectangle(
          vis,
          cv::Point(0, roi_y_min_),
          cv::Point(vis.cols - 1, roi_y_max_),
          cv::Scalar(0, 255, 0),
          2);
        cv::putText(
          vis,
          "ROI",
          cv::Point(10, std::max(20, roi_y_min_ - 8)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(0, 255, 0),
          2);
      }

      if (detected && best_idx >= 0) {
        const auto & d = detections[best_idx];

        cv::Rect draw_box(
          static_cast<int>(d.box.x),
          static_cast<int>(d.box.y),
          static_cast<int>(d.box.width),
          static_cast<int>(d.box.height));

        cv::Point det_center(
          static_cast<int>(center_x),
          static_cast<int>(center_y));

        cv::rectangle(vis, draw_box, cv::Scalar(0, 0, 255), 2);

        std::string label = d.class_name + " " + cv::format("%.2f", d.score);
        cv::putText(
          vis,
          label,
          cv::Point(draw_box.x, std::max(20, draw_box.y - 8)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(0, 0, 255),
          2);

        cv::circle(vis, det_center, 4, cv::Scalar(255, 0, 0), -1);
        cv::line(vis, cv::Point(img_cx, img_cy), det_center, cv::Scalar(255, 0, 0), 2);

        const double dx = center_x - static_cast<double>(img_cx);
        const double dy = static_cast<double>(img_cy) - center_y;
        std::string pos_text =
          "x=" + cv::format("%.4f", center_x) +
          " y=" + cv::format("%.4f", center_y) +
          " dx=" + cv::format("%.1f", dx) +
          " dy=" + cv::format("%.1f", dy);

        cv::putText(
          vis,
          pos_text,
          cv::Point(10, vis.rows - 20),
          cv::FONT_HERSHEY_SIMPLEX,
          0.65,
          cv::Scalar(255, 255, 255),
          2);
      } else {
        cv::putText(
          vis,
          "No light",
          cv::Point(10, vis.rows - 20),
          cv::FONT_HERSHEY_SIMPLEX,
          0.65,
          cv::Scalar(0, 0, 255),
          2);
      }

      publishImage(result_image_pub_, vis, msg->header.stamp, "bgr8");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "调试图像发布失败: %s", e.what());
    }
  }

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    processing_ = false;
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

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = "camera";

  auto msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
  pub.publish(msg);
}

}  // namespace pka

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pka::DartDetectorNode)