#include "dart_detector/dart_detector_node.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include <chrono>
#include <cmath>
#include <exception>

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
    if (enable_debug_) {
      RCLCPP_WARN(get_logger(), "收到空图像，跳过本帧");
    }
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
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "OpenCV DNN 推理失败: %s", e.what());
    return;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "detector 推理失败: %s", e.what());
    return;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "detector 推理失败: 未知异常");
    return;
  }

  const int img_cx = image.cols / 2;
  const int img_cy = image.rows / 2;

  float center_x = last_x_;
  float center_y = last_y_;
  float raw_center_x = last_x_;
  float raw_center_y = last_y_;
  bool detected = false;

  // 调试显示用
  std::string track_mode = "LOST";
  double adaptive_jump = max_jump_px_;
  double used_alpha_x = smooth_alpha_;

  if (!detections.empty()) {
    const auto & d = detections.front();

    float new_x = static_cast<float>(d.box.x + d.box.width * 0.5f);
    float new_y = static_cast<float>(d.box.y + d.box.height * 0.5f);
    raw_center_x = new_x;
    raw_center_y = new_y;

    // ===== 只按 x 方向判断 =====
    const float dx_center = new_x - static_cast<float>(img_cx);
    const float abs_dx = std::abs(dx_center);

    // x方向状态切换
    if (abs_dx > 80.0f) {
      track_mode = "TRACKING";
      adaptive_jump = std::max(max_jump_px_, 220.0);
      used_alpha_x = 0.85;   // 快速跟随
    } else if (abs_dx > 5.0f) {
      track_mode = "ALIGNING";
      adaptive_jump = std::max(max_jump_px_, 100.0);
      used_alpha_x = 0.55;   // 明显减速，准备贴线
    } else {
      track_mode = "LOCKED";
      adaptive_jump = std::max(10.0, max_jump_px_ * 0.25);
      used_alpha_x = 0.12;   // 慢慢贴中线，不做硬吸附
    }

    if (has_last_) {
      // 只对 x 跳变做约束
      const float jump_x = new_x - last_x_;
      const float abs_jump_x = std::abs(jump_x);

      // LOCKED 下，如果 x 跳变异常大，进行更强的软融合
      if (abs_jump_x > adaptive_jump) {
        if (enable_debug_) {
          RCLCPP_WARN(
            get_logger(),
            "x方向跳变较大(%.1fpx > %.1fpx)，采用软融合抑制",
            abs_jump_x,
            adaptive_jump);
        }

        // TRACKING 阶段允许更多响应，LOCKED 阶段更保守
        if (track_mode == "TRACKING") {
          new_x = static_cast<float>(0.35 * new_x + 0.65 * last_x_);
        } else if (track_mode == "ALIGNING") {
          new_x = static_cast<float>(0.25 * new_x + 0.75 * last_x_);
        } else {
          new_x = static_cast<float>(0.15 * new_x + 0.85 * last_x_);
        }
      }
    }

    // x方向：按状态机 alpha 平滑，LOCK 后慢慢贴中线
    if (has_last_) {
      center_x = static_cast<float>(used_alpha_x * new_x + (1.0 - used_alpha_x) * last_x_);
    } else {
      center_x = new_x;
    }

    // y方向：只做普通平滑显示，不参与状态判定
    if (has_last_) {
      center_y = static_cast<float>(smooth_alpha_ * new_y + (1.0 - smooth_alpha_) * last_y_);
    } else {
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

  try {
    light_pub_->publish(out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "发布 light_position 失败: %s", e.what());
    return;
  }

  if (image_transport_initialized_) {
    try {
      cv::Mat vis = image.clone();

      const cv::Point img_center(img_cx, img_cy);
      const cv::Point raw_det_center(
        static_cast<int>(raw_center_x),
        static_cast<int>(raw_center_y));

      // FPS
      static auto last_time = std::chrono::steady_clock::now();
      static double fps = 0.0;
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last_time).count();
      if (dt > 1e-6) {
        fps = 1.0 / dt;
      }
      last_time = now;

      // 中心十字线
      cv::line(
        vis,
        cv::Point(img_cx, 0),
        cv::Point(img_cx, vis.rows - 1),
        cv::Scalar(0, 0, 255),
        2);

      cv::line(
        vis,
        cv::Point(0, img_cy),
        cv::Point(vis.cols - 1, img_cy),
        cv::Scalar(0, 0, 255),
        2);

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

      if (detected) {
        const auto & d = detections.front();

        // 检测框
        cv::rectangle(vis, d.box, cv::Scalar(0, 0, 255), 2);

        // 标签
        std::string label = d.class_name + " " + cv::format("%.2f", d.score);
        cv::putText(
          vis,
          label,
          cv::Point(d.box.x, std::max(0, d.box.y - 5)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(0, 0, 255),
          2);

        // 原始检测中心（蓝点）
        cv::circle(
          vis,
          raw_det_center,
          4,
          cv::Scalar(255, 0, 0),
          -1);

        // 连线：图像中心直接连到原始检测中心（蓝点）
        cv::line(
          vis,
          img_center,
          raw_det_center,
          cv::Scalar(0, 255, 0),
          2);

        const int dx = raw_det_center.x - img_cx;
        const int dy = raw_det_center.y - img_cy;

        cv::putText(
          vis,
          "x: " + std::to_string(raw_det_center.x),
          cv::Point(10, 30),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(255, 255, 255),
          2);

        cv::putText(
          vis,
          "y: " + std::to_string(raw_det_center.y),
          cv::Point(10, 60),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(255, 255, 255),
          2);

        cv::putText(
          vis,
          "dx: " + std::to_string(dx),
          cv::Point(10, 90),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 255),
          2);

        cv::putText(
          vis,
          "dy: " + std::to_string(dy),
          cv::Point(10, 120),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 255),
          2);

        cv::putText(
          vis,
          "mode: " + track_mode,
          cv::Point(10, 150),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 0),
          2);

        cv::putText(
          vis,
          "jump_th_x: " + cv::format("%.0f", adaptive_jump),
          cv::Point(10, 180),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 0),
          2);

        cv::putText(
          vis,
          "alpha_x: " + cv::format("%.2f", used_alpha_x),
          cv::Point(10, 210),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 0),
          2);
      } else {
        cv::putText(
          vis,
          "Not Detected",
          cv::Point(10, 30),
          cv::FONT_HERSHEY_SIMPLEX,
          0.9,
          cv::Scalar(0, 0, 255),
          2);
      }

      publishImage(result_image_pub_, vis, msg->header.stamp, "bgr8");
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "可视化绘制失败: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "结果图发布失败: %s", e.what());
    }
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
