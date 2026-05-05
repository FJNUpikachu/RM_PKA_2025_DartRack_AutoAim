#include "dart_detector/dart_detector_node.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

namespace {
std::string resolveModelPath(const std::string & input) {
  if (input.empty()) {
    return input;
  }
  // Absolute path: /xxx or C:\xxx
  if (input[0] == '/' || (input.size() >= 2 && input[1] == ':')) {
    return input;
  }
  // Treat as relative to dart_bringup share: <share>/model/xxx
  const std::string bringup_share = ament_index_cpp::get_package_share_directory("dart_bringup");
  return bringup_share + "/" + input;
}
}  // namespace

namespace pka
{

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions & options)
: Node("dart_detector", options),
  image_transport_initialized_(false),
  processing_(false),
  has_printed_image_size_(false),
  frame_count_(0),
  detect_count_(0),
  fps_(0.0),
  last_detected_(false)
{
  declareParameters();
  readParameters();

  if (backend_ == "onnx") {
    const std::string resolved_model_path = resolveModelPath(model_path_);
    model_path_ = resolved_model_path;

    try {
      if (!detector_.init(
          model_path_,
          class_names_,
          input_width_,
          input_height_,
          conf_threshold_,
          iou_threshold_))
      {
        RCLCPP_FATAL(get_logger(), "ONNX模型加载失败: %s", model_path_.c_str());
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
  } else if (backend_ == "traditional") {
    if (y_min_ >= y_max_ || y_min_ < 0 || y_max_ <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid ROI (y_min=%d, y_max=%d), use full image", y_min_, y_max_);
      y_min_ = 0;
      y_max_ = 0;
    }
    traditional_detector_.setParameters(
      green_diff_thresh_,
      green_abs_thresh_,
      blur_ksize_,
      min_radius_,
      max_radius_,
      aspect_ratio_threshold_,
      y_min_,
      y_max_,
      circularity_threshold_,
      enable_debug_);
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown detector backend: %s (expect onnx|traditional)", backend_.c_str());
    throw std::runtime_error("Unknown detector backend");
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
  declare_parameter("backend", std::string("onnx"));

  declare_parameter("image_topic", "image_raw");
  declare_parameter("enable_debug", false);

  // ONNX 模式参数
  declare_parameter("model_path", "model/best.onnx");
  declare_parameter("class_names", std::vector<std::string>{"light"});
  declare_parameter("input_width", 192);
  declare_parameter("input_height", 192);
  declare_parameter("conf_threshold", 0.25);
  declare_parameter("iou_threshold", 0.45);
  declare_parameter("use_cuda", false);

  // Traditional 模式参数
  declare_parameter("green_diff_thresh", 30.0);
  declare_parameter("green_abs_thresh", 80.0);
  declare_parameter("blur_ksize", 5);
  declare_parameter("y_min", 0);
  declare_parameter("y_max", 0);
  declare_parameter("aspect_ratio_threshold", 1.5);
  declare_parameter("circularity_threshold", 0.65);
  declare_parameter("min_radius", 5.0);
  declare_parameter("max_radius", 100.0);
}

void DartDetectorNode::readParameters()
{
  backend_ = get_parameter("backend").as_string();
  image_topic_ = get_parameter("image_topic").as_string();
  enable_debug_ = get_parameter("enable_debug").as_bool();

  // ONNX 模式参数（即便是 traditional 模式也会声明并读取一遍，不影响）
  model_path_ = get_parameter("model_path").as_string();
  class_names_ = get_parameter("class_names").as_string_array();
  input_width_ = get_parameter("input_width").as_int();
  input_height_ = get_parameter("input_height").as_int();
  conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
  iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
  use_cuda_ = get_parameter("use_cuda").as_bool();

  // Traditional 模式参数
  green_diff_thresh_ = get_parameter("green_diff_thresh").as_double();
  green_abs_thresh_ = get_parameter("green_abs_thresh").as_double();
  blur_ksize_ = get_parameter("blur_ksize").as_int();
  y_min_ = get_parameter("y_min").as_int();
  y_max_ = get_parameter("y_max").as_int();
  aspect_ratio_threshold_ = get_parameter("aspect_ratio_threshold").as_double();
  circularity_threshold_ = get_parameter("circularity_threshold").as_double();
  min_radius_ = get_parameter("min_radius").as_double();
  max_radius_ = get_parameter("max_radius").as_double();
}

void DartDetectorNode::printParameters()
{
  RCLCPP_INFO(get_logger(), "===== detector 参数 =====");
  RCLCPP_INFO(get_logger(), "backend: %s", backend_.c_str());
  RCLCPP_INFO(get_logger(), "image_topic: %s", image_topic_.c_str());
  RCLCPP_INFO(get_logger(), "enable_debug: %s", enable_debug_ ? "true" : "false");

  if (backend_ == "onnx") {
    RCLCPP_INFO(get_logger(), "model_path: %s", model_path_.c_str());
    RCLCPP_INFO(get_logger(), "input: %d x %d", input_width_, input_height_);
    RCLCPP_INFO(get_logger(), "conf_threshold: %.2f", conf_threshold_);
    RCLCPP_INFO(get_logger(), "iou_threshold: %.2f", iou_threshold_);
    RCLCPP_INFO(get_logger(), "use_cuda: %s (当前版本忽略，固定 CPU)", use_cuda_ ? "true" : "false");
  } else {
    RCLCPP_INFO(get_logger(), "Traditional params: green_diff_thresh=%.1f, green_abs_thresh=%.1f, blur_ksize=%d",
                 green_diff_thresh_, green_abs_thresh_, blur_ksize_);
    RCLCPP_INFO(get_logger(), "ROI params: y_min=%d, y_max=%d", y_min_, y_max_);
    RCLCPP_INFO(get_logger(), "Shape params: aspect_ratio_threshold=%.2f, circularity_threshold=%.2f",
                 aspect_ratio_threshold_, circularity_threshold_);
    RCLCPP_INFO(get_logger(), "Radius: min=%.1f, max=%.1f", min_radius_, max_radius_);
  }
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

  // FPS 统计
  const rclcpp::Time now_t = this->now();
  if (frame_count_ == 0) {
    last_frame_time_ = now_t;
  } else {
    const double dt = (now_t - last_frame_time_).seconds();
    if (dt > 1e-6) {
      fps_ = 0.9 * fps_ + 0.1 / dt;
    }
  }
  last_frame_time_ = now_t;
  frame_count_++;

  if (!has_printed_image_size_) {
    RCLCPP_INFO(
      get_logger(),
      "[detector] 图像尺寸: %d x %d, channels=%d, backend=%s",
      image.cols, image.rows, image.channels(), backend_.c_str());
    has_printed_image_size_ = true;
  }

  if (image_transport_initialized_) {
    publishImage(raw_image_pub_, image, msg->header.stamp, "bgr8");
  }

  const int img_cx = image.cols / 2;
  const int img_cy = image.rows / 2;

  // Publish light_position
  dart_interfaces::msg::Light out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;

  if (backend_ == "onnx") {
    std::vector<Detection> detections;
    try {
      detections = detector_.infer(image);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "detector 推理失败: %s", e.what());
      std::lock_guard<std::mutex> lock(image_mutex_);
      processing_ = false;
      return;
    }

    bool detected = false;
    int best_idx = -1;
    double center_x = -1.0;
    double center_y = -1.0;

    // 分数优先，分数接近时再选更靠近图像中心的
    float best_score = -1.0f;
    double best_center_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto & d = detections[i];
      const double cx = d.box.x + d.box.width * 0.5;
      const double cy = d.box.y + d.box.height * 0.5;
      const double dist = std::hypot(cx - img_cx, cy - img_cy);

      if (d.score > best_score + 1e-6f) {
        best_score = d.score;
        best_center_dist = dist;
        best_idx = static_cast<int>(i);
      } else if (std::fabs(d.score - best_score) < 0.02f && dist < best_center_dist) {
        best_center_dist = dist;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx >= 0) {
      const auto & d = detections[best_idx];
      center_x = d.box.x + d.box.width * 0.5;
      center_y = d.box.y + d.box.height * 0.5;
      detected = true;
    }

    out.x = detected ? center_x : -1.0;
    out.y = detected ? center_y : -1.0;

    if (detected) { detect_count_++; }

    // 状态变化：found / lost
    if (detected != last_detected_) {
      if (detected) {
        RCLCPP_INFO(get_logger(),
          "[detector/onnx] 目标找到: x=%.2f y=%.2f score=%.2f  (frame #%zu)",
          center_x, center_y, best_idx >= 0 ? detections[best_idx].score : 0.f,
          frame_count_);
      } else {
        RCLCPP_INFO(get_logger(),
          "[detector/onnx] 目标丢失 (frame #%zu, 丢失前共检测 %zu 帧)",
          frame_count_, detect_count_);
      }
      last_detected_ = detected;
    }

    // 每帧 DEBUG：候选框数量 + 最终选取结果
    RCLCPP_DEBUG(get_logger(),
      "[detector/onnx] frame=%zu raw_detections=%zu detected=%s "
      "x=%.3f y=%.3f score=%.3f fps=%.1f",
      frame_count_, detections.size(),
      detected ? "Y" : "N",
      out.x, out.y,
      best_idx >= 0 ? detections[best_idx].score : 0.f,
      fps_);

    // 每 5 秒打一条 INFO 摘要（enable_debug 时开启）
    if (enable_debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[detector/onnx] FPS=%.1f  帧计数=%zu  检测帧=%zu  检测率=%.1f%%",
        fps_, frame_count_, detect_count_,
        frame_count_ > 0 ? 100.0 * detect_count_ / frame_count_ : 0.0);
    }

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
            cv::Point(draw_box.x, std::max(0, draw_box.y - 5)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 0, 255),
            2);

          cv::circle(vis, det_center, 4, cv::Scalar(255, 0, 0), -1);
          cv::line(vis, cv::Point(img_cx, img_cy), det_center, cv::Scalar(0, 255, 0), 2);

          const int dx = det_center.x - img_cx;
          const int dy = det_center.y - img_cy;

          cv::putText(vis, "x: " + cv::format("%.4f", center_x), cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
          cv::putText(vis, "y: " + cv::format("%.4f", center_y), cv::Point(10, 60),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
          cv::putText(vis, "dx: " + std::to_string(dx), cv::Point(10, 90),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
          cv::putText(vis, "dy: " + std::to_string(dy), cv::Point(10, 120),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        } else {
          cv::putText(vis, "Not Detected", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
        }

        publishImage(result_image_pub_, vis, msg->header.stamp, "bgr8");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "结果图发布失败: %s", e.what());
      }
    }
  } else {
    cv::Mat binary_image;
    std::vector<std::vector<cv::Point>> all_contours;
    std::vector<cv::Point> best_contour;
    cv::Point2f light_center(0, 0);
    double best_area = 0.0;

    bool detected = false;
    try {
      detected = traditional_detector_.detect(
        image,
        binary_image,
        all_contours,
        best_contour,
        light_center,
        best_area);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "traditional detect failed: %s", e.what());
    }

    if (detected) {
      out.x = light_center.x;
      out.y = light_center.y;
      detect_count_++;
    } else {
      out.x = 0.0;
      out.y = 0.0;
    }

    // 状态变化：found / lost
    if (detected != last_detected_) {
      if (detected) {
        RCLCPP_INFO(get_logger(),
          "[detector/traditional] 目标找到: x=%.2f y=%.2f area=%.1f  (frame #%zu)",
          light_center.x, light_center.y, best_area, frame_count_);
      } else {
        RCLCPP_INFO(get_logger(),
          "[detector/traditional] 目标丢失 (frame #%zu, 丢失前共检测 %zu 帧)",
          frame_count_, detect_count_);
      }
      last_detected_ = detected;
    }

    // 每帧 DEBUG：检测坐标 + 面积 + FPS
    RCLCPP_DEBUG(get_logger(),
      "[detector/traditional] frame=%zu detected=%s "
      "x=%.3f y=%.3f area=%.1f fps=%.1f",
      frame_count_,
      detected ? "Y" : "N",
      out.x, out.y, best_area, fps_);

    // 每 5 秒打一条 INFO 摘要（enable_debug 时开启）
    if (enable_debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[detector/traditional] FPS=%.1f  帧计数=%zu  检测帧=%zu  检测率=%.1f%%",
        fps_, frame_count_, detect_count_,
        frame_count_ > 0 ? 100.0 * detect_count_ / frame_count_ : 0.0);
    }

    light_pub_->publish(out);

    if (image_transport_initialized_) {
      try {
        cv::Mat vis = image.clone();

        // 图像中轴线
        cv::line(vis, cv::Point(img_cx, 0), cv::Point(img_cx, vis.rows - 1), cv::Scalar(0, 0, 255), 2);
        cv::line(vis, cv::Point(0, img_cy), cv::Point(vis.cols - 1, img_cy), cv::Scalar(0, 0, 255), 2);
        cv::circle(vis, cv::Point(img_cx, img_cy), 4, cv::Scalar(0, 255, 255), -1);

        if (detected) {
          // 等效半径可让可视化更贴近实际灯大小
          const int disp_radius = std::max(
            8,
            static_cast<int>(std::sqrt(best_area / CV_PI)));
          cv::circle(vis, light_center, disp_radius, cv::Scalar(255, 0, 0), 2);
          cv::line(vis, cv::Point(img_cx, img_cy), light_center, cv::Scalar(0, 255, 0), 2);
          if (!best_contour.empty()) {
            std::vector<std::vector<cv::Point>> bv = {best_contour};
            cv::drawContours(vis, bv, 0, cv::Scalar(0, 0, 255), 2);
          }
          cv::putText(vis, "Detected", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
        } else {
          cv::putText(vis, "Not Detected", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
        }

        publishImage(result_image_pub_, vis, msg->header.stamp, "bgr8");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "结果图发布失败: %s", e.what());
      }
    }
  }

  // ONNX 分支在绘制时没有发布 light_pub_，这里统一发布（传统分支已发布也没关系）
  if (backend_ == "onnx") {
    light_pub_->publish(out);
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