#include "dart_detector/onnx_detector.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace pka
{

OnnxDetector::OnnxDetector() = default;

bool OnnxDetector::init(
  const std::string & model_path,
  const std::vector<std::string> & class_names,
  int input_width,
  int input_height,
  float conf_threshold,
  float iou_threshold)
{
  class_names_ = class_names;
  input_width_ = input_width;
  input_height_ = input_height;
  conf_threshold_ = conf_threshold;
  iou_threshold_ = iou_threshold;

  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "dart_detector");
    session_options_ = std::make_unique<Ort::SessionOptions>();

    // 使用 4 线程 CPU 推理
    session_options_->SetIntraOpNumThreads(4);
    session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    session_ = std::make_unique<Ort::Session>(
      *env_, model_path.c_str(), *session_options_);

    Ort::AllocatorWithDefaultOptions allocator;

    {
      auto input_name_alloc = session_->GetInputNameAllocated(0, allocator);
      input_name_ = input_name_alloc.get();
    }

    {
      auto output_name_alloc = session_->GetOutputNameAllocated(0, allocator);
      output_name_ = output_name_alloc.get();
    }

    initialized_ = true;
    return true;
  } catch (const Ort::Exception & e) {
    std::cerr << "Failed to initialize ONNX Runtime session: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  } catch (const std::exception & e) {
    std::cerr << "Failed to initialize detector: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  }
}

cv::Mat OnnxDetector::preprocess(
  const cv::Mat & image,
  std::vector<float> & input_tensor_values,
  float & scale_x,
  float & scale_y) const
{
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(input_width_, input_height_));

  // 记录从网络输入坐标映射回原图坐标所需的比例
  scale_x = static_cast<float>(image.cols) / static_cast<float>(input_width_);
  scale_y = static_cast<float>(image.rows) / static_cast<float>(input_height_);

  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

  input_tensor_values.resize(1 * 3 * input_height_ * input_width_);

  // HWC -> CHW
  std::vector<cv::Mat> channels(3);
  cv::split(rgb, channels);

  const int channel_size = input_height_ * input_width_;
  for (int c = 0; c < 3; ++c) {
    std::memcpy(
      input_tensor_values.data() + c * channel_size,
      channels[c].data,
      channel_size * sizeof(float));
  }

  return resized;
}

std::vector<Detection> OnnxDetector::infer(const cv::Mat & image)
{
  std::vector<Detection> detections;
  if (!initialized_ || image.empty()) {
    return detections;
  }

  std::vector<float> input_tensor_values;
  float scale_x = 1.0f;
  float scale_y = 1.0f;
  preprocess(image, input_tensor_values, scale_x, scale_y);

  std::vector<int64_t> input_shape = {1, 3, input_height_, input_width_};

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
    OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info,
    input_tensor_values.data(),
    input_tensor_values.size(),
    input_shape.data(),
    input_shape.size());

  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};

  auto output_tensors = session_->Run(
    Ort::RunOptions{nullptr},
    input_names,
    &input_tensor,
    1,
    output_names,
    1);

  if (output_tensors.empty()) {
    return detections;
  }

  auto & output_tensor = output_tensors[0];
  auto type_info = output_tensor.GetTensorTypeAndShapeInfo();
  std::vector<int64_t> output_shape = type_info.GetShape();
  const float * output_data = output_tensor.GetTensorData<float>();

  return postprocess(image, output_data, output_shape, scale_x, scale_y);
}

std::vector<Detection> OnnxDetector::postprocess(
  const cv::Mat & image,
  const float * output_data,
  const std::vector<int64_t> & output_shape,
  float scale_x,
  float scale_y) const
{
  std::vector<Detection> detections;

  if (output_data == nullptr || output_shape.empty()) {
    return detections;
  }

  if (output_shape.size() != 3) {
    return detections;
  }

  const int64_t batch = output_shape[0];
  const int64_t dim1 = output_shape[1];
  const int64_t dim2 = output_shape[2];

  if (batch != 1) {
    return detections;
  }

  const int rows = static_cast<int>(dim2);
  const int dims = static_cast<int>(dim1);

  if (dims < 5) {
    return detections;
  }

  const int num_classes = dims - 4;

  std::vector<int> class_ids;
  std::vector<float> scores;

  // 浮点框：用于保留更高精度
  std::vector<cv::Rect2d> boxes_float;

  // 整数框：只用于 OpenCV NMS
  std::vector<cv::Rect> boxes_nms;

  for (int i = 0; i < rows; ++i) {
    auto get_val = [&](int d) -> float {
      return output_data[d * rows + i];
    };

    const float cx = get_val(0);
    const float cy = get_val(1);
    const float w  = get_val(2);
    const float h  = get_val(3);

    int class_id = -1;
    float conf = 0.0f;

    if (num_classes == 1) {
      conf = get_val(4);
      class_id = 0;
    } else {
      for (int c = 0; c < num_classes; ++c) {
        const float score = get_val(4 + c);
        if (score > conf) {
          conf = score;
          class_id = c;
        }
      }
    }

    if (conf < conf_threshold_) {
      continue;
    }

    // 将网络输出框映射回原图坐标
    double left   = (cx - 0.5f * w) * scale_x;
    double top    = (cy - 0.5f * h) * scale_y;
    double width  = w * scale_x;
    double height = h * scale_y;

    left = std::max(0.0, std::min(left, static_cast<double>(image.cols - 1)));
    top = std::max(0.0, std::min(top, static_cast<double>(image.rows - 1)));
    width = std::max(1.0, std::min(width, static_cast<double>(image.cols) - left));
    height = std::max(1.0, std::min(height, static_cast<double>(image.rows) - top));

    boxes_float.emplace_back(left, top, width, height);

    boxes_nms.emplace_back(
      static_cast<int>(left),
      static_cast<int>(top),
      static_cast<int>(width),
      static_cast<int>(height));

    scores.emplace_back(conf);
    class_ids.emplace_back(class_id);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes_nms, scores, conf_threshold_, iou_threshold_, indices);

  for (int idx : indices) {
    Detection det;
    det.class_id = class_ids[idx];
    det.score = scores[idx];
    det.box = boxes_float[idx];

    if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size())) {
      det.class_name = class_names_[det.class_id];
    } else {
      det.class_name = "unknown";
    }

    detections.push_back(det);
  }

  // 按分数从高到低排序
  std::sort(
    detections.begin(), detections.end(),
    [](const Detection & a, const Detection & b) {
      return a.score > b.score;
    });

  return detections;
}

}  // namespace pka