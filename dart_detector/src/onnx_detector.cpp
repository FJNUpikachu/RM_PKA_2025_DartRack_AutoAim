#include "dart_detector/onnx_detector.hpp"

#include <algorithm>
#include <iostream>

namespace pka
{

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
    net_ = cv::dnn::readNet(model_path);
  } catch (const std::exception & e) {
    std::cerr << "Failed to load ONNX model: " << e.what() << std::endl;
    return false;
  }

  // 固定 CPU 推理
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  initialized_ = true;
  return true;
}

cv::Mat OnnxDetector::preprocess(
  const cv::Mat & image,
  cv::Mat & blob,
  float & scale_x,
  float & scale_y) const
{
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(input_width_, input_height_));

  scale_x = static_cast<float>(image.cols) / static_cast<float>(input_width_);
  scale_y = static_cast<float>(image.rows) / static_cast<float>(input_height_);

  blob = cv::dnn::blobFromImage(
    resized,
    1.0 / 255.0,
    cv::Size(input_width_, input_height_),
    cv::Scalar(),
    true,
    false);

  return resized;
}

std::vector<Detection> OnnxDetector::infer(const cv::Mat & image)
{
  std::vector<Detection> detections;
  if (!initialized_ || image.empty()) {
    return detections;
  }

  cv::Mat blob;
  float scale_x = 1.0f;
  float scale_y = 1.0f;
  preprocess(image, blob, scale_x, scale_y);

  net_.setInput(blob);

  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  return postprocess(image, outputs, scale_x, scale_y);
}

std::vector<Detection> OnnxDetector::postprocess(
  const cv::Mat & image,
  const std::vector<cv::Mat> & outputs,
  float scale_x,
  float scale_y) const
{
  std::vector<Detection> detections;

  if (outputs.empty()) {
    std::cerr << "[OnnxDetector] outputs is empty." << std::endl;
    return detections;
  }

  cv::Mat output = outputs[0];

  // 兼容 Ultralytics 常见 ONNX 输出:
  // 例如:
  // [1, 5, 8400]   单类别
  // [1, 84, 8400]  多类别
  // 转成 [8400, 5] 或 [8400, 84]
  if (output.dims == 3) {
    int dim0 = output.size[0];
    int dim1 = output.size[1];
    int dim2 = output.size[2];

    if (dim0 != 1) {
      std::cerr << "[OnnxDetector] Unexpected batch size: " << dim0 << std::endl;
      return detections;
    }

    cv::Mat output_2d = output.reshape(1, dim1);  // [dim1, dim2]
    cv::transpose(output_2d, output);             // [dim2, dim1]

    (void)dim2;
  } else if (output.dims != 2) {
    std::cerr << "[OnnxDetector] Unsupported output dims: " << output.dims << std::endl;
    return detections;
  }

  if (output.empty()) {
    std::cerr << "[OnnxDetector] output is empty after reshape." << std::endl;
    return detections;
  }

  std::vector<int> class_ids;
  std::vector<float> scores;
  std::vector<cv::Rect> boxes;

  const int rows = output.rows;
  const int dims = output.cols;

  if (dims < 5) {
    std::cerr << "[OnnxDetector] Invalid output dims: " << dims << std::endl;
    return detections;
  }

  const int num_classes = dims - 4;

  for (int i = 0; i < rows; ++i) {
    const float * data = output.ptr<float>(i);

    float cx = data[0];
    float cy = data[1];
    float w  = data[2];
    float h  = data[3];

    int class_id = -1;
    float conf = 0.0f;

    if (num_classes == 1) {
      // 单类别，data[4] 直接当置信度
      conf = data[4];
      class_id = 0;
    } else {
      // 多类别，从 data[4] 开始找最大类别分数
      cv::Mat scores_mat(1, num_classes, CV_32FC1, (void *)(data + 4));
      cv::Point class_id_point;
      double max_class_score = 0.0;
      cv::minMaxLoc(scores_mat, nullptr, &max_class_score, nullptr, &class_id_point);

      conf = static_cast<float>(max_class_score);
      class_id = class_id_point.x;
    }

    if (conf < conf_threshold_) {
      continue;
    }

    int left   = static_cast<int>((cx - 0.5f * w) * scale_x);
    int top    = static_cast<int>((cy - 0.5f * h) * scale_y);
    int width  = static_cast<int>(w * scale_x);
    int height = static_cast<int>(h * scale_y);

    left = std::max(0, std::min(left, image.cols - 1));
    top = std::max(0, std::min(top, image.rows - 1));
    width = std::max(1, std::min(width, image.cols - left));
    height = std::max(1, std::min(height, image.rows - top));

    boxes.emplace_back(left, top, width, height);
    scores.emplace_back(conf);
    class_ids.emplace_back(class_id);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, iou_threshold_, indices);

  for (int idx : indices) {
    Detection det;
    det.class_id = class_ids[idx];
    det.score = scores[idx];
    det.box = boxes[idx];

    if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names_.size())) {
      det.class_name = class_names_[det.class_id];
    } else {
      det.class_name = "unknown";
    }

    detections.push_back(det);
  }

  std::sort(
    detections.begin(), detections.end(),
    [](const Detection & a, const Detection & b) {
      return a.score > b.score;
    });

  return detections;
}

}  // namespace pka