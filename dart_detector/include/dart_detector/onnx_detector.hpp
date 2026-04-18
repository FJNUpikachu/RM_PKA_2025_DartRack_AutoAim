#ifndef DART_DETECTOR_ONNX_DETECTOR_HPP_
#define DART_DETECTOR_ONNX_DETECTOR_HPP_

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "dart_detector/types.hpp"

namespace pka
{

// 这个类只负责 ONNX 模型的推理与后处理
class OnnxDetector
{
public:
  OnnxDetector();
  ~OnnxDetector() = default;

  bool init(
    const std::string & model_path,
    const std::vector<std::string> & class_names,
    int input_width,
    int input_height,
    float conf_threshold,
    float iou_threshold);

  // 输入原图，输出检测结果
  std::vector<Detection> infer(const cv::Mat & image);

private:
  // 前处理：resize + BGR->RGB + 归一化 + HWC->CHW
  cv::Mat preprocess(
    const cv::Mat & image,
    std::vector<float> & input_tensor_values,
    float & scale_x,
    float & scale_y) const;

  // 后处理：解析输出、筛框、NMS
  std::vector<Detection> postprocess(
    const cv::Mat & image,
    const float * output_data,
    const std::vector<int64_t> & output_shape,
    float scale_x,
    float scale_y) const;

private:
  bool initialized_ = false;

  std::vector<std::string> class_names_;
  int input_width_ = 640;
  int input_height_ = 640;
  float conf_threshold_ = 0.25f;
  float iou_threshold_ = 0.45f;

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::SessionOptions> session_options_;

  std::string input_name_;
  std::string output_name_;
};

}  // namespace pka

#endif  // DART_DETECTOR_ONNX_DETECTOR_HPP_