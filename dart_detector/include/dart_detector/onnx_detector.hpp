#ifndef DART_DETECTOR_ONNX_DETECTOR_HPP_
#define DART_DETECTOR_ONNX_DETECTOR_HPP_

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <memory>

#include <onnxruntime_cxx_api.h>

#include "dart_detector/types.hpp"

namespace pka
{

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

  std::vector<Detection> infer(const cv::Mat & image);

private:
  cv::Mat preprocess(
    const cv::Mat & image,
    std::vector<float> & input_tensor_values,
    float & scale_x,
    float & scale_y) const;

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

  // ONNX Runtime
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::SessionOptions> session_options_;

  std::string input_name_;
  std::string output_name_;
};

}  // namespace pka

#endif  // DART_DETECTOR_ONNX_DETECTOR_HPP_