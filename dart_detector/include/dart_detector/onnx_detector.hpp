#ifndef DART_DETECTOR_ONNX_DETECTOR_HPP_
#define DART_DETECTOR_ONNX_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

#include "dart_detector/types.hpp"

namespace pka
{

class OnnxDetector
{
public:
  OnnxDetector() = default;
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
    cv::Mat & blob,
    float & scale_x,
    float & scale_y) const;

  std::vector<Detection> postprocess(
    const cv::Mat & image,
    const std::vector<cv::Mat> & outputs,
    float scale_x,
    float scale_y) const;

private:
  cv::dnn::Net net_;
  bool initialized_ = false;

  std::vector<std::string> class_names_;
  int input_width_ = 640;
  int input_height_ = 640;
  float conf_threshold_ = 0.25f;
  float iou_threshold_ = 0.45f;
};

}  // namespace pka

#endif  // DART_DETECTOR_ONNX_DETECTOR_HPP_