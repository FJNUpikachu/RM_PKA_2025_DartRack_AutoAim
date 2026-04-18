#ifndef DART_DETECTOR_TYPES_HPP_
#define DART_DETECTOR_TYPES_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace pka
{

// 单个检测结果的数据结构
struct Detection
{
  int class_id = -1;
  float score = 0.0f;

  // 使用浮点框，保留更高精度
  cv::Rect2d box;

  std::string class_name = "unknown";
};

}  // namespace pka

#endif  // DART_DETECTOR_TYPES_HPP_