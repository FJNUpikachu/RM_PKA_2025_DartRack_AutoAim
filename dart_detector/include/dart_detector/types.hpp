#ifndef DART_DETECTOR_TYPES_HPP_
#define DART_DETECTOR_TYPES_HPP_

#include <opencv2/opencv.hpp>
#include <string>

namespace pka
{

struct Detection
{
  int class_id = -1;
  float score = 0.0f;
  cv::Rect box;
  std::string class_name = "unknown";
};

}  // namespace pka

#endif  // DART_DETECTOR_TYPES_HPP_