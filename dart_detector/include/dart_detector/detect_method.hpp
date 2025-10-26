#ifndef DART_DETECTOR_DETECT_METHOD_HPP_
#define DART_DETECTOR_DETECT_METHOD_HPP_

#include <opencv2/opencv.hpp>
#include <memory>

namespace dart_detector {

class DartDetector {
public:
    DartDetector();
    ~DartDetector() = default;

    void setParameters(double h_min, double h_max, double s_min, double s_max, double v_min, double v_max,
                      double min_radius, double max_radius, double aspect_ratio_threshold,
                      int y_min, int y_max, double circularity_threshold, bool enable_debug);

    bool detect(const cv::Mat& image, cv::Mat& processed_image, cv::Point2f& center);

private:
    double h_min_, h_max_;
    double s_min_, s_max_;
    double v_min_, v_max_;
    double min_radius_, max_radius_;
    double aspect_ratio_threshold_;
    int y_min_, y_max_;
    double circularity_threshold_;
    bool enable_debug_;
};

}  // namespace dart_detector

#endif  // DART_DETECTOR_DETECT_METHOD_HPP_