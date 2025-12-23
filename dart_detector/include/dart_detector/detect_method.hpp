#ifndef DART_DETECTOR_DETECT_METHOD_HPP
#define DART_DETECTOR_DETECT_METHOD_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace pka {

class DartDetector {
public:
    DartDetector();
    
    void setParameters(double h_min, double h_max, double s_min, double s_max, double v_min, double v_max,
                      double min_radius, double max_radius, double aspect_ratio_threshold,
                      int y_min, int y_max, double circularity_threshold, bool enable_debug);
    
    // 修改：简化detect函数接口，移除绘制功能
    bool detect(const cv::Mat& image, cv::Mat& binary_image, 
                std::vector<std::vector<cv::Point>>& all_contours,
                std::vector<cv::Point>& best_contour,
                cv::Point2f& center, double& best_area);
    
private:
    // HSV参数
    double h_min_, h_max_;
    double s_min_, s_max_;
    double v_min_, v_max_;
    
    // 半径范围
    double min_radius_, max_radius_;
    
    // 形状筛选参数
    double aspect_ratio_threshold_;
    double circularity_threshold_;
    
    // ROI参数
    int y_min_, y_max_;
    
    // 调试模式
    bool enable_debug_;
};

}  // namespace pka

#endif  // DART_DETECTOR_DETECT_METHOD_HPP