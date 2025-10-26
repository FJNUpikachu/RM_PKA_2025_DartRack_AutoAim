#include "dart_detector/detect_method.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace dart_detector {

DartDetector::DartDetector() 
    : h_min_(40), h_max_(80),    
      s_min_(100), s_max_(255),
      v_min_(100), v_max_(255),
      min_radius_(5), max_radius_(50),  
      aspect_ratio_threshold_(1.2),
      y_min_(0), y_max_(0),
      circularity_threshold_(0.75),
      enable_debug_(false) {}

void DartDetector::setParameters(double h_min, double h_max, double s_min, double s_max, double v_min, double v_max,
                                double min_radius, double max_radius, double aspect_ratio_threshold,
                                int y_min, int y_max, double circularity_threshold, bool enable_debug) {
    h_min_ = h_min;
    h_max_ = h_max;
    s_min_ = s_min;
    s_max_ = s_max;
    v_min_ = v_min;
    v_max_ = v_max;
    min_radius_ = min_radius;
    max_radius_ = max_radius;
    aspect_ratio_threshold_ = aspect_ratio_threshold;
    y_min_ = y_min;
    y_max_ = y_max;
    circularity_threshold_ = circularity_threshold;
    enable_debug_ = enable_debug;
}

// 新增：通过center参数传出目标坐标
bool DartDetector::detect(const cv::Mat& image, cv::Mat& processed_image, cv::Point2f& center) {
    // 初始化中心坐标为无效值
    center = cv::Point2f(-1.0, -1.0);
    
    if (image.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "输入图像为空，无法检测");
        return false;
    }

    if (image.channels() != 3) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "输入图像必须是3通道BGR格式，当前通道数: %d", image.channels());
        processed_image = image.clone();
        return false;
    }

    processed_image = image.clone();

    // 截取ROI
    cv::Mat roi;
    bool is_roi_valid = (y_min_ >= 0 && y_max_ > y_min_ && y_max_ <= image.rows);
    if (is_roi_valid) {
        roi = image(cv::Range(y_min_, y_max_), cv::Range::all());
        if (enable_debug_) {
            cv::rectangle(processed_image, 
                          cv::Point(0, y_min_), 
                          cv::Point(image.cols, y_max_), 
                          cv::Scalar(0, 255, 0), 2);
        }
    } else {
        roi = image.clone();
    }

    if (roi.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "ROI处理后为空图像");
        return false;
    }

    // HSV转换与颜色过滤
    cv::Mat hsv_image, color_mask;
    cv::cvtColor(roi, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, 
               cv::Scalar(h_min_, s_min_, v_min_), 
               cv::Scalar(h_max_, s_max_, v_max_), 
               color_mask);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_valid_area = 0.0;
    cv::Point2f best_center;

    // 调试模式：绘制所有轮廓
    if (enable_debug_) {
        for (const auto& contour : contours) {
            std::vector<cv::Point> contour_transformed = contour;
            for (auto& pt : contour_transformed) {
                pt.y += y_min_;
            }
            cv::drawContours(processed_image, {contour_transformed}, -1, cv::Scalar(255, 0, 0), 2);
        }
    }

    // 筛选最佳轮廓
    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        if (rect.width == 0 || rect.height == 0) continue;

        // 长宽比筛选
        double aspect_ratio = static_cast<double>(std::max(rect.width, rect.height)) / 
                             std::min(rect.width, rect.height);
        if (aspect_ratio > aspect_ratio_threshold_) continue;

        // 面积与周长计算
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        if (area <= 0) continue;  // 避免除零错误

        // 半径筛选
        double radius = std::sqrt(area / CV_PI);
        if (radius < min_radius_ || radius > max_radius_) continue;

        // 圆形度筛选
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < circularity_threshold_) continue;

        // 记录最大面积轮廓
        if (area > max_valid_area) {
            max_valid_area = area;
            cv::Moments mom = cv::moments(contour, false);
            best_center.x = mom.m10 / mom.m00;
            best_center.y = mom.m01 / mom.m00 + y_min_;  // 补偿ROI偏移
        }
    }

    // 标记检测结果并赋值中心坐标
    if (max_valid_area > 0) {
        center = best_center;  // 将有效坐标传出
        cv::circle(processed_image, best_center, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(processed_image, "Detected", 
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        return true;
    }

    if (enable_debug_) {
        cv::putText(processed_image, "Not detected", 
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    return false;
}

}  // namespace dart_detector