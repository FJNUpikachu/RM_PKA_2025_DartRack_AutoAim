#include "dart_detector/detect_method.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace pka {

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

bool DartDetector::detect(const cv::Mat& image, cv::Mat& binary_image, 
                         std::vector<std::vector<cv::Point>>& all_contours,
                         std::vector<cv::Point>& best_contour,
                         cv::Point2f& center, double& best_area) {
    all_contours.clear();
    best_contour.clear();
    center = cv::Point2f(0, 0);
    best_area = 0.0;
    binary_image = cv::Mat();
    
    if (image.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "输入图像为空，无法检测");
        return false;
    }

    // 检查图像通道数
    if (image.channels() != 3) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), 
                   "输入图像必须是3通道BGR格式，当前通道数: %d, 尺寸: %dx%d", 
                   image.channels(), image.cols, image.rows);
        return false;
    }

    // 截取ROI
    cv::Mat roi;
    bool is_roi_valid = (y_min_ >= 0 && y_max_ > y_min_ && y_max_ <= image.rows);
    
    if (is_roi_valid) {
        // 使用cv::Rect确保范围正确
        cv::Rect roi_rect(0, y_min_, image.cols, y_max_ - y_min_);
        if (roi_rect.x >= 0 && roi_rect.y >= 0 && 
            roi_rect.x + roi_rect.width <= image.cols && 
            roi_rect.y + roi_rect.height <= image.rows) {
            roi = image(roi_rect).clone();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("DartDetector"), 
                       "ROI矩形无效: x=%d, y=%d, width=%d, height=%d, 图像尺寸: %dx%d",
                       roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height,
                       image.cols, image.rows);
            roi = image.clone();
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
    
    // 确保HSV值在有效范围内
    double h_min = std::max(0.0, std::min(180.0, h_min_));
    double h_max = std::max(0.0, std::min(180.0, h_max_));
    double s_min = std::max(0.0, std::min(255.0, s_min_));
    double s_max = std::max(0.0, std::min(255.0, s_max_));
    double v_min = std::max(0.0, std::min(255.0, v_min_));
    double v_max = std::max(0.0, std::min(255.0, v_max_));

    cv::inRange(hsv_image, 
               cv::Scalar(h_min, s_min, v_min), 
               cv::Scalar(h_max, s_max, v_max), 
               color_mask);

    // 检查掩码是否为空
    if (color_mask.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("DartDetector"), "颜色掩码为空，没有检测到目标颜色");
        return false;
    }

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, kernel);

    // 生成二值化图像（全图尺寸）
    binary_image = cv::Mat::zeros(image.size(), CV_8UC1);
    if (is_roi_valid) {
        color_mask.copyTo(binary_image(cv::Rect(0, y_min_, color_mask.cols, color_mask.rows)));
    } else {
        color_mask.copyTo(binary_image);
    }

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 存储所有轮廓（ROI坐标）
    all_contours = contours;

    bool found = false;
    cv::Point2f best_center(0, 0);
    double max_valid_area = 0.0;

    // 筛选最佳轮廓
    for (const auto& contour : contours) {
        if (contour.empty()) continue;
        
        cv::Rect rect = cv::boundingRect(contour);
        if (rect.width == 0 || rect.height == 0) continue;

        // 长宽比筛选
        double aspect_ratio = static_cast<double>(std::max(rect.width, rect.height)) / 
                             std::min(rect.width, rect.height);
        if (aspect_ratio > aspect_ratio_threshold_) continue;

        // 面积与周长计算
        double perimeter = cv::arcLength(contour, true);
        double area = cv::contourArea(contour);
        if (area <= 0) continue;

        // 半径筛选
        double radius = std::sqrt(area / CV_PI);
        if (radius < min_radius_ || radius > max_radius_) continue;

        // 圆形度筛选
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < circularity_threshold_) continue;

        // 计算中心点
        cv::Moments mom = cv::moments(contour, false);
        if (mom.m00 == 0) continue;  // 避免除零

        // 记录最大面积轮廓
        if (area > max_valid_area) {
            max_valid_area = area;
            best_center.x = static_cast<float>(mom.m10 / mom.m00);
            best_center.y = static_cast<float>(mom.m01 / mom.m00 + y_min_);  // 补偿ROI偏移
            
            // 保存最佳轮廓（考虑ROI偏移）
            best_contour = contour;
            for (auto& pt : best_contour) {
                pt.y += y_min_;
            }
            
            found = true;
            
            if (enable_debug_) {
                RCLCPP_INFO(rclcpp::get_logger("DartDetector"), 
                           "找到候选轮廓: 面积=%.1f, 半径=%.1f, 长宽比=%.2f, 圆形度=%.2f, 中心=(%.1f, %.1f)", 
                           area, radius, aspect_ratio, circularity, best_center.x, best_center.y);
            }
        }
    }

    if (found) {
        center = best_center;
        best_area = max_valid_area;
        
        if (enable_debug_) {
            RCLCPP_INFO(rclcpp::get_logger("DartDetector"), 
                       "检测成功: 最佳中心位置=(%.1f, %.1f), 面积=%.1f", 
                       center.x, center.y, max_valid_area);
        }
    } else if (enable_debug_ && !contours.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("DartDetector"), 
                   "找到 %zu 个轮廓，但都不符合筛选条件", contours.size());
    }

    return found;
}

}  // namespace pka