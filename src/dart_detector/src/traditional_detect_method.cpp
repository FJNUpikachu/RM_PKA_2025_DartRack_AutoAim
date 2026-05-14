#include "dart_detector/traditional_detect_method.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace pka {

TraditionalDartDetector::TraditionalDartDetector() 
    : green_diff_thresh_(30),    // G - max(R,B) 的最小差值，用于捕获纯绿区域
      green_abs_thresh_(80),     // G 通道绝对亮度阈值，用于捕获白色高亮中心
      blur_ksize_(5),            // 高斯模糊核大小（必须为奇数）
      min_radius_(5), max_radius_(50),  
      aspect_ratio_threshold_(1.5),
      y_min_(0), y_max_(0),
      circularity_threshold_(0.65),   // 稍微放宽圆形度，因为灯晕边缘不规则
      enable_debug_(false) {}

void TraditionalDartDetector::setParameters(
    double green_diff_thresh, double green_abs_thresh, int blur_ksize,
    double min_radius, double max_radius, double aspect_ratio_threshold,
    int y_min, int y_max, double circularity_threshold, bool enable_debug)
{
    green_diff_thresh_     = green_diff_thresh;
    green_abs_thresh_      = green_abs_thresh;
    blur_ksize_            = (blur_ksize % 2 == 1) ? blur_ksize : blur_ksize + 1;  // 保证奇数
    min_radius_            = min_radius;
    max_radius_            = max_radius;
    aspect_ratio_threshold_= aspect_ratio_threshold;
    y_min_                 = y_min;
    y_max_                 = y_max;
    circularity_threshold_ = circularity_threshold;
    enable_debug_          = enable_debug;
}

bool TraditionalDartDetector::detect(const cv::Mat& image, cv::Mat& binary_image,
                          std::vector<std::vector<cv::Point>>& all_contours,
                          std::vector<cv::Point>& best_contour,
                          cv::Point2f& center, double& best_area)
{
    all_contours.clear();
    best_contour.clear();
    center    = cv::Point2f(0, 0);
    best_area = 0.0;
    binary_image = cv::Mat();

    if (image.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "输入图像为空，无法检测");
        return false;
    }
    if (image.channels() != 3) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"),
                    "输入图像必须是3通道BGR格式，当前通道数: %d", image.channels());
        return false;
    }

    // ── 1. ROI 截取 ──────────────────────────────────────────────────────────
    cv::Mat roi;
    bool is_roi_valid = (y_min_ >= 0 && y_max_ > y_min_ && y_max_ <= image.rows);
    if (is_roi_valid) {
        cv::Rect roi_rect(0, y_min_, image.cols, y_max_ - y_min_);
        if (roi_rect.x >= 0 && roi_rect.y >= 0 &&
            roi_rect.x + roi_rect.width  <= image.cols &&
            roi_rect.y + roi_rect.height <= image.rows) {
            roi = image(roi_rect).clone();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "ROI矩形无效，使用全图");
            roi = image.clone();
            is_roi_valid = false;
        }
    } else {
        roi = image.clone();
    }
    if (roi.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DartDetector"), "ROI为空");
        return false;
    }

    // ── 2. 预处理：高斯模糊，抑制噪点 ────────────────────────────────────────
    cv::Mat blurred;
    int ks = (blur_ksize_ % 2 == 1) ? blur_ksize_ : blur_ksize_ + 1;
    cv::GaussianBlur(roi, blurred, cv::Size(ks, ks), 0);

    // ── 3. 通道分离 (BGR顺序) ─────────────────────────────────────────────────
    std::vector<cv::Mat> channels(3);
    cv::split(blurred, channels);
    cv::Mat& ch_b = channels[0];   // Blue
    cv::Mat& ch_g = channels[1];   // Green
    cv::Mat& ch_r = channels[2];   // Red

    // ── 4. 构建"绿色得分图" ───────────────────────────────────────────────────
    //
    // 目标特性：外圈是纯绿（G高，R/B低），中心是白色（R/G/B都高）
    //
    // mask_green_halo: 捕获纯绿光晕区域
    //   条件：G - max(R,B) > green_diff_thresh_  &&  G > green_abs_thresh_
    //
    // mask_white_core: 捕获中心高亮白色区域
    //   条件：G > green_abs_thresh_ * 1.5  &&  R > green_abs_thresh_  &&  B > green_abs_thresh_/2
    //   (白色区域三通道都亮，但仍要求G最强，排除纯红/纯蓝干扰)
    //
    // 两者合并，再做闭运算填充空洞

    cv::Mat max_rb;
    cv::max(ch_r, ch_b, max_rb);  // max(R, B)

    // green_diff = G - max(R,B)，用saturate防下溢
    cv::Mat green_diff;
    cv::subtract(ch_g, max_rb, green_diff);  // 结果在 [0, 255]（saturate_cast）

    // mask_green_halo
    cv::Mat mask_green_halo;
    cv::threshold(green_diff, mask_green_halo,
                  static_cast<double>(green_diff_thresh_), 255, cv::THRESH_BINARY);
    {
        cv::Mat g_bright;
        cv::threshold(ch_g, g_bright,
                      static_cast<double>(green_abs_thresh_), 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask_green_halo, g_bright, mask_green_halo);
    }

    // mask_white_core：中心白色（三通道均亮，G仍需最强）
    cv::Mat mask_white_core;
    {
        double wt = green_abs_thresh_ * 1.5;
        wt = std::min(wt, 254.0);
        cv::Mat g_very_bright, r_bright, b_moderate, g_dominant;
        cv::threshold(ch_g, g_very_bright, wt, 255, cv::THRESH_BINARY);
        cv::threshold(ch_r, r_bright,      static_cast<double>(green_abs_thresh_) * 0.8, 255, cv::THRESH_BINARY);
        cv::threshold(ch_b, b_moderate,    static_cast<double>(green_abs_thresh_) * 0.4, 255, cv::THRESH_BINARY);
        // G必须 > R（排除纯红/黄光）
        cv::Mat g_gt_r;
        cv::compare(ch_g, ch_r, g_gt_r, cv::CMP_GT);
        cv::bitwise_and(g_very_bright, r_bright,   mask_white_core);
        cv::bitwise_and(mask_white_core, b_moderate, mask_white_core);
        cv::bitwise_and(mask_white_core, g_gt_r,     mask_white_core);
    }

    // 合并两个掩码
    cv::Mat color_mask;
    cv::bitwise_or(mask_green_halo, mask_white_core, color_mask);

    // ── 5. 形态学操作 ─────────────────────────────────────────────────────────
    // Close：填充中心白色区域与绿色光晕之间的空洞，让整个灯点连成一片
    // Open：去除细小噪点
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::Mat kernel_open  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel_close);
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN,  kernel_open);

    if (color_mask.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("DartDetector"), "颜色掩码为空");
        return false;
    }

    // ── 6. 生成全图尺寸二值图（用于发布调试图像）────────────────────────────
    binary_image = cv::Mat::zeros(image.size(), CV_8UC1);
    if (is_roi_valid) {
        color_mask.copyTo(binary_image(cv::Rect(0, y_min_, color_mask.cols, color_mask.rows)));
    } else {
        color_mask.copyTo(binary_image);
    }

    // ── 7. 轮廓查找与筛选 ────────────────────────────────────────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    all_contours = contours;

    bool found = false;
    cv::Point2f best_center(0, 0);
    double max_valid_area = 0.0;

    for (const auto& contour : contours) {
        if (contour.empty()) continue;

        cv::Rect rect = cv::boundingRect(contour);
        if (rect.width == 0 || rect.height == 0) continue;

        // 长宽比筛选
        double aspect_ratio = static_cast<double>(std::max(rect.width, rect.height)) /
                              std::min(rect.width, rect.height);
        if (aspect_ratio > aspect_ratio_threshold_) continue;

        // 面积与周长
        double perimeter = cv::arcLength(contour, true);
        double area      = cv::contourArea(contour);
        if (area <= 0 || perimeter <= 0) continue;

        // 半径筛选（等效圆半径）
        double radius = std::sqrt(area / CV_PI);
        if (radius < min_radius_ || radius > max_radius_) continue;

        // 圆形度筛选
        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < circularity_threshold_) continue;

        // 计算质心
        cv::Moments mom = cv::moments(contour, false);
        if (mom.m00 == 0) continue;

        if (area > max_valid_area) {
            max_valid_area = area;
            best_center.x  = static_cast<float>(mom.m10 / mom.m00);
            best_center.y  = static_cast<float>(mom.m01 / mom.m00 + (is_roi_valid ? y_min_ : 0));

            best_contour = contour;
            if (is_roi_valid) {
                for (auto& pt : best_contour) pt.y += y_min_;
            }

            found = true;

            if (enable_debug_) {
                RCLCPP_INFO(rclcpp::get_logger("DartDetector"),
                            "候选轮廓: 面积=%.1f 半径=%.1f 长宽比=%.2f 圆形度=%.2f 中心=(%.1f,%.1f)",
                            area, radius, aspect_ratio, circularity,
                            best_center.x, best_center.y);
            }
        }
    }

    if (found) {
        center    = best_center;
        best_area = max_valid_area;
        if (enable_debug_) {
            RCLCPP_INFO(rclcpp::get_logger("DartDetector"),
                        "检测成功: 中心=(%.1f,%.1f) 面积=%.1f",
                        center.x, center.y, best_area);
        }
    } else if (enable_debug_ && !contours.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("DartDetector"),
                    "找到 %zu 个轮廓，均不符合条件", contours.size());
    }

    return found;
}

}  // namespace pka