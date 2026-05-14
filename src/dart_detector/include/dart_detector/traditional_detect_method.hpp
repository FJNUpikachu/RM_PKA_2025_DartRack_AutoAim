#ifndef DART_DETECTOR_TRADITIONAL_DETECT_METHOD_HPP
#define DART_DETECTOR_TRADITIONAL_DETECT_METHOD_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace pka {

class TraditionalDartDetector {
public:
    TraditionalDartDetector();

    /**
     * @brief 设置检测参数
     *
     * @param green_diff_thresh  G - max(R,B) 差值阈值，用于提取纯绿光晕区域
     *                           典型值: 20~50（场景越暗越小）
     * @param green_abs_thresh   G 通道绝对亮度阈值，用于提取中心白色高亮区域
     *                           典型值: 60~120
     * @param blur_ksize         预处理高斯模糊核大小（奇数），抑制噪点
     *                           典型值: 3~7
     * @param min_radius         等效圆最小半径（像素）
     * @param max_radius         等效圆最大半径（像素）
     * @param aspect_ratio_threshold  外接矩形长宽比上限（越接近1越圆）
     * @param y_min              ROI 起始行（0 表示不裁剪）
     * @param y_max              ROI 结束行（0 或 <= y_min 表示不裁剪）
     * @param circularity_threshold  圆形度下限 [0,1]，建议 0.6~0.8
     * @param enable_debug       是否输出调试日志
     */
    void setParameters(
        double green_diff_thresh, double green_abs_thresh, int blur_ksize,
        double min_radius, double max_radius, double aspect_ratio_threshold,
        int y_min, int y_max, double circularity_threshold, bool enable_debug);

    bool detect(const cv::Mat& image, cv::Mat& binary_image,
                std::vector<std::vector<cv::Point>>& all_contours,
                std::vector<cv::Point>& best_contour,
                cv::Point2f& center, double& best_area);

private:
    // RGB 检测参数
    double green_diff_thresh_;   // G - max(R,B) 阈值
    double green_abs_thresh_;    // G 绝对亮度阈值

    // 预处理
    int blur_ksize_;

    // 形状筛选
    double min_radius_, max_radius_;
    double aspect_ratio_threshold_;
    double circularity_threshold_;

    // ROI
    int y_min_, y_max_;

    bool enable_debug_;
};

}  // namespace pka

#endif  // DART_DETECTOR_TRADITIONAL_DETECT_METHOD_HPP