#ifndef DART_DETECTOR_TRADITIONAL_DETECTOR_NODE_HPP
#define DART_DETECTOR_TRADITIONAL_DETECTOR_NODE_HPP

#include "dart_detector/traditional_detect_method.hpp"
#include "dart_interfaces/msg/light.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <string>

namespace pka {

class TraditionalDartDetectorNode : public rclcpp::Node {
public:
    explicit TraditionalDartDetectorNode(const rclcpp::NodeOptions& options);
    
private:
    void declare_parameters();
    void readParameters();
    void printParameters();
    
    void initImageTransport();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void publishImage(image_transport::Publisher& pub, const cv::Mat& image, rclcpp::Time stamp);
    void publishBinaryImage(const cv::Mat& binary_image, rclcpp::Time stamp);
    
    // ROS2 订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<dart_interfaces::msg::Light>::SharedPtr light_pub_;
    
    // 图像传输
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher raw_image_pub_;
    image_transport::Publisher result_image_pub_;
    image_transport::Publisher binary_image_pub_;
    image_transport::Publisher contour_image_pub_;
    
    // 基础参数
    std::string mode_;
    std::string video_path_;
    double      video_fps_;
    std::string image_topic_;
    
    // RGB 检测参数（替换原 HSV 参数）
    double green_diff_thresh_;   ///< G - max(R,B) 差值阈值，捕获纯绿光晕
    double green_abs_thresh_;    ///< G 通道绝对亮度阈值，捕获白色高亮中心
    int    blur_ksize_;          ///< 预处理高斯模糊核大小

    // ROI
    int y_min_, y_max_;
    
    // 形状筛选
    double aspect_ratio_threshold_;
    double circularity_threshold_;
    double min_radius_, max_radius_;
    
    bool enable_debug_;
    
    // 状态标志
    bool has_printed_image_size_;
    bool image_transport_initialized_;
    
    // 检测器
    TraditionalDartDetector detector_;
};

}  // namespace pka

#endif  // DART_DETECTOR_TRADITIONAL_DETECTOR_NODE_HPP