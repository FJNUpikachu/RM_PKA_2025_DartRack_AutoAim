#ifndef DART_DETECTOR_DART_DETECTOR_NODE_HPP_
#define DART_DETECTOR_DART_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "dart_detector/detect_method.hpp"
#include "dart_detector/screenshot_manager.hpp"
#include "dart_detector/video_recorder.hpp"
// 引入Light消息类型
#include "dart_interface/msg/light.hpp"
#include <memory>

namespace dart_detector {

class DartDetectorNode : public rclcpp::Node {
public:
    explicit DartDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DartDetectorNode() = default;

private:
    void initImageTransport();
    void declare_parameters();
    void readParameters();
    void printParameters();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void publishImage(image_transport::Publisher& pub, const cv::Mat& image, rclcpp::Time stamp);

    DartDetector detector_;
    
    std::string mode_;
    std::string video_path_;
    double video_fps_;
    std::string image_topic_;
    
    double h_min_, h_max_;
    double s_min_, s_max_;
    double v_min_, v_max_;
    int y_min_, y_max_;
    double aspect_ratio_threshold_;
    double circularity_threshold_;
    double min_radius_, max_radius_;
    bool enable_debug_;
    bool has_printed_image_size_;
    
    std::string screenshot_path_;
    bool enable_auto_screenshot_;
    double screenshot_interval_;
    bool save_raw_image_;
    bool save_processed_image_;
    
    bool enable_video_recording_;
    std::string recording_path_;
    double recording_fps_;
    std::string fourcc_codec_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    
    // 图像传输相关成员
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher raw_image_pub_;
    image_transport::Publisher processed_image_pub_;
    
    // 新增：Light消息发布器
    rclcpp::Publisher<dart_interface::msg::Light>::SharedPtr light_pub_;
    
    std::unique_ptr<ScreenshotManager> screenshot_manager_;
    std::unique_ptr<VideoRecorder> video_recorder_;
};

}  // namespace dart_detector

#endif  // DART_DETECTOR_DART_DETECTOR_NODE_HPP_