#include "dart_detector/dart_detector_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace dart_detector {

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions& options) 
    : Node("dart_detector", options),
      has_printed_image_size_(false) {
    declare_parameters();
    readParameters();
    
    detector_.setParameters(h_min_, h_max_, s_min_, s_max_, v_min_, v_max_,
                          min_radius_, max_radius_, aspect_ratio_threshold_,
                          y_min_, y_max_, circularity_threshold_, enable_debug_);
    
    if (y_min_ >= y_max_ || y_min_ < 0 || y_max_ <= 0) {
        RCLCPP_WARN(get_logger(), "无效的ROI y范围，使用全图");
        y_min_ = 0;
        y_max_ = 0;
    }
    
    printParameters();
    
    initImageTransport();
    
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&DartDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    
    // 新增：初始化Light消息发布器，话题名为"detected_light"
    light_pub_ = this->create_publisher<dart_interface::msg::Light>("detected_light", 10);
    
    RCLCPP_INFO(get_logger(), "已订阅图像话题: %s (QoS: SensorData)", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "已创建图像发布话题: raw_image 和 processed_image");
    RCLCPP_INFO(get_logger(), "已创建目标坐标发布话题: detected_light");  // 新增日志
    
    screenshot_manager_ = std::make_unique<ScreenshotManager>(
        this,
        screenshot_path_, enable_auto_screenshot_, 
        screenshot_interval_, save_raw_image_, save_processed_image_);
    
    video_recorder_ = std::make_unique<VideoRecorder>(
        this,
        enable_video_recording_, recording_path_, 
        recording_fps_, fourcc_codec_);
    
    RCLCPP_INFO(get_logger(), "Dart detector node初始化成功，模式: %s", mode_.c_str());
}

void DartDetectorNode::initImageTransport() {
    try {
        it_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
        raw_image_pub_ = it_->advertise("raw_image", 10);
        processed_image_pub_ = it_->advertise("processed_image", 10);
    } catch (const std::bad_weak_ptr& e) {
        RCLCPP_FATAL(get_logger(), "初始化ImageTransport失败: %s", e.what());
        rclcpp::shutdown();
    }
}

void DartDetectorNode::declare_parameters() {
    // 保持不变（参数声明逻辑未变）
    declare_parameter("mode", "camera");
    declare_parameter("video_path", "path/to/your/video.mp4");
    declare_parameter("video_fps", 30.0);
    declare_parameter("image_topic", "image_raw");
    
    declare_parameter("h_min", 40.0);
    declare_parameter("h_max", 80.0);
    declare_parameter("s_min", 100.0);
    declare_parameter("s_max", 255.0);
    declare_parameter("v_min", 100.0);
    declare_parameter("v_max", 255.0);
    
    declare_parameter("y_min", 0);
    declare_parameter("y_max", 0);
    
    declare_parameter("aspect_ratio_threshold", 1.2);
    declare_parameter("circularity_threshold", 0.75);
    
    declare_parameter("min_radius", 5.0);
    declare_parameter("max_radius", 50.0);
    
    declare_parameter("enable_debug", true);
    
    declare_parameter("screenshot_path", "./screenshots");
    declare_parameter("enable_auto_screenshot", false);
    declare_parameter("screenshot_interval", 10.0);
    declare_parameter("save_raw_image", true);
    declare_parameter("save_processed_image", true);
    
    declare_parameter("enable_video_recording", false);
    declare_parameter("recording_path", "./recordings");
    declare_parameter("recording_fps", 30.0);
    declare_parameter("fourcc_codec", "mp4v");
}

void DartDetectorNode::readParameters() {
    // 保持不变（参数读取逻辑未变）
    mode_ = get_parameter("mode").as_string();
    video_path_ = get_parameter("video_path").as_string();
    video_fps_ = get_parameter("video_fps").as_double();
    image_topic_ = get_parameter("image_topic").as_string();
    
    h_min_ = get_parameter("h_min").as_double();
    h_max_ = get_parameter("h_max").as_double();
    s_min_ = get_parameter("s_min").as_double();
    s_max_ = get_parameter("s_max").as_double();
    v_min_ = get_parameter("v_min").as_double();
    v_max_ = get_parameter("v_max").as_double();
    
    y_min_ = get_parameter("y_min").as_int();
    y_max_ = get_parameter("y_max").as_int();
    
    aspect_ratio_threshold_ = get_parameter("aspect_ratio_threshold").as_double();
    circularity_threshold_ = get_parameter("circularity_threshold").as_double();
    
    min_radius_ = get_parameter("min_radius").as_double();
    max_radius_ = get_parameter("max_radius").as_double();
    
    enable_debug_ = get_parameter("enable_debug").as_bool();
    
    screenshot_path_ = get_parameter("screenshot_path").as_string();
    enable_auto_screenshot_ = get_parameter("enable_auto_screenshot").as_bool();
    screenshot_interval_ = get_parameter("screenshot_interval").as_double();
    save_raw_image_ = get_parameter("save_raw_image").as_bool();
    save_processed_image_ = get_parameter("save_processed_image").as_bool();
    
    enable_video_recording_ = get_parameter("enable_video_recording").as_bool();
    recording_path_ = get_parameter("recording_path").as_string();
    recording_fps_ = get_parameter("recording_fps").as_double();
    fourcc_codec_ = get_parameter("fourcc_codec").as_string();
}

void DartDetectorNode::printParameters() {
    // 保持不变（参数打印逻辑未变）
    RCLCPP_INFO(get_logger(), "===== 加载的参数 =====");
    RCLCPP_INFO(get_logger(), "运行模式: %s", mode_.c_str());
    RCLCPP_INFO(get_logger(), "图像话题: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "HSV参数: H[%.1f, %.1f], S[%.1f, %.1f], V[%.1f, %.1f]",
               h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
    RCLCPP_INFO(get_logger(), "ROI参数: y_min=%d, y_max=%d", y_min_, y_max_);
    RCLCPP_INFO(get_logger(), "形状筛选: 长宽比阈值=%.2f, 圆形度阈值=%.2f",
               aspect_ratio_threshold_, circularity_threshold_);
    RCLCPP_INFO(get_logger(), "半径范围: 最小=%.1f, 最大=%.1f", min_radius_, max_radius_);
    RCLCPP_INFO(get_logger(), "调试模式: %s", enable_debug_ ? "启用" : "禁用");
    RCLCPP_INFO(get_logger(), "截图设置: 路径=%s, 自动截图=%s, 间隔=%.1f秒",
               screenshot_path_.c_str(), 
               enable_auto_screenshot_ ? "启用" : "禁用",
               screenshot_interval_);
    RCLCPP_INFO(get_logger(), "截图保存: 原始图像=%s, 处理后图像=%s",
               save_raw_image_ ? "启用" : "禁用",
               save_processed_image_ ? "启用" : "禁用");
    RCLCPP_INFO(get_logger(), "视频录制: %s, 路径=%s, 帧率=%.1f, 编码=%s",
               enable_video_recording_ ? "启用" : "禁用",
               recording_path_.c_str(), recording_fps_, fourcc_codec_.c_str());
    RCLCPP_INFO(get_logger(), "====================");
}

void DartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv::Mat input_image;

        try {
            if (msg->encoding == "rgb8") {
                cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
                input_image = cv_ptr->image.clone();
                cv::cvtColor(input_image, input_image, cv::COLOR_RGB2BGR);
            } else if (msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvShare(msg, "mono8");
                cv::cvtColor(cv_ptr->image, input_image, cv::COLOR_GRAY2BGR);
            } else {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                input_image = cv_ptr->image.clone();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge转换错误: %s", e.what());
            return;
        }
        
        if (input_image.empty()) {
            RCLCPP_WARN(get_logger(), "收到空图像，跳过处理");
            return;
        }
        
        if (enable_debug_ && !has_printed_image_size_) {
            RCLCPP_INFO(get_logger(), "图像尺寸: 宽度=%d, 高度=%d, 编码=%s", 
                       input_image.cols, input_image.rows, msg->encoding.c_str());
            has_printed_image_size_ = true;
        }

        cv::Mat raw_image = input_image.clone();
        cv::Mat processed_image;
        cv::Point2f target_center;  // 新增：存储目标中心坐标
        
        // 修改：调用detect时传入坐标参数
        bool detected = detector_.detect(input_image, processed_image, target_center);
        
        if (enable_debug_) {
            RCLCPP_DEBUG(get_logger(), "检测结果: %s", detected ? "成功" : "失败");
        }
        
        publishImage(raw_image_pub_, raw_image, msg->header.stamp);
        publishImage(processed_image_pub_, processed_image, msg->header.stamp);
        
        screenshot_manager_->setLatestImages(raw_image, processed_image);
        
        if (enable_video_recording_) {
            video_recorder_->processImage(processed_image);
        }
        
        // 新增：如果检测到目标，发布Light消息
        if (detected && target_center.x >= 0 && target_center.y >= 0) {
            auto light_msg = dart_interface::msg::Light();
            light_msg.x = target_center.x;  // 像素坐标系x
            light_msg.y = target_center.y;  // 像素坐标系y
            light_pub_->publish(light_msg);
            
            if (enable_debug_) {
                RCLCPP_DEBUG(get_logger(), "发布目标坐标: (%.2f, %.2f)", light_msg.x, light_msg.y);
            }
        }
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV异常: %s", e.what());
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "异常: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "未知错误发生在图像处理回调中");
    }
}

void DartDetectorNode::publishImage(image_transport::Publisher& pub, const cv::Mat& image, rclcpp::Time stamp) {
    // 保持不变（图像发布逻辑未变）
    if (image.empty() || !pub.getNumSubscribers()) {
        return;
    }
    
    try {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = stamp;
        pub.publish(*msg);
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时OpenCV异常: %s", e.what());
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时cv_bridge异常: %s", e.what());
    }
}

}  // namespace dart_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dart_detector::DartDetectorNode)