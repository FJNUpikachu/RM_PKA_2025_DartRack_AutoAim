#include "dart_detector/traditional_detector_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <chrono>

namespace pka {

TraditionalDartDetectorNode::TraditionalDartDetectorNode(const rclcpp::NodeOptions& options) 
    : Node("dart_detector", options),
      has_printed_image_size_(false),
      image_transport_initialized_(false) {
    declare_parameters();
    readParameters();
    
    detector_.setParameters(green_diff_thresh_, green_abs_thresh_, blur_ksize_,
                            min_radius_, max_radius_, aspect_ratio_threshold_,
                            y_min_, y_max_, circularity_threshold_, enable_debug_);
    
    if (y_min_ >= y_max_ || y_min_ < 0 || y_max_ <= 0) {
        RCLCPP_WARN(get_logger(), "无效的ROI y范围，使用全图");
        y_min_ = 0;
        y_max_ = 0;
    }
    
    printParameters();
    
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.reliable();
    qos.durability_volatile();
    
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        qos, 
        std::bind(&TraditionalDartDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    
    light_pub_ = this->create_publisher<dart_interfaces::msg::Light>(
        "light_position", 
        10
    );
    
    RCLCPP_INFO(get_logger(), "已订阅图像话题: %s (QoS: RELIABLE)", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "已创建灯位置发布话题: light_position");
    RCLCPP_INFO(get_logger(), "Dart detector node初始化成功，模式: %s", mode_.c_str());
}

void TraditionalDartDetectorNode::initImageTransport() {
    if (!image_transport_initialized_) {
        try {
            it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
            raw_image_pub_     = it_->advertise("raw_image",    10);
            result_image_pub_  = it_->advertise("result_img",   10);
            binary_image_pub_  = it_->advertise("binary_image", 10);
            contour_image_pub_ = it_->advertise("contour_image",10);
            
            image_transport_initialized_ = true;
            RCLCPP_INFO(get_logger(), "图像传输对象初始化成功");
            RCLCPP_INFO(get_logger(), "已创建图像发布话题: raw_image, result_img, binary_image, contour_image");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "图像传输对象初始化失败: %s", e.what());
            rclcpp::shutdown();
        }
    }
}

void TraditionalDartDetectorNode::declare_parameters() {
    declare_parameter("mode", "camera");
    declare_parameter("video_path", "path/to/your/video.mp4");
    declare_parameter("video_fps", 30.0);
    declare_parameter("image_topic", "image_raw");
    
    // RGB 检测参数（替换 HSV）
    declare_parameter("green_diff_thresh", 30.0);   // G - max(R,B) 差值阈值
    declare_parameter("green_abs_thresh",  80.0);   // G 通道绝对亮度阈值
    declare_parameter("blur_ksize", 5);             // 高斯模糊核大小
    
    declare_parameter("y_min", 0);
    declare_parameter("y_max", 0);
    
    declare_parameter("aspect_ratio_threshold", 1.5);
    declare_parameter("circularity_threshold", 0.65);
    
    declare_parameter("min_radius", 5.0);
    declare_parameter("max_radius", 100.0);
    
    declare_parameter("enable_debug", false);
}

void TraditionalDartDetectorNode::readParameters() {
    mode_        = get_parameter("mode").as_string();
    video_path_  = get_parameter("video_path").as_string();
    video_fps_   = get_parameter("video_fps").as_double();
    image_topic_ = get_parameter("image_topic").as_string();
    
    green_diff_thresh_ = get_parameter("green_diff_thresh").as_double();
    green_abs_thresh_  = get_parameter("green_abs_thresh").as_double();
    blur_ksize_        = get_parameter("blur_ksize").as_int();
    
    y_min_ = get_parameter("y_min").as_int();
    y_max_ = get_parameter("y_max").as_int();
    
    aspect_ratio_threshold_ = get_parameter("aspect_ratio_threshold").as_double();
    circularity_threshold_  = get_parameter("circularity_threshold").as_double();
    
    min_radius_ = get_parameter("min_radius").as_double();
    max_radius_ = get_parameter("max_radius").as_double();
    
    enable_debug_ = get_parameter("enable_debug").as_bool();
}

void TraditionalDartDetectorNode::printParameters() {
    RCLCPP_INFO(get_logger(), "===== 加载的参数 =====");
    RCLCPP_INFO(get_logger(), "运行模式: %s", mode_.c_str());
    RCLCPP_INFO(get_logger(), "图像话题: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "RGB检测参数: green_diff_thresh=%.1f, green_abs_thresh=%.1f, blur_ksize=%d",
               green_diff_thresh_, green_abs_thresh_, blur_ksize_);
    RCLCPP_INFO(get_logger(), "ROI参数: y_min=%d, y_max=%d", y_min_, y_max_);
    RCLCPP_INFO(get_logger(), "形状筛选: 长宽比阈值=%.2f, 圆形度阈值=%.2f",
               aspect_ratio_threshold_, circularity_threshold_);
    RCLCPP_INFO(get_logger(), "半径范围: 最小=%.1f, 最大=%.1f", min_radius_, max_radius_);
    RCLCPP_INFO(get_logger(), "调试模式: %s", enable_debug_ ? "启用" : "禁用");
    RCLCPP_INFO(get_logger(), "====================");
}

void TraditionalDartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!image_transport_initialized_) {
        initImageTransport();
    }
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        RCLCPP_DEBUG(get_logger(), "收到图像消息，时间戳: %ld.%09ld", 
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat input_image;

        try {
            RCLCPP_DEBUG(get_logger(), "收到图像: 编码=%s, 尺寸=%dx%d", 
                        msg->encoding.c_str(), msg->width, msg->height);
            
            if (msg->encoding == "rgb8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
                input_image = cv_ptr->image;
                cv::cvtColor(input_image, input_image, cv::COLOR_RGB2BGR);
            } else if (msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                input_image = cv_ptr->image;
            } else if (msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
                cv::cvtColor(cv_ptr->image, input_image, cv::COLOR_GRAY2BGR);
            } else {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                input_image = cv_ptr->image;
                RCLCPP_WARN(get_logger(), "未知编码格式: %s，尝试强制转换到BGR8", 
                           msg->encoding.c_str());
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge转换错误: %s", e.what());
            return;
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "OpenCV颜色转换错误: %s", e.what());
            return;
        }
        
        if (input_image.empty()) {
            RCLCPP_WARN(get_logger(), "收到空图像，跳过处理");
            return;
        }
        
        if (enable_debug_ && !has_printed_image_size_) {
            RCLCPP_INFO(get_logger(), "图像尺寸: 宽度=%d, 高度=%d, 通道数=%d, 编码=%s", 
                       input_image.cols, input_image.rows, input_image.channels(),
                       msg->encoding.c_str());
            has_printed_image_size_ = true;
        }

        cv::Mat raw_image = input_image.clone();
        cv::Mat binary_image;
        std::vector<std::vector<cv::Point>> all_contours;
        std::vector<cv::Point> best_contour;
        cv::Point2f light_center(0, 0);
        double best_area = 0.0;
        
        if (input_image.channels() != 3) {
            RCLCPP_WARN(get_logger(), "输入图像不是3通道BGR格式，实际通道数: %d，跳过处理", 
                       input_image.channels());
            return;
        }
        
        bool detected = detector_.detect(input_image, binary_image, all_contours, best_contour, light_center, best_area);
        
        // 发布灯位置消息
        auto light_msg = dart_interfaces::msg::Light();
        light_msg.header.stamp    = msg->header.stamp;
        light_msg.header.frame_id = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;
        
        if (detected) {
            light_msg.x = light_center.x;
            light_msg.y = light_center.y;
            if (enable_debug_) {
                RCLCPP_INFO(get_logger(), "发布灯位置: x=%.1f, y=%.1f, 面积=%.1f", 
                           light_center.x, light_center.y, best_area);
            }
        } else {
            light_msg.x = 0.0;
            light_msg.y = 0.0;
            if (enable_debug_) {
                RCLCPP_DEBUG(get_logger(), "未检测到灯，发布默认位置(0,0)");
            }
        }
        
        if (enable_debug_) {
            size_t subscribers_count = light_pub_->get_subscription_count();
            RCLCPP_DEBUG(get_logger(), "light_position话题订阅者数量: %zu", subscribers_count);
        }
        
        try {
            light_pub_->publish(light_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "发布light_position消息时出错: %s", e.what());
        }
        
        // 计算处理时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (enable_debug_) {
            RCLCPP_DEBUG(get_logger(), "检测结果: %s, 处理时间: %ldms", 
                        detected ? "成功" : "失败", duration.count());
        }
        
        // ── 绘制结果图像 ──────────────────────────────────────────────────────
        cv::Mat result_image  = raw_image.clone();
        cv::Mat contour_image = raw_image.clone();
        
        bool is_roi_valid = (y_min_ >= 0 && y_max_ > y_min_ && y_max_ <= result_image.rows);
        if (is_roi_valid) {
            cv::rectangle(result_image,  cv::Point(0, y_min_), cv::Point(result_image.cols,  y_max_), cv::Scalar(0, 255, 0), 2);
            cv::rectangle(contour_image, cv::Point(0, y_min_), cv::Point(contour_image.cols, y_max_), cv::Scalar(0, 255, 0), 2);
            cv::putText(result_image,  "ROI", cv::Point(10, y_min_ - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            cv::putText(contour_image, "ROI", cv::Point(10, y_min_ - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        
        // 中心十字线
        int center_x = result_image.cols / 2;
        int center_y = result_image.rows / 2;
        cv::line(result_image,  cv::Point(center_x, 0), cv::Point(center_x, result_image.rows),  cv::Scalar(0, 0, 255), 2);
        cv::line(result_image,  cv::Point(0, center_y), cv::Point(result_image.cols, center_y),  cv::Scalar(0, 0, 255), 2);
        cv::line(contour_image, cv::Point(center_x, 0), cv::Point(center_x, contour_image.rows), cv::Scalar(0, 0, 255), 2);
        cv::line(contour_image, cv::Point(0, center_y), cv::Point(contour_image.cols, center_y), cv::Scalar(0, 0, 255), 2);
        cv::circle(result_image,  cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(contour_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 255), -1);
        
        // 绘制所有轮廓（绿色）
        if (!all_contours.empty()) {
            std::vector<std::vector<cv::Point>> global_contours = all_contours;
            for (auto& contour : global_contours) {
                for (auto& point : contour) {
                    point.y += y_min_;
                }
            }
            cv::drawContours(contour_image, global_contours, -1, cv::Scalar(0, 255, 0), 2);
        }
        
        // 绘制检测结果
        if (detected) {
            if (light_center.x >= 0 && light_center.x < result_image.cols &&
                light_center.y >= 0 && light_center.y < result_image.rows) {
                
                // 用等效半径画圈（比固定30更准确）
                int disp_radius = static_cast<int>(std::sqrt(best_area / CV_PI));
                disp_radius = std::max(disp_radius, 8);
                
                cv::circle(result_image,  light_center, disp_radius, cv::Scalar(0, 0, 255), 3);
                cv::circle(result_image,  light_center, 6, cv::Scalar(0, 0, 255), -1);
                cv::circle(contour_image, light_center, 6, cv::Scalar(0, 0, 255), -1);
                
                if (!best_contour.empty()) {
                    std::vector<std::vector<cv::Point>> bv = {best_contour};
                    cv::drawContours(contour_image, bv, 0, cv::Scalar(0, 0, 255), 3);
                    if (enable_debug_) {
                        cv::drawContours(result_image, bv, 0, cv::Scalar(255, 0, 0), 2);
                    }
                }
                
                std::string light_text = "Light: (" + std::to_string(static_cast<int>(light_center.x)) + 
                                        ", " + std::to_string(static_cast<int>(light_center.y)) + ")";
                cv::putText(result_image,  light_text, cv::Point(static_cast<int>(light_center.x + 20), static_cast<int>(light_center.y - 20)), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                cv::putText(contour_image, light_text, cv::Point(static_cast<int>(light_center.x + 20), static_cast<int>(light_center.y - 20)), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                
                cv::line(result_image,  cv::Point(center_x, center_y), light_center, cv::Scalar(255, 0, 0), 2);
                cv::line(contour_image, cv::Point(center_x, center_y), light_center, cv::Scalar(255, 0, 0), 2);
                
                float offset_x = light_center.x - center_x;
                float offset_y = center_y - light_center.y;
                cv::Point mid_point((center_x + static_cast<int>(light_center.x)) / 2,
                                   (center_y + static_cast<int>(light_center.y)) / 2);
                std::string offset_text = "Offset: (" + std::to_string(static_cast<int>(offset_x)) + 
                                        ", " + std::to_string(static_cast<int>(offset_y)) + ")";
                cv::putText(result_image,  offset_text, cv::Point(mid_point.x + 10, mid_point.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
                cv::putText(contour_image, offset_text, cv::Point(mid_point.x + 10, mid_point.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            }
        }
        
        cv::putText(result_image,  "Center", cv::Point(center_x + 10, center_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::putText(contour_image, "Center", cv::Point(center_x + 10, center_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        std::string status_text  = detected ? "Detected" : "Not Detected";
        cv::Scalar  status_color = detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(result_image,  status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2);
        cv::putText(contour_image, status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2);
        
        if (detected) {
            std::string area_text = "Area: " + std::to_string(static_cast<int>(best_area));
            cv::putText(result_image,  area_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            cv::putText(contour_image, area_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }
        
        std::string fps_text = "FPS: " + std::to_string(1000 / std::max(1L, duration.count()));
        cv::putText(result_image,  fps_text, cv::Point(result_image.cols  - 120, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(contour_image, fps_text, cv::Point(contour_image.cols - 120, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        
        if (image_transport_initialized_) {
            if (!raw_image.empty())    publishImage(raw_image_pub_,    raw_image,    msg->header.stamp);
            if (!result_image.empty()) publishImage(result_image_pub_, result_image, msg->header.stamp);
            if (!binary_image.empty()) publishBinaryImage(binary_image, msg->header.stamp);
            if (!contour_image.empty()) publishImage(contour_image_pub_, contour_image, msg->header.stamp);
        }
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV异常: %s (file=%s, line=%d)", e.what(), e.file.c_str(), e.line);
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "异常: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "未知错误发生在图像处理回调中");
    }
}

void TraditionalDartDetectorNode::publishImage(image_transport::Publisher& pub, const cv::Mat& image, rclcpp::Time stamp) {
    if (image.empty() || !pub.getNumSubscribers()) return;
    
    try {
        if (image.data == nullptr || image.cols <= 0 || image.rows <= 0) {
            RCLCPP_WARN(get_logger(), "尝试发布无效图像，跳过");
            return;
        }
        cv::Mat img_to_publish = image.clone();
        if (img_to_publish.empty()) return;
        
        if (img_to_publish.channels() != 3) {
            if (img_to_publish.channels() == 1) {
                cv::cvtColor(img_to_publish, img_to_publish, cv::COLOR_GRAY2BGR);
            } else {
                return;
            }
        }
        if (img_to_publish.depth() != CV_8U) {
            img_to_publish.convertTo(img_to_publish, CV_8U);
        }
        
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp    = stamp;
        cv_image.header.frame_id = "camera";
        cv_image.encoding        = "bgr8";
        cv_image.image           = img_to_publish;
        pub.publish(*cv_image.toImageMsg());
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时OpenCV异常: %s", e.what());
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时异常: %s", e.what());
    }
}

void TraditionalDartDetectorNode::publishBinaryImage(const cv::Mat& binary_image, rclcpp::Time stamp) {
    if (binary_image.empty() || !binary_image_pub_.getNumSubscribers()) return;
    
    try {
        if (binary_image.data == nullptr || binary_image.cols <= 0 || binary_image.rows <= 0) return;
        cv::Mat img_to_publish = binary_image.clone();
        if (img_to_publish.empty()) return;
        
        if (img_to_publish.channels() != 1) {
            if (img_to_publish.channels() == 3) {
                cv::cvtColor(img_to_publish, img_to_publish, cv::COLOR_BGR2GRAY);
            } else {
                return;
            }
        }
        if (img_to_publish.depth() != CV_8U) {
            img_to_publish.convertTo(img_to_publish, CV_8U);
        }
        
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp    = stamp;
        cv_image.header.frame_id = "camera";
        cv_image.encoding        = "mono8";
        cv_image.image           = img_to_publish;
        binary_image_pub_.publish(*cv_image.toImageMsg());
        
        RCLCPP_DEBUG(get_logger(), "已发布二值化图像，尺寸: %dx%d", img_to_publish.cols, img_to_publish.rows);
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布二值化图像时OpenCV异常: %s", e.what());
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布二值化图像时cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "发布二值化图像时异常: %s", e.what());
    }
}

}  // namespace pka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::TraditionalDartDetectorNode)