#include "dart_detector/dart_detector_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <chrono>

namespace pka {

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions& options) 
    : Node("dart_detector", options),
      has_printed_image_size_(false),
      image_transport_initialized_(false) {
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
    
    // 使用更安全的线程模型 - 显式指定QoS
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.reliable();
    qos.durability_volatile();
    
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        qos, 
        std::bind(&DartDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    
    // 创建灯位置发布者
    light_pub_ = this->create_publisher<dart_interfaces::msg::Light>(
        "light_position", 
        10
    );
    
    RCLCPP_INFO(get_logger(), "已订阅图像话题: %s (QoS: RELIABLE)", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "已创建灯位置发布话题: light_position");
    RCLCPP_INFO(get_logger(), "Dart detector node初始化成功，模式: %s", mode_.c_str());
}

void DartDetectorNode::initImageTransport() {
    if (!image_transport_initialized_) {
        try {
            it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
            raw_image_pub_ = it_->advertise("raw_image", 10);
            result_image_pub_ = it_->advertise("result_img", 10);
            // 新增：二值化图像和轮廓图像发布者
            binary_image_pub_ = it_->advertise("binary_image", 10);
            contour_image_pub_ = it_->advertise("contour_image", 10);
            
            image_transport_initialized_ = true;
            
            RCLCPP_INFO(get_logger(), "图像传输对象初始化成功");
            RCLCPP_INFO(get_logger(), "已创建图像发布话题: raw_image, result_img, binary_image, contour_image");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "图像传输对象初始化失败: %s", e.what());
            rclcpp::shutdown();
        }
    }
}

void DartDetectorNode::declare_parameters() {
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
}

void DartDetectorNode::readParameters() {
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
}

void DartDetectorNode::printParameters() {
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
    RCLCPP_INFO(get_logger(), "====================");
}

void DartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // 延迟初始化图像传输对象
    if (!image_transport_initialized_) {
        initImageTransport();
    }
    
    try {
        // 记录开始时间用于性能分析
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 添加调试信息：显示收到图像的时间
        RCLCPP_DEBUG(get_logger(), "收到图像消息，时间戳: %ld.%09ld", 
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat input_image;

        try {
            RCLCPP_DEBUG(get_logger(), "收到图像: 编码=%s, 尺寸=%dx%d", 
                        msg->encoding.c_str(), msg->width, msg->height);
            
            // 根据编码格式进行不同的处理
            if (msg->encoding == "rgb8") {
                // 从RGB8转换到BGR
                cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
                input_image = cv_ptr->image;
                cv::cvtColor(input_image, input_image, cv::COLOR_RGB2BGR);
                RCLCPP_DEBUG(get_logger(), "转换RGB8到BGR");
            } else if (msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                input_image = cv_ptr->image;
            } else if (msg->encoding == "mono8") {
                cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
                cv::cvtColor(cv_ptr->image, input_image, cv::COLOR_GRAY2BGR);
                RCLCPP_DEBUG(get_logger(), "转换GRAY到BGR");
            } else {
                // 尝试通用转换
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
        
        // 确保图像是有效的3通道BGR图像
        if (input_image.channels() != 3) {
            RCLCPP_WARN(get_logger(), "输入图像不是3通道BGR格式，实际通道数: %d，跳过处理", 
                       input_image.channels());
            return;
        }
        
        bool detected = detector_.detect(input_image, binary_image, all_contours, best_contour, light_center, best_area);
        
        // 发布灯位置消息 - 按照您的msg结构只发布x, y和header
        auto light_msg = dart_interfaces::msg::Light();
        light_msg.header.stamp = msg->header.stamp;
        light_msg.header.frame_id = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;
        
        if (detected) {
            // 检测到灯，发布实际坐标
            light_msg.x = light_center.x;
            light_msg.y = light_center.y;
            
            if (enable_debug_) {
                RCLCPP_INFO(get_logger(), "发布灯位置: x=%.1f, y=%.1f, 面积=%.1f", 
                           light_center.x, light_center.y, best_area);
            }
        } else {
            // 未检测到灯，发布默认值(0,0)
            light_msg.x = 0.0;
            light_msg.y = 0.0;
            
            if (enable_debug_) {
                RCLCPP_DEBUG(get_logger(), "未检测到灯，发布默认位置(0,0)");
            }
        }
        
        // 添加更多调试信息：检查发布者状态
        if (enable_debug_) {
            size_t subscribers_count = light_pub_->get_subscription_count();
            RCLCPP_DEBUG(get_logger(), "light_position话题订阅者数量: %zu", subscribers_count);
        }
        
        // 始终发布消息
        try {
            light_pub_->publish(light_msg);
            if (enable_debug_) {
                RCLCPP_DEBUG(get_logger(), "已发布light_position消息");
            }
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
        
        // 创建结果图像 - 总是从原始图像开始
        cv::Mat result_image = raw_image.clone();
        
        // 创建轮廓图像 - 从原始图像开始
        cv::Mat contour_image = raw_image.clone();
        
        // 绘制ROI区域（如果有效）
        bool is_roi_valid = (y_min_ >= 0 && y_max_ > y_min_ && y_max_ <= result_image.rows);
        if (is_roi_valid) {
            // 在结果图像上绘制绿色矩形框出ROI区域
            cv::rectangle(result_image, 
                         cv::Point(0, y_min_), 
                         cv::Point(result_image.cols, y_max_), 
                         cv::Scalar(0, 255, 0),  // 绿色 (BGR格式)
                         2);                     // 线宽2像素
            
            // 在轮廓图像上绘制绿色矩形框出ROI区域
            cv::rectangle(contour_image, 
                         cv::Point(0, y_min_), 
                         cv::Point(contour_image.cols, y_max_), 
                         cv::Scalar(0, 255, 0),  // 绿色
                         2);
            
            // 添加ROI标注
            cv::putText(result_image, 
                       "ROI", 
                       cv::Point(10, y_min_ - 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.6, 
                       cv::Scalar(0, 255, 0),  // 绿色
                       2);
            
            cv::putText(contour_image, 
                       "ROI", 
                       cv::Point(10, y_min_ - 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.6, 
                       cv::Scalar(0, 255, 0),  // 绿色
                       2);
        }
        
        // 绘制中心十字线
        int center_x = result_image.cols / 2;
        int center_y = result_image.rows / 2;
        
        // 绘制中心竖线
        if (center_x >= 0 && center_x < result_image.cols) {
            cv::line(result_image, 
                     cv::Point(center_x, 0), 
                     cv::Point(center_x, result_image.rows), 
                     cv::Scalar(0, 0, 255),  // 红色 (BGR格式)
                     2);                     // 线宽2像素
                     
            cv::line(contour_image, 
                     cv::Point(center_x, 0), 
                     cv::Point(center_x, contour_image.rows), 
                     cv::Scalar(0, 0, 255),  // 红色
                     2);
        }
        
        // 绘制中心横线
        if (center_y >= 0 && center_y < result_image.rows) {
            cv::line(result_image, 
                     cv::Point(0, center_y), 
                     cv::Point(result_image.cols, center_y), 
                     cv::Scalar(0, 0, 255),  // 红色
                     2);
                     
            cv::line(contour_image, 
                     cv::Point(0, center_y), 
                     cv::Point(contour_image.cols, center_y), 
                     cv::Scalar(0, 0, 255),  // 红色
                     2);
        }
        
        // 在中心点绘制一个小圆圈
        cv::circle(result_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 255), -1); // 黄色实心圆
        cv::circle(contour_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 255), -1); // 黄色实心圆
        
        // 绘制所有轮廓（绿色）
        if (!all_contours.empty()) {
            std::vector<std::vector<cv::Point>> global_contours = all_contours;
            for (auto& contour : global_contours) {
                for (auto& point : contour) {
                    point.y += y_min_;  // 转换为全局坐标
                }
            }
            cv::drawContours(contour_image, global_contours, -1, cv::Scalar(0, 255, 0), 2);
        }
        
        // 如果检测到灯，绘制标记
        if (detected) {
            // 确保中心点在图像范围内
            if (light_center.x >= 0 && light_center.x < result_image.cols &&
                light_center.y >= 0 && light_center.y < result_image.rows) {
                
                // 在结果图像上绘制红色圆圈框出引导灯
                cv::circle(result_image, 
                          light_center, 
                          30,  // 固定的半径大小
                          cv::Scalar(0, 0, 255),  // 红色
                          3);                     // 线宽3像素
                
                // 在灯中心绘制更明显的标记
                cv::circle(result_image, 
                          light_center, 
                          8,  // 稍大的中心点
                          cv::Scalar(0, 0, 255),  // 红色
                          -1);                    // 实心圆
                
                // 在轮廓图像上绘制最佳轮廓（红色）
                if (!best_contour.empty()) {
                    std::vector<std::vector<cv::Point>> best_contours_vector;
                    best_contours_vector.push_back(best_contour);
                    cv::drawContours(contour_image, best_contours_vector, 0, cv::Scalar(0, 0, 255), 3);
                    
                    // 也在结果图像上绘制最佳轮廓（蓝色）
                    if (enable_debug_) {
                        cv::drawContours(result_image, best_contours_vector, 0, cv::Scalar(255, 0, 0), 2);
                    }
                }
                
                // 在轮廓图像上绘制中心点
                cv::circle(contour_image, 
                          light_center, 
                          8,  // 中心点大小
                          cv::Scalar(0, 0, 255),  // 红色
                          -1);                    // 实心圆
                
                // 添加文字标注
                std::string light_text = "Light: (" + std::to_string(static_cast<int>(light_center.x)) + 
                                       ", " + std::to_string(static_cast<int>(light_center.y)) + ")";
                cv::putText(result_image, 
                           light_text, 
                           cv::Point(static_cast<int>(light_center.x + 20), 
                                    static_cast<int>(light_center.y - 20)), 
                           cv::FONT_HERSHEY_SIMPLEX, 
                           0.6, 
                           cv::Scalar(0, 0, 255),  // 红色
                           2);
                           
                cv::putText(contour_image, 
                           light_text, 
                           cv::Point(static_cast<int>(light_center.x + 20), 
                                    static_cast<int>(light_center.y - 20)), 
                           cv::FONT_HERSHEY_SIMPLEX, 
                           0.6, 
                           cv::Scalar(0, 0, 255),  // 红色
                           2);
                
                // 绘制从中心到灯的连线
                cv::line(result_image,
                        cv::Point(center_x, center_y),
                        light_center,
                        cv::Scalar(255, 0, 0),  // 蓝色
                        2);
                        
                cv::line(contour_image,
                        cv::Point(center_x, center_y),
                        light_center,
                        cv::Scalar(255, 0, 0),  // 蓝色
                        2);
                
                // 计算偏移量（用于显示，不发布）
                float offset_x = light_center.x - center_x;
                float offset_y = center_y - light_center.y;  // 注意：图像坐标系y轴向下
                
                // 在连线中点添加偏移量信息
                cv::Point mid_point((center_x + light_center.x) / 2, (center_y + light_center.y) / 2);
                std::string offset_text = "Offset: (" + std::to_string(static_cast<int>(offset_x)) + 
                                        ", " + std::to_string(static_cast<int>(offset_y)) + ")";
                cv::putText(result_image,
                          offset_text,
                          cv::Point(mid_point.x + 10, mid_point.y),
                          cv::FONT_HERSHEY_SIMPLEX,
                          0.5,
                          cv::Scalar(255, 0, 0),  // 蓝色
                          1);
                          
                cv::putText(contour_image,
                          offset_text,
                          cv::Point(mid_point.x + 10, mid_point.y),
                          cv::FONT_HERSHEY_SIMPLEX,
                          0.5,
                          cv::Scalar(255, 0, 0),  // 蓝色
                          1);
            }
        }
        
        // 添加图像中心点标注
        cv::putText(result_image, 
                   "Center", 
                   cv::Point(center_x + 10, center_y - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.6, 
                   cv::Scalar(0, 255, 255),  // 黄色
                   2);
                   
        cv::putText(contour_image, 
                   "Center", 
                   cv::Point(center_x + 10, center_y - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.6, 
                   cv::Scalar(0, 255, 255),  // 黄色
                   2);
        
        // 添加检测状态标注
        std::string status_text = detected ? "Detected" : "Not Detected";
        cv::Scalar status_color = detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        
        cv::putText(result_image, 
                   status_text, 
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, 
                   status_color,
                   2);
                   
        cv::putText(contour_image, 
                   status_text, 
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.8, 
                   status_color,
                   2);
        
        // 如果检测到，添加面积信息
        if (detected) {
            std::string area_text = "Area: " + std::to_string(static_cast<int>(best_area));
            cv::putText(result_image, 
                       area_text, 
                       cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.6, 
                       cv::Scalar(255, 255, 255),  // 白色
                       2);
                       
            cv::putText(contour_image, 
                       area_text, 
                       cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.6, 
                       cv::Scalar(255, 255, 255),  // 白色
                       2);
        }
        
        // 添加帧率信息
        std::string fps_text = "FPS: " + std::to_string(1000 / std::max(1L, duration.count()));
        cv::putText(result_image,
                   fps_text,
                   cv::Point(result_image.cols - 120, 30),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.6,
                   cv::Scalar(255, 255, 255),  // 白色
                   2);
                   
        cv::putText(contour_image,
                   fps_text,
                   cv::Point(contour_image.cols - 120, 30),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.6,
                   cv::Scalar(255, 255, 255),  // 白色
                   2);
        
        // 发布所有图像
        if (image_transport_initialized_) {
            // 确保所有图像都是有效的
            if (!raw_image.empty()) {
                publishImage(raw_image_pub_, raw_image, msg->header.stamp);
            }
            if (!result_image.empty()) {
                publishImage(result_image_pub_, result_image, msg->header.stamp);
            }
            // 新增：发布二值化图像和轮廓图像
            if (!binary_image.empty()) {
                publishBinaryImage(binary_image, msg->header.stamp);
            }
            if (!contour_image.empty()) {
                publishImage(contour_image_pub_, contour_image, msg->header.stamp);
            }
        }
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV异常: %s", e.what());
        RCLCPP_ERROR(get_logger(), "OpenCV异常详情: file=%s, line=%d, func=%s", 
                    e.file.c_str(), e.line, e.func.c_str());
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "异常: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "未知错误发生在图像处理回调中");
    }
}

void DartDetectorNode::publishImage(image_transport::Publisher& pub, const cv::Mat& image, rclcpp::Time stamp) {
    if (image.empty() || !pub.getNumSubscribers()) {
        return;
    }
    
    try {
        // 检查图像是否有效
        if (image.data == nullptr || image.cols <= 0 || image.rows <= 0) {
            RCLCPP_WARN(get_logger(), "尝试发布无效图像，跳过");
            return;
        }
        
        // 克隆图像确保数据独立和连续
        cv::Mat img_to_publish = image.clone();
        
        // 检查克隆后的图像
        if (img_to_publish.empty()) {
            RCLCPP_WARN(get_logger(), "克隆后图像为空，跳过发布");
            return;
        }
        
        // 确保图像是3通道
        if (img_to_publish.channels() != 3) {
            RCLCPP_WARN(get_logger(), "发布图像不是3通道，实际通道数: %d", img_to_publish.channels());
            // 尝试转换为3通道
            if (img_to_publish.channels() == 1) {
                cv::cvtColor(img_to_publish, img_to_publish, cv::COLOR_GRAY2BGR);
            } else {
                return;
            }
        }
        
        // 确保数据类型正确
        if (img_to_publish.depth() != CV_8U) {
            img_to_publish.convertTo(img_to_publish, CV_8U);
        }
        
        // 创建cv_bridge消息
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = stamp;
        cv_image.header.frame_id = "camera";
        cv_image.encoding = "bgr8";
        cv_image.image = img_to_publish;
        
        // 转换为ROS消息并发布
        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
        pub.publish(*msg);
        
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时OpenCV异常: %s", e.what());
        RCLCPP_ERROR(get_logger(), "异常详情: file=%s, line=%d", e.file.c_str(), e.line);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "发布图像时异常: %s", e.what());
    }
}

void DartDetectorNode::publishBinaryImage(const cv::Mat& binary_image, rclcpp::Time stamp) {
    if (binary_image.empty() || !binary_image_pub_.getNumSubscribers()) {
        return;
    }
    
    try {
        // 检查图像是否有效
        if (binary_image.data == nullptr || binary_image.cols <= 0 || binary_image.rows <= 0) {
            RCLCPP_WARN(get_logger(), "尝试发布无效的二值化图像，跳过");
            return;
        }
        
        // 克隆图像确保数据独立和连续
        cv::Mat img_to_publish = binary_image.clone();
        
        // 检查克隆后的图像
        if (img_to_publish.empty()) {
            RCLCPP_WARN(get_logger(), "克隆后二值化图像为空，跳过发布");
            return;
        }
        
        // 确保二值化图像是单通道
        if (img_to_publish.channels() != 1) {
            // 如果不是单通道，尝试转换
            if (img_to_publish.channels() == 3) {
                cv::cvtColor(img_to_publish, img_to_publish, cv::COLOR_BGR2GRAY);
            } else {
                RCLCPP_WARN(get_logger(), "二值化图像通道数: %d，无法处理", img_to_publish.channels());
                return;
            }
        }
        
        // 确保数据类型正确
        if (img_to_publish.depth() != CV_8U) {
            img_to_publish.convertTo(img_to_publish, CV_8U);
        }
        
        // 创建cv_bridge消息 - 二值化图像使用mono8编码
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = stamp;
        cv_image.header.frame_id = "camera";
        cv_image.encoding = "mono8";
        cv_image.image = img_to_publish;
        
        // 转换为ROS消息并发布
        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
        binary_image_pub_.publish(*msg);
        
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
RCLCPP_COMPONENTS_REGISTER_NODE(pka::DartDetectorNode)