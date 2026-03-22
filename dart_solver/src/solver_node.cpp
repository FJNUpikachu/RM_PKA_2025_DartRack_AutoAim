#include "dart_solver/solver_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

namespace pka {

SolverNode::SolverNode(const rclcpp::NodeOptions& options)
    : Node("dart_solver_node", options),
      frame_count_(0), 
      fps_(0.0), 
      camera_info_received_(false),
      filter_enabled_(false),  // 初始化所有成员变量
      filter_freq_(0.0),
      filter_min_cutoff_(0.0),
      filter_beta_(0.0) {
    try {
        // 初始化顺序：参数 -> 滤波器 -> 发布器 -> 订阅器
        initParameters();
        initFilters();
        initPublishers();   // 先初始化发布器
        initSubscribers();  // 再初始化订阅器
        
        RCLCPP_INFO(get_logger(), "Dart solver node initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize node: %s", e.what());
        rclcpp::shutdown();
    }
}

void SolverNode::initParameters() {
    // 获取图像尺寸参数
    solver_params_.image_width = this->declare_parameter("image.width", 640.0);
    solver_params_.image_height = this->declare_parameter("image.height", 480.0);
    
    // 滤波器参数
    filter_enabled_ = this->declare_parameter("filter.enable", true);
    filter_freq_ = this->declare_parameter("filter.freq", 30.0);
    filter_min_cutoff_ = this->declare_parameter("filter.min_cutoff", 1.0);
    filter_beta_ = this->declare_parameter("filter.beta", 0.5);
    
    // yaw角度判断阈值
    solver_params_.yaw_threshold = this->declare_parameter("fire.yaw_threshold", 1.0);
    
    // 初始化解算方法
    solver_method_ = std::make_unique<SolverMethod>(solver_params_);
    
    RCLCPP_INFO(get_logger(), "Image size: %.0fx%.0f", solver_params_.image_width, solver_params_.image_height);
    RCLCPP_INFO(get_logger(), "Yaw threshold: %.2f degrees", solver_params_.yaw_threshold);
    RCLCPP_INFO(get_logger(), "Filter enabled: %s, freq=%.1fHz",
                filter_enabled_ ? "true" : "false", filter_freq_);
}

void SolverNode::initFilters() {
    x_filter_ = std::make_unique<OneEuroFilter>(filter_freq_, filter_min_cutoff_, filter_beta_);
    y_filter_ = std::make_unique<OneEuroFilter>(filter_freq_, filter_min_cutoff_, filter_beta_);
}

void SolverNode::initSubscribers() {
    
    light_sub_ = this->create_subscription<dart_interfaces::msg::Light>(
        "light_position", 
        10,
        std::bind(&SolverNode::lightCallback, this, std::placeholders::_1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 
        10,
        std::bind(&SolverNode::cameraInfoCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Subscribed to topics: light_position, camera_info (QoS: RELIABLE)");
}

void SolverNode::initPublishers() {
    // 发布器也使用可靠QoS
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    serial_pub_ = this->create_publisher<dart_interfaces::msg::SerialSendData>(
        "serial_send_data", 
        qos_reliable);
    
    fire_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "fire_state", 
        qos_reliable);
    
    RCLCPP_INFO(get_logger(), "Publishing to topics: serial_send_data, fire_state (QoS: RELIABLE)");
}

void SolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
        // 从camera_info获取相机内参
        solver_params_.fx = msg->k[0];  // fx
        solver_params_.fy = msg->k[4];  // fy
        
        solver_method_->updateParameters(solver_params_);
        
        camera_info_received_ = true;
        RCLCPP_INFO(get_logger(), "Received camera info: fx=%.2f, fy=%.2f",
                    solver_params_.fx, solver_params_.fy);
    }
}

void SolverNode::lightCallback(const dart_interfaces::msg::Light::SharedPtr msg) {
    // 等待相机内参
    if (!camera_info_received_) {
        RCLCPP_DEBUG(get_logger(), "Waiting for camera info...");
        return;
    }
    
    // 计算帧率
    rclcpp::Time current_time = this->now();
    if (frame_count_ == 0) {
        last_time_ = current_time;
    } else {
        double dt = (current_time - last_time_).seconds();
        fps_ = 0.9 * fps_ + 0.1 / dt;  // 指数平滑
        last_time_ = current_time;
    }
    frame_count_++;
    
    // 应用一欧元滤波（根据开关控制）
    double x_processed = msg->x;
    double y_processed = msg->y;
    double t = current_time.seconds();
    
    if (filter_enabled_) {
        x_processed = x_filter_->filter(msg->x, t);
        y_processed = y_filter_->filter(msg->y, t);
    }
    
    // 计算yaw角度
    double yaw_angle = solver_method_->calculateYawAngle(x_processed);
    
    // 判断是否可发射
    uint8_t fire_advice = solver_method_->determineFireAdvice(yaw_angle);
    
    // 发布发射状态
    auto fire_state_msg = std_msgs::msg::Int32();
    fire_state_msg.data = fire_advice;
    fire_state_pub_->publish(fire_state_msg);
    
    // 构建并发布串口数据消息
    auto serial_msg = dart_interfaces::msg::SerialSendData();
    serial_msg.header.stamp = current_time;
    serial_msg.yaw = static_cast<float>(yaw_angle);
    serial_msg.fire_advice = fire_advice;
    serial_pub_->publish(serial_msg);
    
    RCLCPP_DEBUG(get_logger(), "Filtered position: (%.2f, %.2f), Yaw: %.2f°, Fire: %d, FPS: %.1f",
                x_processed, y_processed, yaw_angle, fire_advice, fps_);
}

}  // namespace pka

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pka::SolverNode)