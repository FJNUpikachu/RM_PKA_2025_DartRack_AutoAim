#include "dart_solver/solver_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

namespace dart_solver {

SolverNode::SolverNode(const rclcpp::NodeOptions& options)
    : Node("dart_solver_node", options),
      frame_count_(0), fps_(0.0), camera_info_received_(false), current_mode_(1) {
    try {
        initParameters();
        initFilters();
        initSubscribers();
        initPublishers();
        
        // 初始化TF广播器
        tf_broadcaster_ = std::make_unique<TFBroadcaster>(this);
        
        RCLCPP_INFO(get_logger(), "Dart solver node initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize node: %s", e.what());
        rclcpp::shutdown();
    }
}

void SolverNode::initParameters() {
    // 一欧元滤波器参数
    filter_enabled_ = this->declare_parameter("filter.enable", true);
    filter_freq_ = this->declare_parameter("filter.freq", 30.0);
    filter_min_cutoff_ = this->declare_parameter("filter.min_cutoff", 1.0);
    filter_beta_ = this->declare_parameter("filter.beta", 0.5);
    
    // 发射判断阈值
    solver_params_.x_threshold = this->declare_parameter("fire.x_threshold", 3.0);
    solver_params_.sentinel_y_min = this->declare_parameter("fire.sentinel_y_min", 100.0);
    solver_params_.sentinel_y_max = this->declare_parameter("fire.sentinel_y_max", 500.0);
    solver_params_.base_y_min = this->declare_parameter("fire.base_y_min", 80.0);
    solver_params_.base_y_max = this->declare_parameter("fire.base_y_max", 450.0);
    
    // 发射点位置
    solver_params_.launch_position.x() = this->declare_parameter("launch_position.x", 0.0);
    solver_params_.launch_position.y() = this->declare_parameter("launch_position.y", 0.0);
    solver_params_.launch_position.z() = this->declare_parameter("launch_position.z", 0.5);
    
    // 弹道模型参数
    ballistic_params_.enable = this->declare_parameter("ballistic.enable", true);
    ballistic_params_.gravity = this->declare_parameter("ballistic.gravity", 9.81);
    ballistic_params_.drag_coeff = this->declare_parameter("ballistic.drag_coeff", 0.01);
    ballistic_params_.muzzle_velocity = this->declare_parameter("ballistic.muzzle_velocity", 10.0);
    ballistic_params_.sentinel_distance = this->declare_parameter("ballistic.sentinel_distance", 5.0);
    ballistic_params_.base_distance = this->declare_parameter("ballistic.base_distance", 8.0);
    ballistic_params_.vertical_offset = this->declare_parameter("ballistic.vertical_offset", 1.2);
    
    // 可视化参数
    enable_tf_ = this->declare_parameter("visualization.enable_tf", true);
    enable_rviz_ = this->declare_parameter("visualization.enable_rviz", true);
    
    // 初始化解算方法
    solver_method_ = std::make_unique<SolverMethod>(solver_params_);
    
    // 初始化弹道模型
    ballistic_model_.setParameters(ballistic_params_);
    
    RCLCPP_INFO(get_logger(), "Ballistic parameters: enable=%s, gravity=%.2f, muzzle_velocity=%.2f",
                ballistic_params_.enable ? "true" : "false",
                ballistic_params_.gravity,
                ballistic_params_.muzzle_velocity);
    RCLCPP_INFO(get_logger(), "Filter enabled: %s, freq=%.1fHz",
                filter_enabled_ ? "true" : "false", filter_freq_);
}

void SolverNode::initFilters() {
    x_filter_ = std::make_unique<OneEuroFilter>(filter_freq_, filter_min_cutoff_, filter_beta_);
    y_filter_ = std::make_unique<OneEuroFilter>(filter_freq_, filter_min_cutoff_, filter_beta_);
}

void SolverNode::initSubscribers() {
    light_sub_ = this->create_subscription<dart_interfaces::msg::Light>(
        "light_position", 10,
        std::bind(&SolverNode::lightCallback, this, std::placeholders::_1));
    
    serial_receive_sub_ = this->create_subscription<dart_interfaces::msg::SerialReceiveData>(
        "serial_receive_data", 10,
        std::bind(&SolverNode::serialReceiveCallback, this, std::placeholders::_1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10,
        std::bind(&SolverNode::cameraInfoCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Subscribed to topics: light_position, serial_receive_data, camera_info");
}

void SolverNode::initPublishers() {
    serial_pub_ = this->create_publisher<dart_interfaces::msg::SerialSendData>(
        "serial_send_data", 10);
    
    fire_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "fire_state", 10);
    
    ballistic_params_pub_ = this->create_publisher<dart_interfaces::msg::BallisticParams>(
        "ballistic_params", 10);
    
    RCLCPP_INFO(get_logger(), "Publishing to topics: serial_send_data, fire_state, ballistic_params");
}

void SolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
        // 从camera_info获取相机内参
        solver_params_.fx = msg->k[0];  // fx
        solver_params_.fy = msg->k[4];  // fy
        solver_params_.cx = msg->k[2];  // cx
        solver_params_.cy = msg->k[5];  // cy
        
        solver_method_->updateParameters(solver_params_);
        
        camera_info_received_ = true;
        RCLCPP_INFO(get_logger(), "Received camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    solver_params_.fx, solver_params_.fy, solver_params_.cx, solver_params_.cy);
    }
}

void SolverNode::serialReceiveCallback(const dart_interfaces::msg::SerialReceiveData::SharedPtr msg) {
    // 更新当前模式（1: 前哨站, 2: 基地）
    if (msg->mode == 1 || msg->mode == 2) {
        if (current_mode_ != msg->mode) {
            current_mode_ = msg->mode;
            solver_method_->updateMode(current_mode_);
            RCLCPP_INFO(get_logger(), "Updated mode to: %d", current_mode_);
            
            // 发布更新后的弹道参数
            publishBallisticParams();
        }
    }
}

void SolverNode::publishBallisticParams() {
    auto msg = dart_interfaces::msg::BallisticParams();
    msg.header.stamp = this->now();
    
    // 填充弹道参数
    msg.enable = ballistic_params_.enable;
    msg.gravity = ballistic_params_.gravity;
    msg.drag_coeff = ballistic_params_.drag_coeff;
    msg.muzzle_velocity = ballistic_params_.muzzle_velocity;
    msg.sentinel_distance = ballistic_params_.sentinel_distance;
    msg.base_distance = ballistic_params_.base_distance;
    msg.vertical_offset = ballistic_params_.vertical_offset;
    
    // 当前模式信息
    msg.mode = current_mode_;
    msg.current_distance = (current_mode_ == 1) ? 
                          ballistic_params_.sentinel_distance : 
                          ballistic_params_.base_distance;
    
    ballistic_params_pub_->publish(msg);
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
    
    // 解算pitch和yaw角度
    double pitch, yaw;
    solver_method_->solveAngles(x_processed, y_processed, pitch, yaw);
    
    // 判断是否可发射
    uint8_t fire_advice = solver_method_->determineFireAdvice(x_processed, y_processed);
    
    // 发布发射状态给detector
    auto fire_state_msg = std_msgs::msg::Int32();
    fire_state_msg.data = fire_advice;
    fire_state_pub_->publish(fire_state_msg);
    
    // 构建并发布串口数据消息
    auto serial_msg = dart_interfaces::msg::SerialSendData();
    serial_msg.header.stamp = current_time;
    serial_msg.pitch = static_cast<float>(pitch);
    serial_msg.yaw = static_cast<float>(yaw);
    serial_msg.fire_advice = fire_advice;
    serial_pub_->publish(serial_msg);
    
    // 计算目标点在3D空间中的位置
    Eigen::Vector3d target_point = solver_method_->calculateTargetPoint(x_processed, y_processed);
    
    // 计算弹道轨迹（受总开关控制）
    std::vector<Eigen::Vector3d> trajectory;
    if (ballistic_params_.enable) {
        trajectory = ballistic_model_.calculateTrajectory(
            solver_params_.launch_position, target_point, pitch, yaw, current_mode_);
    }
    
    // 发布TF变换
    tf_broadcaster_->publishTransforms(
        solver_params_.launch_position, target_point, trajectory, enable_tf_, enable_rviz_);
    
    // 定期发布弹道参数（每10帧）
    if (frame_count_ % 10 == 0) {
        publishBallisticParams();
    }
    
    RCLCPP_DEBUG(get_logger(), "Mode: %d, Filtered position: (%.2f, %.2f), Pitch: %.2f, Yaw: %.2f, Fire: %d, FPS: %.1f",
                current_mode_, x_processed, y_processed, pitch, yaw, fire_advice, fps_);
}

}  // namespace dart_solver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dart_solver::SolverNode)
