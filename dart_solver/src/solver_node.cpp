#include "dart_solver/solver_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace pka {

SolverNode::SolverNode(const rclcpp::NodeOptions& options)
    : Node("dart_solver_node", options),
      frame_count_(0),
      fps_(0.0),
      camera_info_received_(false),
      filter_enabled_(false),
      filter_mode_(1),
      filter_freq_(0.0),
      filter_min_cutoff_(0.0),
      filter_beta_(0.0),
      filter_d_cutoff_(1.0),
      kalman_q_(0.0),
      kalman_r_(0.0),
      kalman_init_p_(0.0),
      has_last_valid_yaw_(false),
      last_valid_yaw_(0.0) {
    try {
        // 初始化顺序：参数 -> 滤波器 -> 发布器 -> 订阅器
        initParameters();
        initFilters();
        initPublishers();
        initSubscribers();

        RCLCPP_INFO(get_logger(), "Dart solver node initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize node: %s", e.what());
        rclcpp::shutdown();
    }
}

void SolverNode::initParameters() {
    // 图像尺寸参数
    solver_params_.image_width = this->declare_parameter("image.width", 800.0);
    solver_params_.image_height = this->declare_parameter("image.height", 600.0);

    // 滤波器参数
    filter_enabled_ = this->declare_parameter("filter.enable", true);
    filter_mode_ = this->declare_parameter("filter.mode", 1);

    // 一欧元滤波参数（按 2025 逻辑）
    filter_freq_ = this->declare_parameter("one_euro.freq", 30.0);
    filter_min_cutoff_ = this->declare_parameter("one_euro.min_cutoff", 1.0);
    filter_beta_ = this->declare_parameter("one_euro.beta", 0.5);
    filter_d_cutoff_ = this->declare_parameter("one_euro.d_cutoff", 1.0);

    // 卡尔曼参数（保留当前包结构）
    kalman_q_ = this->declare_parameter("kalman.q", 8.0);
    kalman_r_ = this->declare_parameter("kalman.r", 12.0);
    kalman_init_p_ = this->declare_parameter("kalman.init_p", 500.0);

    // yaw角度判断阈值
    solver_params_.yaw_threshold = this->declare_parameter("fire.yaw_threshold", 0.2);

    // 初始化解算方法
    solver_method_ = std::make_unique<SolverMethod>(solver_params_);

    RCLCPP_INFO(get_logger(), "Image size: %.0fx%.0f",
                solver_params_.image_width, solver_params_.image_height);
    RCLCPP_INFO(get_logger(), "Yaw threshold: %.2f degrees", solver_params_.yaw_threshold);
    RCLCPP_INFO(get_logger(), "Filter enabled: %s, mode=%d",
                filter_enabled_ ? "true" : "false", filter_mode_);
}

void SolverNode::initFilters() {
    // 一欧元滤波器
    x_filter_ = std::make_unique<OneEuroFilter>(
        filter_freq_, filter_min_cutoff_, filter_beta_, filter_d_cutoff_);
    y_filter_ = std::make_unique<OneEuroFilter>(
        filter_freq_, filter_min_cutoff_, filter_beta_, filter_d_cutoff_);

    // 卡尔曼滤波器
    x_kalman_filter_ = std::make_unique<KalmanFilter1D>(kalman_q_, kalman_r_, kalman_init_p_);
    y_kalman_filter_ = std::make_unique<KalmanFilter1D>(kalman_q_, kalman_r_, kalman_init_p_);
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

    RCLCPP_INFO(get_logger(), "Subscribed to topics: light_position, camera_info");
}

void SolverNode::initPublishers() {
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    serial_pub_ = this->create_publisher<dart_interfaces::msg::SerialSendData>(
        "serial_send_data",
        qos_reliable);

    fire_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "fire_state",
        qos_reliable);

    RCLCPP_INFO(get_logger(), "Publishing to topics: serial_send_data, fire_state");
}

void SolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
        // 从 camera_info 获取相机内参
        solver_params_.fx = msg->k[0];
        solver_params_.fy = msg->k[4];

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
        if (dt > 1e-6) {
            fps_ = 0.9 * fps_ + 0.1 / dt;  // 指数平滑
        }
        last_time_ = current_time;
    }
    frame_count_++;

    // ===== 按 2025 solver 逻辑判断是否检测到目标 =====
    // 2025 传统 detector：未检测到时发布 (0, 0)
    // 2026 神经网络 detector：未检测到时发布 (-1, -1)
    bool detected = true;

    if ((msg->x == 0.0 && msg->y == 0.0) || (msg->x < 0.0 || msg->y < 0.0)) {
        detected = false;
    }

    double x_processed = msg->x;
    double y_processed = msg->y;
    double t = current_time.seconds();

    // 只有检测到目标时才进行滤波
    if (detected && filter_enabled_) {
        if (filter_mode_ == 1) {
            // 一欧元：完全按 2025 逻辑
            x_processed = x_filter_->filter(msg->x, t);
            y_processed = y_filter_->filter(msg->y, t);
        } else if (filter_mode_ == 2) {
            // 卡尔曼：只是保留入口，不影响一欧元主逻辑
            x_processed = x_kalman_filter_->filter(msg->x, t);
            y_processed = y_kalman_filter_->filter(msg->y, t);
        }
    }

    double yaw_angle = 0.0;

    if (detected) {
        // 正常计算 yaw
        yaw_angle = solver_method_->calculateYawAngle(x_processed);

        // 更新最近一次有效 yaw
        last_valid_yaw_ = yaw_angle;
        has_last_valid_yaw_ = true;
    } else {
        // 未检测到目标：保持上一次有效 yaw
        if (has_last_valid_yaw_) {
            yaw_angle = last_valid_yaw_;
        } else {
            // 系统刚启动且还没有任何有效目标时
            yaw_angle = 0.0;
        }
    }

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

    if (detected) {
        RCLCPP_DEBUG(get_logger(),
                    "Detected. Filtered position: (%.2f, %.2f), Yaw: %.2f°, Fire: %d, FPS: %.1f",
                    x_processed, y_processed, yaw_angle, fire_advice, fps_);
    } else {
        RCLCPP_DEBUG(get_logger(),
                    "Not detected. Keep last yaw: %.2f°, Fire: %d, FPS: %.1f",
                    yaw_angle, fire_advice, fps_);
    }
}

}  // namespace pka

RCLCPP_COMPONENTS_REGISTER_NODE(pka::SolverNode)
