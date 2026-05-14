#include "dart_solver/solver_node.hpp"

#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>

namespace pka {

SolverNode::SolverNode(const rclcpp::NodeOptions& options)
    : Node("dart_solver_node", options),
      filter_enabled_(false),
      filter_type_("one_euro"),
      one_euro_freq_(0.0),
      one_euro_min_cutoff_(0.0),
      one_euro_beta_(0.0),
      one_euro_d_cutoff_(0.0),
      ekf_process_noise_x_(0.0),
      ekf_process_noise_y_(0.0),
      ekf_measurement_noise_x_(0.0),
      ekf_measurement_noise_y_(0.0),
      ekf_initial_covariance_(0.0),
      yaw_offset_deg_(0.0),
      publish_debug_topics_(true),
      debug_log_enabled_(false),
      fire_prediction_enabled_(true),
      fire_stop_delay_s_(0.08),
      fire_max_yaw_rate_deg_s_(3.0),
      fire_yaw_rate_lpf_alpha_(0.35),
      fire_zero_yaw_when_advice_(true),
      frame_count_(0),
      fps_(0.0),
      camera_info_received_(false),
      has_last_valid_yaw_(false),
      last_valid_yaw_(0.0),
      has_last_yaw_sample_(false),
      last_yaw_sample_(0.0),
      has_yaw_rate_(false),
      yaw_rate_deg_s_(0.0),
      last_fire_advice_(255),
      last_valid_measurement_(false) {
    try {
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
    solver_params_.image_width = this->declare_parameter("image.width", 800.0);
    solver_params_.image_height = this->declare_parameter("image.height", 600.0);

    filter_enabled_ = this->declare_parameter("filter.enable", true);
    filter_type_ = this->declare_parameter("filter.type", std::string("one_euro"));

    one_euro_freq_ = this->declare_parameter("one_euro.freq", 30.0);
    one_euro_min_cutoff_ = this->declare_parameter("one_euro.min_cutoff", 1.0);
    one_euro_beta_ = this->declare_parameter("one_euro.beta", 0.5);
    one_euro_d_cutoff_ = this->declare_parameter("one_euro.d_cutoff", 1.0);

    ekf_process_noise_x_ = this->declare_parameter("ekf.process_noise_x", 8.0);
    ekf_process_noise_y_ = this->declare_parameter("ekf.process_noise_y", 8.0);
    ekf_measurement_noise_x_ = this->declare_parameter("ekf.measurement_noise_x", 12.0);
    ekf_measurement_noise_y_ = this->declare_parameter("ekf.measurement_noise_y", 12.0);
    ekf_initial_covariance_ = this->declare_parameter("ekf.initial_covariance", 500.0);

    // ===== yaw 零位补偿 =====
    // 用于补偿相机光轴、镖架发射轴、电控零位之间的固定角度偏差。
    // 最终控制使用：yaw = yaw_raw + calib.yaw_offset_deg
    // 如果视觉 yaw=0 但实际还差 +0.9°，可以先尝试 +0.9 或 -0.9，按实际方向调整。
    yaw_offset_deg_ = this->declare_parameter("calib.yaw_offset_deg", 0.0);

    // 发射判断阈值：你现在要压到 0.1 度内
    solver_params_.yaw_threshold = this->declare_parameter("fire.yaw_threshold", 0.1);

    // ===== 新增：发射预测补偿参数 =====
    fire_prediction_enabled_ = this->declare_parameter("fire.prediction_enable", true);

    // 预测提前量，单位秒。
    // 这个值代表：视觉延迟 + 串口延迟 + 电控响应 + 机械惯性造成的等效滞后。
    fire_stop_delay_s_ = this->declare_parameter("fire.stop_delay_s", 0.08);

    // 发射时允许的最大 yaw 速度。
    // 太快时即使预测会经过中心，也先不给 fire，避免高速穿越造成更大过冲。
    fire_max_yaw_rate_deg_s_ = this->declare_parameter("fire.max_yaw_rate_deg_s", 3.0);

    // yaw 速度低通系数，0~1。
    // 越大越跟手，越小越平滑。
    fire_yaw_rate_lpf_alpha_ = this->declare_parameter("fire.yaw_rate_lpf_alpha", 0.35);
    if (fire_yaw_rate_lpf_alpha_ < 0.0) {
        fire_yaw_rate_lpf_alpha_ = 0.0;
    }
    if (fire_yaw_rate_lpf_alpha_ > 1.0) {
        fire_yaw_rate_lpf_alpha_ = 1.0;
    }

    // fire_advice=1 时，是否把下发给电控的 yaw 置 0。
    // 这样可以避免“允许发射了，但 yaw 字段还在让电控继续修正”。
    fire_zero_yaw_when_advice_ = this->declare_parameter("fire.zero_yaw_when_advice", true);

    publish_debug_topics_ = this->declare_parameter("debug.publish_topics", true);
    debug_log_enabled_ = this->declare_parameter("debug.enable_log", false);

    solver_method_ = std::make_unique<SolverMethod>(solver_params_);

    RCLCPP_INFO(get_logger(), "===== solver 参数 =====");
    RCLCPP_INFO(get_logger(), "Image size: %.0fx%.0f",
                solver_params_.image_width, solver_params_.image_height);
    RCLCPP_INFO(get_logger(), "Yaw threshold: %.3f deg", solver_params_.yaw_threshold);
    RCLCPP_INFO(get_logger(), "Yaw offset: %.3f deg", yaw_offset_deg_);
    RCLCPP_INFO(get_logger(), "Filter: enable=%s  type=%s",
                filter_enabled_ ? "true" : "false", filter_type_.c_str());

    if (filter_type_ == "one_euro") {
        RCLCPP_INFO(get_logger(),
                    "OneEuro: freq=%.1f  min_cutoff=%.2f  beta=%.3f  d_cutoff=%.2f",
                    one_euro_freq_, one_euro_min_cutoff_, one_euro_beta_, one_euro_d_cutoff_);
    } else if (filter_type_ == "ekf") {
        RCLCPP_INFO(get_logger(),
                    "EKF: q_x=%.2f q_y=%.2f  r_x=%.2f r_y=%.2f  P0=%.1f",
                    ekf_process_noise_x_, ekf_process_noise_y_,
                    ekf_measurement_noise_x_, ekf_measurement_noise_y_,
                    ekf_initial_covariance_);
    }

    RCLCPP_INFO(get_logger(),
                "Fire prediction: enable=%s  stop_delay=%.3fs  max_rate=%.2f deg/s  rate_alpha=%.2f  zero_yaw_when_fire=%s",
                fire_prediction_enabled_ ? "true" : "false",
                fire_stop_delay_s_,
                fire_max_yaw_rate_deg_s_,
                fire_yaw_rate_lpf_alpha_,
                fire_zero_yaw_when_advice_ ? "true" : "false");

    RCLCPP_INFO(get_logger(), "Debug: publish_topics=%s  enable_log=%s",
                publish_debug_topics_ ? "true" : "false",
                debug_log_enabled_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "=======================");
}

void SolverNode::initFilters() {
    x_one_euro_filter_ = std::make_unique<OneEuroFilter>(
        one_euro_freq_, one_euro_min_cutoff_, one_euro_beta_, one_euro_d_cutoff_);

    y_one_euro_filter_ = std::make_unique<OneEuroFilter>(
        one_euro_freq_, one_euro_min_cutoff_, one_euro_beta_, one_euro_d_cutoff_);

    ekf_filter_ = std::make_unique<Ekf2DFilter>(
        ekf_process_noise_x_,
        ekf_process_noise_y_,
        ekf_measurement_noise_x_,
        ekf_measurement_noise_y_,
        ekf_initial_covariance_);
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

    yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "yaw",
        qos_reliable);

    filtered_x_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "filtered_x",
        qos_reliable);

    filtered_y_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "filtered_y",
        qos_reliable);

    RCLCPP_INFO(get_logger(),
                "Publishing to topics: serial_send_data, fire_state, yaw, filtered_x, filtered_y");
}

void SolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
        solver_params_.fx = msg->k[0];
        solver_params_.fy = msg->k[4];
        solver_params_.cx = msg->k[2];
        solver_params_.cy = msg->k[5];

        // 只有 camera_info 主点有效时，yaw 解算中心才使用 cx。
        // 否则自动回退到 image.width / 2.0，避免错误 camera_info 造成中心错位。
        solver_params_.use_camera_center =
            std::isfinite(solver_params_.cx) &&
            std::isfinite(solver_params_.cy) &&
            solver_params_.cx > 0.0 &&
            solver_params_.cy > 0.0;

        solver_method_->updateParameters(solver_params_);
        camera_info_received_ = true;

        RCLCPP_INFO(get_logger(),
                    "Received camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, yaw_center=%.2f (%s)",
                    solver_params_.fx,
                    solver_params_.fy,
                    solver_params_.cx,
                    solver_params_.cy,
                    solver_params_.use_camera_center ? solver_params_.cx : solver_params_.image_width / 2.0,
                    solver_params_.use_camera_center ? "camera_info" : "image_center");
    }
}

void SolverNode::lightCallback(const dart_interfaces::msg::Light::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_DEBUG(get_logger(), "Waiting for camera info...");
        return;
    }

    rclcpp::Time current_time = this->now();

    if (frame_count_ == 0) {
        last_time_ = current_time;
    } else {
        double dt = (current_time - last_time_).seconds();
        if (dt > 1e-6) {
            fps_ = 0.9 * fps_ + 0.1 / dt;
        }
        last_time_ = current_time;
    }
    frame_count_++;

    double x_processed = msg->x;
    double y_processed = msg->y;

    const double t = current_time.seconds();

    const bool invalid_by_negative = (msg->x < 0.0 || msg->y < 0.0);
    const bool invalid_by_zero = (msg->x == 0.0 && msg->y == 0.0);
    const bool invalid_by_range =
        (msg->x >= solver_params_.image_width || msg->y >= solver_params_.image_height);

    const bool valid_measurement =
        !(invalid_by_negative || invalid_by_zero || invalid_by_range);

    double yaw = 0.0;
    double yaw_pred = 0.0;
    double serial_yaw = 0.0;
    uint8_t fire_advice = 0;

    // 测量有效性状态变化
    if (valid_measurement != last_valid_measurement_) {
        if (valid_measurement) {
            RCLCPP_INFO(get_logger(),
                "[solver] 目标重新出现: raw=(%.2f, %.2f)  frame=%zu",
                msg->x, msg->y, frame_count_);
        } else {
            RCLCPP_INFO(get_logger(),
                "[solver] 目标丢失 (raw=(%.2f, %.2f))  frame=%zu",
                msg->x, msg->y, frame_count_);
        }
        last_valid_measurement_ = valid_measurement;
    }

    if (valid_measurement) {
        if (filter_enabled_) {
            if (filter_type_ == "one_euro") {
                x_processed = x_one_euro_filter_->filter(msg->x, t);
                y_processed = y_one_euro_filter_->filter(msg->y, t);
            } else if (filter_type_ == "ekf") {
                const auto filtered_state = ekf_filter_->filter(msg->x, msg->y);
                x_processed = filtered_state[0];
                y_processed = filtered_state[1];
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Unknown filter.type=%s, use raw measurement", filter_type_.c_str());
            }
        }

        const double yaw_raw = solver_method_->calculateYawAngle(x_processed);
        yaw = yaw_raw + yaw_offset_deg_;
        yaw_pred = yaw;

        // ===== yaw 速度估计 =====
        if (has_last_yaw_sample_) {
            const double dt = (current_time - last_yaw_sample_time_).seconds();

            // dt 太小或太大都不用于速度估计：
            // 太小容易被数值噪声放大，太大说明中间可能丢帧或丢目标。
            if (dt > 1e-4 && dt < 1.0) {
                const double measured_yaw_rate = (yaw - last_yaw_sample_) / dt;

                if (has_yaw_rate_) {
                    yaw_rate_deg_s_ =
                        fire_yaw_rate_lpf_alpha_ * measured_yaw_rate +
                        (1.0 - fire_yaw_rate_lpf_alpha_) * yaw_rate_deg_s_;
                } else {
                    yaw_rate_deg_s_ = measured_yaw_rate;
                    has_yaw_rate_ = true;
                }
            }
        }

        last_yaw_sample_ = yaw;
        last_yaw_sample_time_ = current_time;
        has_last_yaw_sample_ = true;

        // ===== 发射预测补偿 =====
        if (fire_prediction_enabled_ && has_yaw_rate_) {
            yaw_pred = yaw + yaw_rate_deg_s_ * fire_stop_delay_s_;

            const bool moving_towards_center = (yaw * yaw_rate_deg_s_ < 0.0);
            const bool rate_is_safe =
                (std::abs(yaw_rate_deg_s_) <= fire_max_yaw_rate_deg_s_);

            // 情况 1：
            // 当前已经在 0.1 度内，并且速度不大，允许 fire。
            const bool already_center_and_slow =
                (std::abs(yaw) <= solver_params_.yaw_threshold && rate_is_safe);

            // 情况 2：
            // 当前还没到中心，但正在向中心运动；
            // 按当前速度和延迟预测，停点会进入 0.1 度内，允许提前 fire。
            const bool predicted_center =
                moving_towards_center &&
                rate_is_safe &&
                (std::abs(yaw_pred) <= solver_params_.yaw_threshold);

            fire_advice = (already_center_and_slow || predicted_center) ? 1 : 0;
        } else {
            // 关闭预测补偿时，退回原始逻辑
            fire_advice = solver_method_->determineFireAdvice(yaw);
        }

        last_valid_yaw_ = yaw;
        has_last_valid_yaw_ = true;
    } else {
        yaw = has_last_valid_yaw_ ? last_valid_yaw_ : 0.0;
        yaw_pred = yaw;
        fire_advice = 0;

        x_processed = msg->x;
        y_processed = msg->y;

        // 丢灯后不继续用旧 yaw 估计速度
        has_last_yaw_sample_ = false;
        has_yaw_rate_ = false;
        yaw_rate_deg_s_ = 0.0;
    }

    // 开火状态变化
    if (fire_advice != last_fire_advice_) {
        if (fire_advice == 1) {
            RCLCPP_INFO(get_logger(),
                "[solver] 开火许可: yaw=%.3f deg  yaw_pred=%.3f deg  yaw_rate=%.3f deg/s  threshold=%.3f deg  frame=%zu",
                yaw,
                yaw_pred,
                yaw_rate_deg_s_,
                solver_params_.yaw_threshold,
                frame_count_);
        } else {
            RCLCPP_INFO(get_logger(),
                "[solver] 开火禁止: yaw=%.3f deg  yaw_pred=%.3f deg  yaw_rate=%.3f deg/s  frame=%zu",
                yaw,
                yaw_pred,
                yaw_rate_deg_s_,
                frame_count_);
        }
        last_fire_advice_ = fire_advice;
    }

    auto fire_state_msg = std_msgs::msg::Int32();
    fire_state_msg.data = fire_advice;
    fire_state_pub_->publish(fire_state_msg);

    if (publish_debug_topics_) {
        auto yaw_msg = std_msgs::msg::Float32();
        yaw_msg.data = static_cast<float>(yaw);
        yaw_pub_->publish(yaw_msg);

        auto filtered_x_msg = std_msgs::msg::Float32();
        filtered_x_msg.data = static_cast<float>(x_processed);
        filtered_x_pub_->publish(filtered_x_msg);

        auto filtered_y_msg = std_msgs::msg::Float32();
        filtered_y_msg.data = static_cast<float>(y_processed);
        filtered_y_pub_->publish(filtered_y_msg);
    }

    // fire_advice=1 时，可以选择把 yaw 置 0 后再发给电控，
    // 避免视觉已经允许发射，但 yaw 字段仍然让电控继续修正导致过冲。
    serial_yaw = yaw;
    if (fire_advice == 1 && fire_zero_yaw_when_advice_) {
        serial_yaw = 0.0;
    }

    auto serial_msg = dart_interfaces::msg::SerialSendData();
    serial_msg.header.stamp = current_time;
    serial_msg.yaw = static_cast<float>(serial_yaw);
    serial_msg.fire_advice = fire_advice;
    serial_pub_->publish(serial_msg);

    RCLCPP_DEBUG(
        get_logger(),
        "[solver] frame=%zu filter=%s valid=%s "
        "raw=(%.2f,%.2f) filtered=(%.2f,%.2f) "
        "yaw=%.3f yaw_pred=%.3f yaw_rate=%.3f serial_yaw=%.3f fire=%d fps=%.1f",
        frame_count_,
        filter_type_.c_str(),
        valid_measurement ? "Y" : "N",
        msg->x, msg->y,
        x_processed, y_processed,
        yaw,
        yaw_pred,
        yaw_rate_deg_s_,
        serial_yaw,
        fire_advice,
        fps_);

    if (debug_log_enabled_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[solver] FPS=%.1f  frame=%zu  yaw=%.3f deg  yaw_pred=%.3f deg  yaw_rate=%.3f deg/s  fire=%d  filter=%s  valid=%s",
            fps_,
            frame_count_,
            yaw,
            yaw_pred,
            yaw_rate_deg_s_,
            fire_advice,
            filter_type_.c_str(),
            valid_measurement ? "Y" : "N");
    }
}

}  // namespace pka

RCLCPP_COMPONENTS_REGISTER_NODE(pka::SolverNode)