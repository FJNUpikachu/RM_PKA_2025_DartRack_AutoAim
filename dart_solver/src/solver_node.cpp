#include "dart_solver/solver_node.hpp"

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
      publish_debug_topics_(true),
      debug_log_enabled_(false),
      frame_count_(0),
      fps_(0.0),
      camera_info_received_(false),
      has_last_valid_yaw_(false),
      last_valid_yaw_(0.0),
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

    solver_params_.yaw_threshold = this->declare_parameter("fire.yaw_threshold", 0.2);
    publish_debug_topics_ = this->declare_parameter("debug.publish_topics", true);
    debug_log_enabled_    = this->declare_parameter("debug.enable_log", false);
    solver_method_ = std::make_unique<SolverMethod>(solver_params_);

    RCLCPP_INFO(get_logger(), "===== solver 参数 =====");
    RCLCPP_INFO(get_logger(), "Image size: %.0fx%.0f",
                solver_params_.image_width, solver_params_.image_height);
    RCLCPP_INFO(get_logger(), "Yaw threshold: %.2f deg", solver_params_.yaw_threshold);
    RCLCPP_INFO(get_logger(), "Filter: enable=%s  type=%s",
                filter_enabled_ ? "true" : "false", filter_type_.c_str());
    if (filter_type_ == "one_euro") {
        RCLCPP_INFO(get_logger(), "OneEuro: freq=%.1f  min_cutoff=%.2f  beta=%.3f  d_cutoff=%.2f",
                    one_euro_freq_, one_euro_min_cutoff_, one_euro_beta_, one_euro_d_cutoff_);
    } else if (filter_type_ == "ekf") {
        RCLCPP_INFO(get_logger(), "EKF: q_x=%.2f q_y=%.2f  r_x=%.2f r_y=%.2f  P0=%.1f",
                    ekf_process_noise_x_, ekf_process_noise_y_,
                    ekf_measurement_noise_x_, ekf_measurement_noise_y_,
                    ekf_initial_covariance_);
    }
    RCLCPP_INFO(get_logger(), "Debug: publish_topics=%s  enable_log=%s",
                publish_debug_topics_ ? "true" : "false",
                debug_log_enabled_    ? "true" : "false");
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

        solver_method_->updateParameters(solver_params_);
        camera_info_received_ = true;

        RCLCPP_INFO(get_logger(), "Received camera info: fx=%.2f, fy=%.2f",
                    solver_params_.fx, solver_params_.fy);
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

        yaw = solver_method_->calculateYawAngle(x_processed);
        fire_advice = solver_method_->determineFireAdvice(yaw);
        last_valid_yaw_ = yaw;
        has_last_valid_yaw_ = true;
    } else {
        yaw = has_last_valid_yaw_ ? last_valid_yaw_ : 0.0;
        fire_advice = 0;
        x_processed = msg->x;
        y_processed = msg->y;
    }

    // 开火状态变化
    if (fire_advice != last_fire_advice_) {
        if (fire_advice == 1) {
            RCLCPP_INFO(get_logger(),
                "[solver] 开火许可: yaw=%.3f deg  threshold=%.3f deg  frame=%zu",
                yaw, solver_params_.yaw_threshold, frame_count_);
        } else {
            RCLCPP_INFO(get_logger(),
                "[solver] 开火禁止: yaw=%.3f deg  frame=%zu",
                yaw, frame_count_);
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

    auto serial_msg = dart_interfaces::msg::SerialSendData();
    serial_msg.header.stamp = current_time;
    serial_msg.yaw = static_cast<float>(yaw);
    serial_msg.fire_advice = fire_advice;
    serial_pub_->publish(serial_msg);

    // 每帧 DEBUG：完整状态
    RCLCPP_DEBUG(
        get_logger(),
        "[solver] frame=%zu filter=%s valid=%s "
        "raw=(%.2f,%.2f) filtered=(%.2f,%.2f) "
        "yaw=%.3f fire=%d fps=%.1f",
        frame_count_,
        filter_type_.c_str(),
        valid_measurement ? "Y" : "N",
        msg->x, msg->y,
        x_processed, y_processed,
        yaw, fire_advice, fps_);

    // 每 5 秒打一条 INFO 摘要（debug.enable_log 时开启）
    if (debug_log_enabled_) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[solver] FPS=%.1f  frame=%zu  yaw=%.3f deg  fire=%d  filter=%s  valid=%s",
            fps_, frame_count_, yaw, fire_advice,
            filter_type_.c_str(),
            valid_measurement ? "Y" : "N");
    }
}

}  // namespace pka

RCLCPP_COMPONENTS_REGISTER_NODE(pka::SolverNode)