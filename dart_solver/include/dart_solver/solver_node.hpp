#ifndef DART_SOLVER_SOLVER_NODE_HPP_
#define DART_SOLVER_SOLVER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "dart_interfaces/msg/light.hpp"
#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_solver/solver_basic_method.hpp"
#include "dart_solver/filters/solver_ekf_filter.hpp"
#include "dart_solver/filters/solver_one_euro_filter.hpp"

namespace pka {

class SolverNode : public rclcpp::Node {
public:
    explicit SolverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SolverNode() override = default;

private:
    void initParameters();
    void initFilters();
    void initSubscribers();
    void initPublishers();

    void lightCallback(const dart_interfaces::msg::Light::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<dart_interfaces::msg::Light>::SharedPtr light_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<dart_interfaces::msg::SerialSendData>::SharedPtr serial_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fire_state_pub_;

    // 调试发布器
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_y_pub_;

    std::unique_ptr<OneEuroFilter> x_one_euro_filter_;
    std::unique_ptr<OneEuroFilter> y_one_euro_filter_;
    std::unique_ptr<Ekf2DFilter> ekf_filter_;

    std::unique_ptr<SolverMethod> solver_method_;

    SolverParameters solver_params_;

    bool filter_enabled_;
    std::string filter_type_;

    double one_euro_freq_;
    double one_euro_min_cutoff_;
    double one_euro_beta_;
    double one_euro_d_cutoff_;

    double ekf_process_noise_x_;
    double ekf_process_noise_y_;
    double ekf_measurement_noise_x_;
    double ekf_measurement_noise_y_;
    double ekf_initial_covariance_;

    bool publish_debug_topics_;
    bool debug_log_enabled_;

    // ===== 发射预测补偿参数 =====
    bool fire_prediction_enabled_;
    double fire_stop_delay_s_;
    double fire_max_yaw_rate_deg_s_;
    double fire_yaw_rate_lpf_alpha_;
    bool fire_zero_yaw_when_advice_;

    size_t frame_count_;
    double fps_;
    rclcpp::Time last_time_;
    bool camera_info_received_;
    bool has_last_valid_yaw_;
    double last_valid_yaw_;

    // ===== yaw 速度估计状态 =====
    bool has_last_yaw_sample_;
    double last_yaw_sample_;
    rclcpp::Time last_yaw_sample_time_;
    bool has_yaw_rate_;
    double yaw_rate_deg_s_;

    // 状态跟踪（调试用）
    uint8_t last_fire_advice_;
    bool last_valid_measurement_;
};

}  // namespace pka

#endif  // DART_SOLVER_SOLVER_NODE_HPP_