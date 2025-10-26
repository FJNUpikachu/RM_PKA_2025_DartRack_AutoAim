#ifndef DART_SOLVER_SOLVER_NODE_HPP_
#define DART_SOLVER_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "dart_interfaces/msg/light.hpp"
#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_interfaces/msg/serial_receive_data.hpp"
#include "dart_interfaces/msg/ballistic_params.hpp"
#include "dart_solver/one_euro_filter.hpp"
#include "dart_solver/solver_method.hpp"
#include "dart_solver/ballistic_model.hpp"

namespace dart_solver {

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
    void serialReceiveCallback(const dart_interfaces::msg::SerialReceiveData::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    void publishBallisticParams();
    
    // 订阅器
    rclcpp::Subscription<dart_interfaces::msg::Light>::SharedPtr light_sub_;
    rclcpp::Subscription<dart_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // 发布器
    rclcpp::Publisher<dart_interfaces::msg::SerialSendData>::SharedPtr serial_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fire_state_pub_;
    rclcpp::Publisher<dart_interfaces::msg::BallisticParams>::SharedPtr ballistic_params_pub_;
    
    // 组件
    std::unique_ptr<OneEuroFilter> x_filter_;
    std::unique_ptr<OneEuroFilter> y_filter_;
    std::unique_ptr<SolverMethod> solver_method_;
    BallisticModel ballistic_model_;
    std::unique_ptr<TFBroadcaster> tf_broadcaster_;
    
    // 参数
    SolverParameters solver_params_;
    BallisticParameters ballistic_params_;
    bool filter_enabled_;
    double filter_freq_;
    double filter_min_cutoff_;
    double filter_beta_;
    
    // 可视化参数
    bool enable_tf_;
    bool enable_rviz_;
    
    // 状态变量
    size_t frame_count_;
    double fps_;
    rclcpp::Time last_time_;
    bool camera_info_received_;
    uint8_t current_mode_;
};

}  // namespace dart_solver

#endif  // DART_SOLVER_SOLVER_NODE_HPP_
