#ifndef DART_SOLVER_SOLVER_NODE_HPP_
#define DART_SOLVER_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "dart_interfaces/msg/light.hpp"
#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_solver/one_euro_filter.hpp"
#include "dart_solver/solver_method.hpp"

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
    
    // 订阅器
    rclcpp::Subscription<dart_interfaces::msg::Light>::SharedPtr light_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // 发布器
    rclcpp::Publisher<dart_interfaces::msg::SerialSendData>::SharedPtr serial_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fire_state_pub_;
    
    // 滤波器
    std::unique_ptr<OneEuroFilter> x_filter_;
    std::unique_ptr<OneEuroFilter> y_filter_;
    
    // 解算方法
    std::unique_ptr<SolverMethod> solver_method_;
    
    // 参数
    SolverParameters solver_params_;
    bool filter_enabled_;
    double filter_freq_;
    double filter_min_cutoff_;
    double filter_beta_;
    
    // 状态变量
    size_t frame_count_;
    double fps_;
    rclcpp::Time last_time_;
    bool camera_info_received_;
};

}  // namespace pka

#endif  // DART_SOLVER_SOLVER_NODE_HPP_