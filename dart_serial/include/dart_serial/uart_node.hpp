#ifndef DART_SERIAL_UART_NODE_HPP_
#define DART_SERIAL_UART_NODE_HPP_

// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// project
#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_utils/heartbeat.hpp"
#include "dart_serial/uart_driver.hpp"
#include "dart_serial/uart_protocol.hpp"

namespace pka
{

class UARTNode : public rclcpp::Node
{
public:
  explicit UARTNode(const rclcpp::NodeOptions & options);
  ~UARTNode();

private:
  void init_parameters();
  void init_subscriber();
  void init_timer();
  void init_uart();
  void send_data_callback(const dart_interfaces::msg::SerialSendData::SharedPtr msg);
  void virtual_serial_timer_callback();
  void serial_health_check_timer_callback();
  bool restart_serial();
  
  // 辅助函数：格式化字节为十六进制字符串
  std::string format_bytes(const std::vector<uint8_t>& bytes);
  // 发送数据的通用函数
  bool send_serial_data(const std::vector<uint8_t>& buffer);

  std::unique_ptr<UARTDriver> uart_driver_;
  rclcpp::Subscription<dart_interfaces::msg::SerialSendData>::SharedPtr send_sub_;
  rclcpp::TimerBase::SharedPtr virtual_serial_timer_;
  rclcpp::TimerBase::SharedPtr serial_health_check_timer_;

  // 参数
  std::string port_name_;
  int baudrate_;
  double timestamp_offset_;
  bool enable_send_data_print_;
  int serial_mode_;  // 0:关闭, 1:实际串口, 2:虚拟串口
  double virtual_serial_frequency_;
  float virtual_yaw_;
  int virtual_fire_advice_;  // 修改为int类型，以便从yaml正确读取
  double read_frequency_;
  int max_failure_count_;
  double health_check_interval_;
  int max_restart_attempts_;
  double restart_cooldown_;
  bool enable_auto_restart_;
  bool enable_unlimited_restart_;
  int restart_delay_;
  
  // 状态变量
  int consecutive_failure_count_;
  int total_restart_attempts_;
  bool is_healthy_;
  rclcpp::Time last_successful_operation_time_;
  rclcpp::Time last_restart_time_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_pub_;
};

}  // namespace pka

#endif  // DART_SERIAL_UART_NODE_HPP_