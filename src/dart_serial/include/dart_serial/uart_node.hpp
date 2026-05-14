#ifndef DART_SERIAL_UART_NODE_HPP_
#define DART_SERIAL_UART_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_serial/protocol/serial_protocol.hpp"
#include "dart_utils/heartbeat.hpp"
#include "serial/serial.h"

namespace pka
{

class UARTNodeTool;

class UARTNode : public rclcpp::Node
{
public:
  explicit UARTNode(const rclcpp::NodeOptions & options);
  ~UARTNode() override;

  const SerialProtocol & protocol() const;

private:
  friend class UARTNodeTool;

  rclcpp::Time adjusted_stamp() const;

  void init_parameters();
  void init_interfaces();
  void init_timers();
  void init_serial();

  void send_data_callback(const dart_interfaces::msg::SerialSendData::SharedPtr msg);
  void send_timer_callback();
  void virtual_serial_timer_callback();
  void serial_health_check_timer_callback();
  bool restart_serial();

  std::unique_ptr<SerialProtocol> protocol_;
  std::unique_ptr<serial::Serial> serial_;

  rclcpp::Subscription<dart_interfaces::msg::SerialSendData>::SharedPtr send_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr virtual_serial_timer_;
  rclcpp::TimerBase::SharedPtr serial_health_check_timer_;

  std::mutex pending_send_mutex_;
  std::mutex serial_mutex_;
  dart_interfaces::msg::SerialSendData pending_send_;

  std::string port_name_;
  bool auto_detect_port_;
  int baudrate_;
  double timestamp_offset_;
  bool enable_send_data_print_;
  bool debug_;
  int serial_mode_;
  double virtual_send_frequency_;
  float virtual_yaw_;
  bool virtual_fire_advice_;
  double send_frequency_;
  std::string serial_send_topic_;

  int max_failure_count_;
  double health_check_interval_;
  int max_restart_attempts_;
  double restart_cooldown_;
  bool enable_auto_restart_;
  bool enable_unlimited_restart_;
  int restart_delay_;

  int consecutive_failure_count_;
  int total_restart_attempts_;
  bool is_healthy_;
  std::atomic<bool> restart_in_progress_;
  std::string last_error_message_;
  rclcpp::Time last_successful_operation_time_;
  rclcpp::Time last_restart_time_;

  HeartBeatPublisher::SharedPtr heartbeat_pub_;
};

}  // namespace pka

#endif  // DART_SERIAL_UART_NODE_HPP_
