#include "dart_serial/uart_node_tool.hpp"

#include <chrono>
#include <sstream>
#include <thread>

#include "dart_serial/uart_node.hpp"

namespace pka
{

bool UARTNodeTool::configure_and_open(UARTNode * node)
{
  try {
    if (!node->serial_) {
      node->serial_ = std::make_unique<serial::Serial>();
    }

    if (node->serial_->isOpen()) {
      node->serial_->close();
    }

    serial::Timeout timeout(0, 10, 0, 10, 0);

    node->serial_->setPort(node->port_name_);
    node->serial_->setBaudrate(static_cast<uint32_t>(node->baudrate_));
    node->serial_->setTimeout(timeout);
    node->serial_->setBytesize(serial::eightbits);
    node->serial_->setParity(serial::parity_none);
    node->serial_->setStopbits(serial::stopbits_one);
    node->serial_->setFlowcontrol(serial::flowcontrol_none);
    node->serial_->open();
    node->serial_->flushInput();
    node->serial_->flushOutput();

    node->is_healthy_ = node->serial_->isOpen();
    node->consecutive_failure_count_ = 0;
    node->last_error_message_.clear();
    node->last_successful_operation_time_ = node->now();

    RCLCPP_INFO(node->get_logger(), "串口打开成功: %s", node->port_name_.c_str());
    return true;
  } catch (const std::exception & e) {
    node->is_healthy_ = false;
    node->consecutive_failure_count_++;
    node->last_error_message_ = e.what();
    RCLCPP_ERROR(node->get_logger(), "串口打开失败: %s", e.what());
    return false;
  }
}

bool UARTNodeTool::send_packet(UARTNode * node, const std::vector<uint8_t> & packet)
{
  if (!node->serial_ || !node->serial_->isOpen()) {
    node->is_healthy_ = false;
    node->consecutive_failure_count_++;
    node->last_error_message_ = "serial port not open";
    RCLCPP_ERROR(node->get_logger(), "串口未打开，无法发送");
    return false;
  }

  try {
    const size_t bytes_written = node->serial_->write(packet);
    if (bytes_written != packet.size()) {
      node->is_healthy_ = false;
      node->consecutive_failure_count_++;
      node->last_error_message_ = "partial write";
      RCLCPP_WARN(
        node->get_logger(),
        "串口发送不完整: expected=%zu actual=%zu",
        packet.size(),
        bytes_written);
      return false;
    }

    if (node->enable_send_data_print_) {
      RCLCPP_INFO(
        node->get_logger(), "串口发送: %s",
        SerialProtocol::format_hex(packet).c_str());
    }

    node->is_healthy_ = true;
    node->consecutive_failure_count_ = 0;
    node->last_error_message_.clear();
    node->last_successful_operation_time_ = node->now();
    return true;
  } catch (const std::exception & e) {
    node->is_healthy_ = false;
    node->consecutive_failure_count_++;
    node->last_error_message_ = e.what();
    RCLCPP_ERROR(node->get_logger(), "串口发送异常: %s", e.what());
    return false;
  }
}

bool UARTNodeTool::restart_serial(UARTNode * node)
{
  if (!node->enable_auto_restart_) {
    return false;
  }
  if (node->restart_in_progress_.exchange(true)) {
    return false;
  }

  const auto now = node->now();
  const double elapsed = (now - node->last_restart_time_).seconds();
  if (elapsed < node->restart_cooldown_) {
    node->restart_in_progress_.store(false);
    RCLCPP_WARN(
      node->get_logger(),
      "串口处于重启冷却期，剩余 %.2fs",
      node->restart_cooldown_ - elapsed);
    return false;
  }

  if (!node->enable_unlimited_restart_ &&
    node->total_restart_attempts_ >= node->max_restart_attempts_)
  {
    node->restart_in_progress_.store(false);
    RCLCPP_ERROR(node->get_logger(), "已达到最大重启次数 %d", node->max_restart_attempts_);
    return false;
  }

  node->last_restart_time_ = now;
  node->total_restart_attempts_++;

  RCLCPP_WARN(
    node->get_logger(),
    "开始重启串口，第 %d 次，延迟 %dms",
    node->total_restart_attempts_,
    node->restart_delay_);

  std::this_thread::sleep_for(std::chrono::milliseconds(node->restart_delay_));
  const bool ok = configure_and_open(node);
  node->restart_in_progress_.store(false);
  return ok;
}

std::string UARTNodeTool::serial_config_summary(const UARTNode * node)
{
  std::ostringstream ss;
  ss << node->port_name_ << " @" << node->baudrate_
     << " TX-only 16B  send_hz=" << node->send_frequency_
     << " mode=" << node->serial_mode_
     << " sub=" << node->serial_send_topic_;
  return ss.str();
}

}  // namespace pka
