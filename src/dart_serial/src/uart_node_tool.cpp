#include "dart_serial/uart_node_tool.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <thread>
#include <vector>

#include "dart_serial/uart_node.hpp"

namespace pka
{

namespace
{

bool path_exists(const std::string & path)
{
  std::error_code ec;
  return std::filesystem::exists(path, ec);
}

void collect_paths_from_dir(
  const std::string & dir,
  const std::string & prefix,
  std::vector<std::string> & out)
{
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) {
    return;
  }

  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) {
      return;
    }

    const auto path = entry.path().string();
    const auto name = entry.path().filename().string();
    if (prefix.empty() || name.rfind(prefix, 0) == 0) {
      out.push_back(path);
    }
  }
}

}  // namespace

std::string UARTNodeTool::resolve_serial_port(UARTNode * node)
{
  // 配置端口存在时，优先使用配置端口。
  if (path_exists(node->port_name_)) {
    return node->port_name_;
  }

  if (!node->auto_detect_port_) {
    return node->port_name_;
  }

  std::vector<std::string> candidates;

  // 优先使用固定软链接。建议后续通过 udev 建立 /dev/dart_serial。
  candidates.push_back("/dev/dart_serial");

  // by-id 通常比 ttyACM0/1 稳定；不存在时会自动跳过。
  collect_paths_from_dir("/dev/serial/by-id", "", candidates);

  // STM32 Virtual COM Port 通常是 ttyACM*，部分 USB 转串口可能是 ttyUSB*。
  collect_paths_from_dir("/dev", "ttyACM", candidates);
  collect_paths_from_dir("/dev", "ttyUSB", candidates);

  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

  for (const auto & candidate : candidates) {
    if (path_exists(candidate)) {
      RCLCPP_WARN(
        node->get_logger(),
        "配置串口 %s 不存在，自动切换到可用串口: %s",
        node->port_name_.c_str(),
        candidate.c_str());
      node->port_name_ = candidate;
      return candidate;
    }
  }

  return node->port_name_;
}

void UARTNodeTool::close_serial_quietly(UARTNode * node)
{
  try {
    if (node->serial_ && node->serial_->isOpen()) {
      node->serial_->close();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "关闭串口时出现异常: %s", e.what());
  }
}

void UARTNodeTool::mark_unhealthy_and_close(UARTNode * node, const std::string & error)
{
  node->is_healthy_ = false;
  node->consecutive_failure_count_++;
  node->last_error_message_ = error;
  close_serial_quietly(node);
}


bool UARTNodeTool::configure_and_open(UARTNode * node)
{
  std::lock_guard<std::mutex> lock(node->serial_mutex_);

  try {
    const std::string resolved_port = resolve_serial_port(node);

    if (!node->serial_) {
      node->serial_ = std::make_unique<serial::Serial>();
    }

    close_serial_quietly(node);

    serial::Timeout timeout(0, 10, 0, 10, 0);

    node->serial_->setPort(resolved_port);
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

    RCLCPP_INFO(node->get_logger(), "串口打开成功: %s", resolved_port.c_str());
    return true;
  } catch (const std::exception & e) {
    mark_unhealthy_and_close(node, e.what());
    RCLCPP_ERROR(node->get_logger(), "串口打开失败: %s", e.what());
    return false;
  }
}

bool UARTNodeTool::send_packet(UARTNode * node, const std::vector<uint8_t> & packet)
{
  // 重启过程中暂停发送，避免 write 与 close/open 并发打架。
  if (node->restart_in_progress_.load()) {
    return false;
  }

  // 已知不健康时不再 100Hz 疯狂写坏句柄，等待 health timer 重连。
  if (!node->is_healthy_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(node->serial_mutex_);

  if (!node->serial_ || !node->serial_->isOpen()) {
    node->is_healthy_ = false;
    node->consecutive_failure_count_++;
    node->last_error_message_ = "serial port not open";
    RCLCPP_ERROR_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "串口未打开，无法发送");
    return false;
  }

  try {
    const size_t bytes_written = node->serial_->write(packet);
    if (bytes_written != packet.size()) {
      mark_unhealthy_and_close(node, "partial write");
      RCLCPP_WARN(
        node->get_logger(),
        "串口发送不完整: expected=%zu actual=%zu，已关闭串口等待重连",
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
    mark_unhealthy_and_close(node, e.what());
    RCLCPP_ERROR(
      node->get_logger(),
      "串口发送异常: %s，已关闭串口等待重连",
      e.what());
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

  {
    std::lock_guard<std::mutex> lock(node->serial_mutex_);
    close_serial_quietly(node);
    node->is_healthy_ = false;
  }

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
     << " auto_detect=" << (node->auto_detect_port_ ? "true" : "false")
     << " sub=" << node->serial_send_topic_;
  return ss.str();
}

}  // namespace pka
