#include "dart_serial/uart_node.hpp"

#include <cmath>

#include <chrono>

#include <rclcpp_components/register_node_macro.hpp>

#include "dart_serial/protocol/dart_protocol.hpp"
#include "dart_serial/uart_node_tool.hpp"

namespace pka
{

UARTNode::UARTNode(const rclcpp::NodeOptions & options)
: Node("dart_serial", options),
  port_name_("/dev/ttyACM0"),
  baudrate_(115200),
  timestamp_offset_(0.0),
  enable_send_data_print_(false),
  debug_(false),
  serial_mode_(0),
  virtual_send_frequency_(100.0),
  virtual_yaw_(0.0f),
  virtual_fire_advice_(false),
  send_frequency_(100.0),
  serial_send_topic_("serial_send_data"),
  max_failure_count_(10),
  health_check_interval_(1.0),
  max_restart_attempts_(0),
  restart_cooldown_(2.0),
  enable_auto_restart_(true),
  enable_unlimited_restart_(true),
  restart_delay_(1000),
  consecutive_failure_count_(0),
  total_restart_attempts_(0),
  is_healthy_(false),
  restart_in_progress_(false),
  last_successful_operation_time_(this->now()),
  last_restart_time_(this->now() - rclcpp::Duration::from_seconds(60.0))
{
  try {
    protocol_ = std::make_unique<DartProtocol>();
    init_parameters();
    init_interfaces();
    init_timers();
    heartbeat_pub_ = HeartBeatPublisher::create(this);

    if (serial_mode_ != 0) {
      init_serial();
    } else {
      RCLCPP_INFO(get_logger(), "串口模式 0：关闭串口，不发送");
    }

    RCLCPP_INFO(get_logger(), "串口配置: %s", UARTNodeTool::serial_config_summary(this).c_str());
    RCLCPP_INFO(
      get_logger(),
      "自动重启: enable=%s unlimited=%s max_failures=%d max_restarts=%d cooldown=%.1fs delay=%dms",
      enable_auto_restart_ ? "true" : "false",
      enable_unlimited_restart_ ? "true" : "false",
      max_failure_count_,
      max_restart_attempts_,
      restart_cooldown_,
      restart_delay_);
    RCLCPP_INFO(
      get_logger(),
      "飞镖下行帧: %zu B（仅发送，不读串口）",
      protocol_->send_packet_size());
    RCLCPP_INFO(get_logger(), "dart_serial 节点初始化完成");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "dart_serial 初始化失败: %s", e.what());
    throw;
  }
}

UARTNode::~UARTNode()
{
  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
}

const SerialProtocol & UARTNode::protocol() const
{
  return *protocol_;
}

rclcpp::Time UARTNode::adjusted_stamp() const
{
  return now() - rclcpp::Duration::from_seconds(std::abs(timestamp_offset_));
}

void UARTNode::init_parameters()
{
  serial_mode_ = declare_parameter("serial_mode", 1);

  port_name_ = declare_parameter("port_name", std::string("/dev/ttyACM0"));
  baudrate_ = declare_parameter("baudrate", 115200);
  send_frequency_ = declare_parameter("send_frequency", 100.0);
  serial_send_topic_ = declare_parameter("serial_send_topic", std::string("serial_send_data"));

  timestamp_offset_ = declare_parameter("timestamp_offset", 0.0);
  debug_ = declare_parameter("debug", false);
  enable_send_data_print_ = declare_parameter("enable_send_data_print", false);
  if (debug_) {
    enable_send_data_print_ = true;
  }

  {
    const double default_vs = declare_parameter("virtual_send_frequency", 100.0);
    virtual_send_frequency_ = declare_parameter("virtual_serial_frequency", default_vs);
  }
  virtual_yaw_ = static_cast<float>(declare_parameter("virtual_yaw", 0.0));
  virtual_fire_advice_ = declare_parameter("virtual_fire_advice", false);

  enable_auto_restart_ = declare_parameter("enable_auto_restart", true);
  max_failure_count_ = declare_parameter("max_failure_count", 10);
  health_check_interval_ = declare_parameter("health_check_interval", 1.0);
  max_restart_attempts_ = declare_parameter("max_restart_attempts", 0);
  restart_cooldown_ = declare_parameter("restart_cooldown", 2.0);
  restart_delay_ = declare_parameter("restart_delay", 1000);

  enable_unlimited_restart_ = (max_restart_attempts_ == 0);

  if (send_frequency_ <= 0.0) {
    send_frequency_ = 100.0;
  }
  if (virtual_send_frequency_ <= 0.0) {
    virtual_send_frequency_ = 100.0;
  }
  if (max_failure_count_ <= 0) {
    max_failure_count_ = 10;
  }
  if (health_check_interval_ <= 0.0) {
    health_check_interval_ = 1.0;
  }
  if (restart_delay_ < 0) {
    restart_delay_ = 1000;
  }
  if (restart_cooldown_ < 0.0) {
    restart_cooldown_ = 2.0;
  }

  RCLCPP_INFO(
    get_logger(),
    "串口模式: %d（0=关闭 1=订阅 %s 定时发 2=虚拟 YAML 定时发）",
    serial_mode_,
    serial_send_topic_.c_str());
}

void UARTNode::init_interfaces()
{
  if (serial_mode_ == 1) {
    send_sub_ = create_subscription<dart_interfaces::msg::SerialSendData>(
      serial_send_topic_,
      10,
      std::bind(&UARTNode::send_data_callback, this, std::placeholders::_1));
  }
}

void UARTNode::init_timers()
{
  if (serial_mode_ == 1) {
    const auto send_period = std::chrono::duration<double>(1.0 / send_frequency_);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(send_period),
      std::bind(&UARTNode::send_timer_callback, this));
  }

  if (serial_mode_ == 2) {
    const auto virtual_period = std::chrono::duration<double>(1.0 / virtual_send_frequency_);
    virtual_serial_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(virtual_period),
      std::bind(&UARTNode::virtual_serial_timer_callback, this));
  }

  const auto health_period = std::chrono::duration<double>(health_check_interval_);
  serial_health_check_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(health_period),
    std::bind(&UARTNode::serial_health_check_timer_callback, this));
}

void UARTNode::init_serial()
{
  if (!UARTNodeTool::configure_and_open(this)) {
    RCLCPP_ERROR(get_logger(), "串口初始化失败: %s", last_error_message_.c_str());
  }
}

void UARTNode::send_data_callback(const dart_interfaces::msg::SerialSendData::SharedPtr msg)
{
  if (serial_mode_ != 1) {
    return;
  }
  std::lock_guard<std::mutex> lock(pending_send_mutex_);
  pending_send_ = *msg;
}

void UARTNode::send_timer_callback()
{
  if (serial_mode_ != 1) {
    return;
  }

  dart_interfaces::msg::SerialSendData msg_copy;
  {
    std::lock_guard<std::mutex> lock(pending_send_mutex_);
    msg_copy = pending_send_;
  }

  msg_copy.header.stamp = adjusted_stamp();
  const auto packet = protocol_->pack_send_data(msg_copy);
  const bool ok = UARTNodeTool::send_packet(this, packet);
  if (ok) {
    RCLCPP_DEBUG(
      get_logger(),
      "定时串口发送: yaw=%.3f fire=%u",
      msg_copy.yaw,
      static_cast<unsigned>(msg_copy.fire_advice));
  }
}

void UARTNode::virtual_serial_timer_callback()
{
  if (serial_mode_ != 2) {
    return;
  }

  dart_interfaces::msg::SerialSendData msg;
  msg.header.stamp = adjusted_stamp();
  msg.yaw = virtual_yaw_;
  msg.fire_advice = virtual_fire_advice_ ? 1 : 0;

  const auto packet = protocol_->pack_send_data(msg);
  const bool ok = UARTNodeTool::send_packet(this, packet);
  if (ok) {
    RCLCPP_DEBUG(
      get_logger(),
      "虚拟串口发送: yaw=%.3f fire=%u",
      msg.yaw,
      static_cast<unsigned>(msg.fire_advice));
  }
}

void UARTNode::serial_health_check_timer_callback()
{
  if (serial_mode_ == 0 || !enable_auto_restart_) {
    return;
  }

  const bool port_open = serial_ && serial_->isOpen();
  const double idle_seconds = (now() - last_successful_operation_time_).seconds();
  const bool too_many_failures = consecutive_failure_count_ >= max_failure_count_;
  const bool too_long_idle = idle_seconds > health_check_interval_ * 2.0;

  if (port_open && is_healthy_ && !too_many_failures && !too_long_idle) {
    RCLCPP_DEBUG(get_logger(), "串口健康检查正常");
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "串口异常，准备重启: port_open=%s healthy=%s failures=%d idle=%.2fs last_error=%s",
    port_open ? "true" : "false",
    is_healthy_ ? "true" : "false",
    consecutive_failure_count_,
    idle_seconds,
    last_error_message_.empty() ? "none" : last_error_message_.c_str());
  restart_serial();
}

bool UARTNode::restart_serial()
{
  return UARTNodeTool::restart_serial(this);
}

}  // namespace pka

RCLCPP_COMPONENTS_REGISTER_NODE(pka::UARTNode)
