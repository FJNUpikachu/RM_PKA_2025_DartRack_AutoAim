#ifndef DART_SERIAL_UART_PROTOCOL_HPP_
#define DART_SERIAL_UART_PROTOCOL_HPP_

#include "dart_interfaces/msg/serial_send_data.hpp"
#include <vector>

namespace pka
{

class UARTProtocol
{
public:
  // 帧头帧尾定义
  static constexpr uint8_t FRAME_HEADER = 0xFF;
  static constexpr uint8_t FRAME_TAIL = 0x0D;
  
  // 发送数据包大小
  static constexpr size_t SEND_PACKET_SIZE = 16;

  // 打包发送数据
  static std::vector<uint8_t> pack_send_data(const dart_interfaces::msg::SerialSendData & msg);

private:
  // 数据类型转换工具
  static void float_to_bytes(float value, uint8_t* bytes);
  static uint8_t calculate_checksum(const uint8_t* data, size_t size);
};

}  // namespace pka

#endif  // DART_SERIAL_UART_PROTOCOL_HPP_