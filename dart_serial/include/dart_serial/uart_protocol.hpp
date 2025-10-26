#ifndef DART_SERIAL_UART_PROTOCOL_HPP_
#define DART_SERIAL_UART_PROTOCOL_HPP_

#include "dart_interfaces/msg/serial_send_data.hpp"
#include "dart_interfaces/msg/serial_receive_data.hpp"
#include <vector>
#include <memory>

namespace dart_serial
{

class UARTProtocol
{
public:
  // 帧头帧尾定义
  static constexpr uint8_t FRAME_HEADER = 0xFF;
  static constexpr uint8_t FRAME_TAIL = 0x0D;
  
  // 数据包大小统一为16字节
  // 发送数据包结构：头(1) + pitch(4) + yaw(4) + fire_advice(1) + 填充位(4) + 校验位(1) + 尾(1)
  static constexpr size_t SEND_PACKET_SIZE = 16;
  // 接收数据包结构：头(1) + mode(1) + 填充位(12) + 校验位(1) + 尾(1)
  static constexpr size_t RECEIVE_PACKET_SIZE = 16;

  // 打包发送数据
  static std::vector<uint8_t> pack_send_data(const dart_interfaces::msg::SerialSendData & msg);
  
  // 解包接收数据
  static std::unique_ptr<dart_interfaces::msg::SerialReceiveData> unpack_receive_data(
    const uint8_t* buffer, size_t size);

private:
  // 数据类型转换工具
  static void float_to_bytes(float value, uint8_t* bytes);
  static float bytes_to_float(const uint8_t* bytes);
  static uint8_t calculate_checksum(const uint8_t* data, size_t size);
};

}  // namespace dart_serial

#endif  // DART_SERIAL_UART_PROTOCOL_HPP_
    