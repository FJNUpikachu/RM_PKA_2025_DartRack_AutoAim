#include "dart_serial/uart_protocol.hpp"
#include <cstring>
#include <stdexcept>

namespace pka
{

// 浮点数转字节
void UARTProtocol::float_to_bytes(float value, uint8_t* bytes)
{
  if (!bytes) return;
  
  uint32_t temp;
  std::memcpy(&temp, &value, sizeof(value));
  bytes[0] = temp & 0xFF;
  bytes[1] = (temp >> 8) & 0xFF;
  bytes[2] = (temp >> 16) & 0xFF;
  bytes[3] = (temp >> 24) & 0xFF;
}

// 简单异或校验
uint8_t UARTProtocol::calculate_checksum(const uint8_t* data, size_t size)
{
  if (!data || size == 0) return 0;
  
  uint8_t checksum = 0;
  for (size_t i = 0; i < size; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

std::vector<uint8_t> UARTProtocol::pack_send_data(const dart_interfaces::msg::SerialSendData & msg)
{
  // 初始化16字节数据包，所有位先置0
  std::vector<uint8_t> buffer(SEND_PACKET_SIZE, 0);
  size_t index = 0;

  // 帧头 (1字节)
  buffer[index++] = FRAME_HEADER;

  // yaw（float32，4字节）
  uint8_t yaw_bytes[4];
  float_to_bytes(msg.yaw, yaw_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = yaw_bytes[i];
  }

  // fire_advice（uint8_t，1字节）
  buffer[index++] = static_cast<uint8_t>(msg.fire_advice);

  // 填充位（8字节，保持0）
  index += 8;

  // 校验位（1字节）
  uint8_t checksum = calculate_checksum(&buffer[0], index);
  buffer[index++] = checksum;
  
  // 帧尾（1字节）
  buffer[index] = FRAME_TAIL;

  return buffer;
}

}  // namespace pka