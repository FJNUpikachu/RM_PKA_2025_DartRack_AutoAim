#include "dart_serial/uart_protocol.hpp"
#include <cstring>
#include <stdexcept>

namespace dart_serial
{

// 浮点数转字节
void UARTProtocol::float_to_bytes(float value, uint8_t* bytes)
{
  if (!bytes) return;
  
  uint32_t temp;
  std::memcpy(&temp, &value, sizeof(value));
  bytes[0] = temp & 0xFF; // & 0xFF:保留该整数的最低 8 位，屏蔽掉更高位的所有数据
  bytes[1] = (temp >> 8) & 0xFF;
  bytes[2] = (temp >> 16) & 0xFF;
  bytes[3] = (temp >> 24) & 0xFF;
}

float UARTProtocol::bytes_to_float(const uint8_t* bytes)
{
  if (!bytes) return 0.0f;
  
  uint32_t temp = (static_cast<uint32_t>(bytes[3]) << 24) |
                  (static_cast<uint32_t>(bytes[2]) << 16) |
                  (static_cast<uint32_t>(bytes[1]) << 8) |
                  static_cast<uint32_t>(bytes[0]);
                  
  float value;
  std::memcpy(&value, &temp, sizeof(value));
  return value;
}

// 简单异或校验（暂时弃用）
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

  // pitch（float32，4字节）
  uint8_t pitch_bytes[4];
  float_to_bytes(msg.pitch, pitch_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = pitch_bytes[i];
  }

  // yaw（float32，4字节）
  uint8_t yaw_bytes[4];
  float_to_bytes(msg.yaw, yaw_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = yaw_bytes[i];
  }

  // fire_advice（uint8_t，1字节）
  buffer[index++] = static_cast<uint8_t>(msg.fire_advice);

  // 填充位（4字节，保持0），index从10到13
  index += 4;

  // 校验位（1字节）- 基于有效数据计算（头 + pitch + yaw + fire_advice）
  //uint8_t checksum = calculate_checksum(&buffer[0], 10);
  uint8_t checksum = 0;
  buffer[index++] = checksum;
  
  // 帧尾（1字节）
  buffer[index] = FRAME_TAIL;

  return buffer;
}

std::unique_ptr<dart_interfaces::msg::SerialReceiveData> UARTProtocol::unpack_receive_data(
  const uint8_t* buffer, size_t size)
{
  // 检查数据长度和帧头帧尾: 后续要写一个检测丢包
  if (!buffer || size != RECEIVE_PACKET_SIZE || 
      buffer[0] != FRAME_HEADER || buffer[size - 1] != FRAME_TAIL) {
    return nullptr;
  }

  // 验证校验位（基于有效数据：头 + mode）
  /*
  uint8_t checksum = calculate_checksum(&buffer[0], 2);  // 前2字节是有效数据
  if (buffer[size - 2] != checksum) {
    return nullptr;
  }
  */

  // 解析数据
  auto msg = std::make_unique<dart_interfaces::msg::SerialReceiveData>();
  
  // mode（uint8_t，1字节）- 位于第2个字节
  msg->mode = buffer[1];

  return msg;
}

}  // namespace dart_serial
    