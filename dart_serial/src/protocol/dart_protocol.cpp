#include "dart_serial/protocol/dart_protocol.hpp"

namespace pka
{

std::vector<uint8_t> DartProtocol::pack_send_data(
  const dart_interfaces::msg::SerialSendData & msg) const
{
  std::vector<uint8_t> buffer(kSendPacketSize, 0);
  size_t index = 0;

  buffer[index++] = kFrameHeader;

  uint8_t yaw_bytes[4] = {0};
  float_to_bytes(msg.yaw, yaw_bytes);
  for (int i = 0; i < 4; ++i) {
    buffer[index++] = yaw_bytes[i];
  }

  buffer[index++] = static_cast<uint8_t>(msg.fire_advice);
  index += 8;

  // 校验字节固定为 0（不使用异或或其它算法）
  buffer[index++] = 0;
  buffer[index] = kFrameTail;
  return buffer;
}

}  // namespace pka
