#ifndef DART_SERIAL_PROTOCOL_DART_PROTOCOL_HPP_
#define DART_SERIAL_PROTOCOL_DART_PROTOCOL_HPP_

#include "dart_serial/protocol/serial_protocol.hpp"

namespace pka
{

/// 飞镖架下行协议：每帧 16 字节（仅发送，不解析接收）
class DartProtocol final : public SerialProtocol
{
public:
  static constexpr uint8_t kFrameHeader = 0xFF;
  static constexpr uint8_t kFrameTail = 0x0D;
  static constexpr size_t kSendPacketSize = 16;

  size_t send_packet_size() const noexcept override { return kSendPacketSize; }

  std::vector<uint8_t> pack_send_data(
    const dart_interfaces::msg::SerialSendData & msg) const override;
};

}  // namespace pka

#endif  // DART_SERIAL_PROTOCOL_DART_PROTOCOL_HPP_
