#include "dart_serial/protocol/serial_protocol.hpp"

#include <cstring>
#include <iomanip>
#include <sstream>

namespace pka
{

void SerialProtocol::float_to_bytes(float value, uint8_t * bytes)
{
  if (!bytes) {
    return;
  }
  uint32_t temp = 0;
  std::memcpy(&temp, &value, sizeof(value));
  bytes[0] = static_cast<uint8_t>(temp & 0xFF);
  bytes[1] = static_cast<uint8_t>((temp >> 8) & 0xFF);
  bytes[2] = static_cast<uint8_t>((temp >> 16) & 0xFF);
  bytes[3] = static_cast<uint8_t>((temp >> 24) & 0xFF);
}

std::string SerialProtocol::format_hex(const std::vector<uint8_t> & packet)
{
  std::ostringstream ss;
  for (size_t i = 0; i < packet.size(); ++i) {
    if (i != 0) {
      ss << ' ';
    }
    ss << std::uppercase << std::setw(2) << std::setfill('0') << std::hex
       << static_cast<int>(packet[i]);
  }
  return ss.str();
}

}  // namespace pka
