#ifndef DART_SERIAL_PROTOCOL_SERIAL_PROTOCOL_HPP_
#define DART_SERIAL_PROTOCOL_SERIAL_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dart_interfaces/msg/serial_send_data.hpp"

namespace pka
{

class SerialProtocol
{
public:
  virtual ~SerialProtocol() = default;

  virtual size_t send_packet_size() const noexcept = 0;

  virtual std::vector<uint8_t> pack_send_data(
    const dart_interfaces::msg::SerialSendData & msg) const = 0;

  static std::string format_hex(const std::vector<uint8_t> & packet);

protected:
  static void float_to_bytes(float value, uint8_t * bytes);
};

}  // namespace pka

#endif  // DART_SERIAL_PROTOCOL_SERIAL_PROTOCOL_HPP_
