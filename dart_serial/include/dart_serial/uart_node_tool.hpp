#ifndef DART_SERIAL_UART_NODE_TOOL_HPP_
#define DART_SERIAL_UART_NODE_TOOL_HPP_

#include <string>
#include <vector>

namespace pka
{

class UARTNode;

class UARTNodeTool
{
public:
  static bool configure_and_open(UARTNode * node);
  static bool send_packet(UARTNode * node, const std::vector<uint8_t> & packet);
  static bool restart_serial(UARTNode * node);
  static std::string serial_config_summary(const UARTNode * node);
};

}  // namespace pka

#endif  // DART_SERIAL_UART_NODE_TOOL_HPP_
