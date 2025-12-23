#ifndef DART_SERIAL_UART_DRIVER_HPP_
#define DART_SERIAL_UART_DRIVER_HPP_

#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

namespace pka
{

class UARTDriver
{
public:
  explicit UARTDriver(const std::string & port_name, int baudrate);
  ~UARTDriver();

  bool open_port(); // 初始化串口
  void close();
  bool is_open() const;

  ssize_t read_data(uint8_t * buffer, size_t size);
  ssize_t write_data(const uint8_t * buffer, size_t size);

private:
  std::string port_name_;
  int baudrate_; // 波特率
  int fd_; // 文件描述符
  bool is_open_;
};

}  // namespace dart_serial

#endif  // DART_SERIAL_UART_DRIVER_HPP_
