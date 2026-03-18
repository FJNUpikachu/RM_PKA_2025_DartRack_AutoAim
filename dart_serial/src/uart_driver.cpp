#include "dart_serial/uart_driver.hpp"

namespace pka
{

UARTDriver::UARTDriver(const std::string & port_name, int baudrate)
: port_name_(port_name), baudrate_(baudrate), fd_(-1), is_open_(false)
{
}

UARTDriver::~UARTDriver()
{
  close();
}

bool UARTDriver::open_port()
{
  if (is_open_) {
    RCLCPP_WARN(rclcpp::get_logger("UARTDriver"), "串口已打开");
    return true;
  }
  /*
  【设置文件描述符】
  全局命名空间的open 
  O_RDWR 以读写模式打开 
  O_NOCTTY 防止该设备成为进程的控制终端
  O_NDELAY 设置非阻塞模式
  */
  fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "无法打开串口: %s", port_name_.c_str());
    return false;
  }

  /*
  【配置串口】
  termios:Linux系统中专门用于终端I/O接口的结构体
  */
  struct termios options;
  if (tcgetattr(fd_, &options) != 0) { //获取与文件描述符fd_关联的终端设备的当前属性配置
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "获取串口属性失败");
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 设置波特率
  speed_t baud_speed;
  switch (baudrate_) {
    case 9600:
      baud_speed = B9600;
      break;
    case 115200:
      baud_speed = B115200;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "不支持的波特率: %d", baudrate_);
      ::close(fd_);
      fd_ = -1;
      return false;
  }
  // 注意输入输出都要同时设置
  cfsetispeed(&options, baud_speed);
  cfsetospeed(&options, baud_speed);

  // 设置数据位、停止位和校验位
  options.c_cflag &= ~PARENB;  // 无校验
  options.c_cflag &= ~CSTOPB;  // 1位停止位
  options.c_cflag &= ~CSIZE;   // 清除当前设置的数据位长度相关标志
  options.c_cflag |= CS8;      // 8位数据位

  // 使能接收和忽略调制解调器状态线
  options.c_cflag |= (CLOCAL | CREAD);

  // 设置为原始模式
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  // 设置超时
  options.c_cc[VMIN] = 0;   // 最少读取0字节
  options.c_cc[VTIME] = 10; // 超时时间1秒(10 * 0.1秒)

  // 应用设置
  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "设置串口参数失败");
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 清空输入输出缓冲区
  tcflush(fd_, TCIOFLUSH);

  is_open_ = true;
  RCLCPP_INFO(rclcpp::get_logger("UARTDriver"), "成功打开串口: %s, 波特率: %d", port_name_.c_str(), baudrate_);
  return true;
}

void UARTDriver::close()
{
  if (is_open_) {
    ::close(fd_);
    is_open_ = false;
    fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("UARTDriver"), "关闭串口: %s", port_name_.c_str());
  }
}

bool UARTDriver::is_open() const
{
  return is_open_;
}

ssize_t UARTDriver::read_data(uint8_t * buffer, size_t size)
{
  if (!is_open_) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "串口未打开，无法读取数据");
    return -1;
  }

  if (!buffer || size == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "无效的读取缓冲区或大小");
    return -1;
  }

  return ::read(fd_, buffer, size);
}

ssize_t UARTDriver::write_data(const uint8_t * buffer, size_t size)
{
  if (!is_open_) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "串口未打开，无法写入数据");
    return -1;
  }

  if (!buffer || size == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("UARTDriver"), "无效的写入缓冲区或大小");
    return -1;
  }

  return ::write(fd_, buffer, size);
}

}  // namespace dart_serial
