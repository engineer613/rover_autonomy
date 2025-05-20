#include "serial_interface/serial_interface.hpp"

namespace serial_interface {

SerialInterface::SerialInterface(const rclcpp::NodeOptions &options)
    : rclcpp::Node("serial_interface", options) {
  this->declare_parameter("port", "/dev/gps");
  this->declare_parameter("baud_rate", B115200);
  this->declare_parameter("timeout_ms", 1500);

  this->get_parameter("port", device_path_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("timeout_ms", timeout_ms_);

  timeout_.tv_sec = timeout_ms_ / 1000;
  timeout_.tv_usec = (timeout_ms_ % 1000) * 1000;

  this->serial_data_publisher_ =
      this->create_publisher<SerialStamped>("serial_rx", 10);

  if (!this->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open Serial Port");
  } else {
    this->startReading();
  }
}

SerialInterface::~SerialInterface() {
  if (this->read_thread_.joinable()) {
    this->read_thread_.join();
  }
  this->closePort();
}

bool SerialInterface::openPort() { // Read/write
  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY);

  // Non blocking mode doesn't work
  // fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ < 0) {
    throw std::runtime_error("Failed to open port");
  }

  // Get current settings
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    throw std::runtime_error("Failed to get terminal attributes");
  }

  // set baudrate
  cfsetispeed(&tty, baud_rate_);
  cfsetospeed(&tty, baud_rate_);

  // Set 8N1 (8 data bits, no parity, 1 stop bit)
  tty.c_cflag &= ~PARENB; // no parity
  tty.c_cflag &= ~CSTOPB; // one stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8; // 8 bits

  // No flow control
  tty.c_cflag &= ~CRTSCTS;

  // Enable receiver and set local mode
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw bytes (Not Canonical) | No Echo | Dont echo erase (backspace) | Disable signals (Ctrl C)
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Software flow control settings - unnecessary
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Output flag to post process data- want to send bytes exactly as written
  tty.c_oflag &= ~OPOST;

  // Apply flags
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    throw std::runtime_error("Failed to set terminal attributes");
    return false;
  }
  return true;
}

bool SerialInterface::closePort() {
  if (fd_ >= 0) {
    std::cout << "Closing Serial Port" << std::endl;
    ::close(fd_);
    return true;
  }
  return false;
}

void SerialInterface::startReading() {
  if (this->isOpen()) {
    read_thread_ = std::thread(&SerialInterface::readThreadFunction, this);
  }
}

bool SerialInterface::waitForRxData() {
  fd_set read_fds;        // file descriptor set
  FD_ZERO(&read_fds);     // clear it
  FD_SET(fd_, &read_fds); // read fd_ into the fd_set instance

  // select modified the timeout value so it's better to pass a copy
  struct timeval timeout_copy = timeout_;

  auto ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout_copy);

  // ret > 0 : Atleast one fd is ready
  // FD_ISSET(fd, &read_fds) : Is fd ready in the readfds set after the select() call
  if (ret > 0 && FD_ISSET(fd_, &read_fds)) {
    return true;
  }
  return false;
}

void SerialInterface::readThreadFunction() {
  while (fd_ >= 0) {
    if (waitForRxData()) {
      ssize_t bytes_read = ::read(fd_, read_buffer_, BUFFER_SIZE);
      rclcpp::Time t_arrival = this->now();

      if (bytes_read > 0) {
//        oss_ << "Raw read: ";
//        for (int i = 0; i < bytes_read; ++i) {
//          oss_ << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(read_buffer_[i]) << " ";
//        }
//        RCLCPP_WARN(this->get_logger(), "%s", oss_.str().c_str());

        auto msg = SerialStamped();
        msg.stamp = t_arrival;
        msg.data.assign(read_buffer_, (read_buffer_ + bytes_read));
        serial_data_publisher_->publish(msg);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Serial Read Error: %s",
                     strerror(errno));
      }

    } else {
      std::cout << "No data available" << std::endl;
      std::this_thread::sleep_for(100ms);
    }
  }
}

} // namespace serial_interface

// For composing as a composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_interface::SerialInterface)