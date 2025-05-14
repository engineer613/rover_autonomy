#include "serial_interface/SerialInterface.hpp"

SerialInterface::SerialInterface(const std::string& device_path,
                                 int baud_rate,
                                 int timeout_ms,
                                 uint8_t data_bits,
                                 uint8_t stop_bits,
                                 Parity parity)
:
  device_path_(std::move(device_path)),
  baud_rate_(getTermiosBaud(baud_rate)),
  data_bits_(data_bits),
  stop_bits_(stop_bits),
  parity_(parity),
  output_queue_(std::make_shared<SerialQueue>())
{
  timeout_.tv_sec = timeout_ms / 1000;
  timeout_.tv_usec = (timeout_ms % 1000) * 1000;

  bool open_ok = this->openPort();
}

SerialInterface::~SerialInterface() {
  if (this->read_thread_.joinable()) {
    this->read_thread_.join();
  }
  this->closePort();
}


bool SerialInterface::openPort()
{  // Read/write
  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY );

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
  tty.c_cflag &= ~PARENB;     // no parity
  tty.c_cflag &= ~CSTOPB;     // one stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         // 8 bits

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
  } else {
    return true;
  }
  return false;
}

std::shared_ptr<SerialQueue> SerialInterface::getQueuePtr() {
  return this->output_queue_;
}



void SerialInterface::startReading() {
  if(this->isOpen()){
    read_thread_ = std::thread(&SerialInterface::readThreadFunction, this);
  }
}

bool SerialInterface::waitForRxData() {
  fd_set read_fds; // file descriptor set
  FD_ZERO(&read_fds); // clear it
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
    // get a new buffer
    SerialBuffer *p_buffer;
    size_t total_bytes_read = 0;

    if (p_buffer = output_queue_->getBuffer()) {
      while (total_bytes_read < p_buffer->size()) {
        if (waitForRxData()) {
          ssize_t bytes_read = ::read(fd_, p_buffer->data() + total_bytes_read,
                                      p_buffer->size() - total_bytes_read);
          if (bytes_read > 0) {
            total_bytes_read += bytes_read;
          } else {
            std::cout << "Serial Read Error: " << strerror(errno) << std::endl;
          }
        } else {
          std::cout << "No data available" << std::endl;
          std::this_thread::sleep_for(100ms);
        }
      }
      output_queue_->pushToQueue(p_buffer);

      SerialBuffer* pop_buffer;
      output_queue_->popQueue(pop_buffer);
      output_queue_->releaseBuffer(pop_buffer);
    }
  }
}



