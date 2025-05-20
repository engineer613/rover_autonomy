#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "serial_interface/serial_defs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "serial_interface/msg/serial_stamped.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::UInt8MultiArray;
using serial_interface::msg::SerialStamped;

namespace serial_interface {
  class RCLCPP_PUBLIC SerialInterface : public rclcpp::Node {
  public:
    SerialInterface(const rclcpp::NodeOptions &options);
    ~SerialInterface();

    // Delete copy and move constructors/assignments
    SerialInterface(const SerialInterface &) = delete;
    SerialInterface(SerialInterface &&) = delete;
    SerialInterface &operator=(const SerialInterface &) = delete;
    SerialInterface &operator=(SerialInterface &&) = delete;

    bool openPort();
    bool closePort();
    void startReading();
    bool isOpen() { return fd_ >= 0; };

  private:
    std::string device_path_{};
    speed_t baud_rate_{B115200};
    uint8_t data_bits_{8};
    uint8_t stop_bits_{1};
    Parity parity_{Parity::NONE};

    int fd_; // file descriptor for port
    struct timeval timeout_{};
    int timeout_ms_{0};
    uint8_t read_buffer_[BUFFER_SIZE];

    std::atomic<bool> stop_reading_{false};
    std::thread read_thread_;
    rclcpp::Publisher<SerialStamped>::SharedPtr serial_data_publisher_;

    void readThreadFunction();
    bool waitForRxData();

    std::ostringstream oss_;
  };
} //  namespace serial_interface