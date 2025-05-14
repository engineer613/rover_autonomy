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

#include "utils/PooledQueue.hpp"
#include "serial_interface/SerialDefs.hpp"


using namespace std::chrono_literals;

class SerialInterface {
public:
  SerialInterface(const std::string& device_path,
                  int baud_rate = 115200,
                  int timeout_ms = 1500,
                  uint8_t data_bits = 8,
                  uint8_t stop_bits = 1,
                  Parity parity = Parity::NONE);

  ~SerialInterface();

  // Delete copy and move constructors/assignments
  SerialInterface(const SerialInterface&) = delete;
  SerialInterface(SerialInterface&&) = delete;
  SerialInterface& operator=(const SerialInterface&) = delete;
  SerialInterface& operator=(SerialInterface&&) = delete;

  bool openPort();
  bool closePort();
  void startReading();
  bool isOpen(){return fd_ >= 0;};
  std::shared_ptr<SerialQueue> getQueuePtr();


private:
  const std::string& device_path_;
  speed_t baud_rate_;
  uint8_t data_bits_{8};
  uint8_t stop_bits_{1};
  Parity parity_;

  int fd_; // file descriptor for port
  struct timeval timeout_{};
  int timeout_ms_{0};

  std::shared_ptr<SerialQueue> output_queue_;
  std::atomic<bool> stop_reading_{false};
  std::thread read_thread_;
  void readThreadFunction();
  bool waitForRxData();

};
