#ifndef ROVER_AUTONOMY_SERIAL_DEFS_HPP
#define ROVER_AUTONOMY_SERIAL_DEFS_HPP

#include <string>
#include <array>
#include <cstddef>
#include <chrono>

#include "utils/PooledQueue.hpp"

constexpr int QUEUE_SIZE = 50;
constexpr int BUFFER_SIZE = 256;
using SerialBuffer = std::array<uint8_t, BUFFER_SIZE>;
using SerialQueue = PooledQueue<SerialBuffer, QUEUE_SIZE>;

enum class Parity {
  NONE = 0,
  EVEN,
  ODD
};

static speed_t getTermiosBaud (int baud_rate) {
  switch(baud_rate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  default:
    return -1;
  }
};


#endif // ROVER_AUTONOMY_SERIAL_DEFS_HPP
