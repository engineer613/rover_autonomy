#ifndef ROVER_AUTONOMY_SERIAL_DEFS_HPP
#define ROVER_AUTONOMY_SERIAL_DEFS_HPP

#include <string>
#include <array>
#include <cstddef>
#include <chrono>

constexpr size_t BUFFER_SIZE = 1024;

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
