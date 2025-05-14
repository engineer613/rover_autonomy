#include <iostream>
#include <thread>
#include "SerialInterface.hpp"

int main() {
  SerialInterface gps_serial("/dev/gps");

  if (gps_serial.isOpen()) {
    gps_serial.startReading();
  }

  std::this_thread::sleep_for(10000ms);
  return 0;
}

