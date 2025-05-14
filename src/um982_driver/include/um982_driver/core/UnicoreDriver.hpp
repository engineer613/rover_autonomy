#ifndef ROVER_AUTONOMY_UNICOREDRIVER_HPP
#define ROVER_AUTONOMY_UNICOREDRIVER_HPP

#include <string>
#include <functional>
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <thread>
#include <chrono>

#include "um982_driver/core/UnicoreDriver.hpp"
#include "um982_driver/core/UnicoreMsgs.hpp"
#include "um982_driver/core/UnicoreObservers.hpp"
#include "um982_driver/core/UnicoreParsers.hpp"
#include "um982_driver/ros2/um982_driver_ros2.hpp"
#include "serial_interface/SerialInterface.hpp"

using namespace std::chrono_literals;
using namespace UnicoreMsgs;

class UnicoreDriver {
  friend class UNIHEADINGParser;
  friend class AGRICParser;
  friend class PVTSLNParser;

public:
  UnicoreDriver(const std::string& device_path,
                int baud_rate,
                int timeout_ms,
                uint8_t data_bits,
                uint8_t stop_bits,
                Parity parity);
  ~UnicoreDriver();

  // Delete Moves and copies constructor/assignment
  UnicoreDriver(const UnicoreDriver&) = delete; // copy constructor
  UnicoreDriver(UnicoreDriver&&) = delete; // move constructor
  UnicoreDriver& operator=(const UnicoreDriver&) = delete; //copy assignment
  UnicoreDriver& operator=(UnicoreDriver&&) = delete;  // move assignment

  void registerSerialQueue(std::shared_ptr<SerialQueue> queue_ptr);

private:
  std::unique_ptr<SerialInterface> serial_;
  // Maps LOG msg IDs to parser functions
  std::unordered_map<uint16_t, std::function<void(const uint8_t*)>
                     > log_router_;

  std::thread driver_thread_;
  std::shared_ptr<SerialQueue> serial_queue_;

  SerialBuffer* serial_buffer_;
  std::vector<uint8_t> buffer_;
  std::vector<uint8_t>::iterator sync_it_;

  UNIHEADINGShort uniheading_short_output_;
  AGRICShort agric_short_output_;
  PVTSLNShort pvtsln_short_output_;

  UnicoreObserver<AGRICShort>* agric_ros_pub_;
  UnicoreObserver<UNIHEADINGShort>* uniheading_ros_pub_;
  UnicoreObserver<PVTSLNShort>* pvtsln_ros_pub_;

  void publishUNIHEADING();
  void publishAGRIC();
  void publishPVTSLN();

  bool getSerialData();
  void frameNextLog();
  void driverThreadFunction();
};

#endif // ROVER_AUTONOMY_UNICOREDRIVER_HPP

