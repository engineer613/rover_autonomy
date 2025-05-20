#ifndef ROVER_AUTONOMY_UM982ROSDRIVER_HPP
#define ROVER_AUTONOMY_UM982ROSDRIVER_HPP

#include <string>
#include <functional>
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>

#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "serial_interface/serial_interface.hpp"
#include "um982_ros_driver/unicore_log_messages.hpp"
#include "um982_ros_driver/unicore_parsers.hpp"

#include "unicore_msgs/msg/uniheading_short.hpp"
#include "unicore_msgs/msg/agric_short.hpp"
#include "unicore_msgs/msg/pvtsln_short.hpp"

using namespace std::chrono_literals;

using UnicoreLogMsgs::UNICORE_HEADER_LEN;
using UnicoreLogMsgs::MIN_LOG_LEN;

using namespace rclcpp_lifecycle::node_interfaces;

namespace um982_ros_driver {

class RCLCPP_PUBLIC UM982ROSDriver : public rclcpp_lifecycle::LifecycleNode
{
  friend class UNIHEADINGHandler;
  friend class AGRICHandler;
  friend class PVTSLNHandler;
  friend class RTKSTATUSHandler;

public:
  explicit UM982ROSDriver(const rclcpp::NodeOptions &options);
  ~UM982ROSDriver();

  // Delete Moves and copies
  UM982ROSDriver(const UM982ROSDriver &) = delete; // copy constructor
  UM982ROSDriver(UM982ROSDriver &&) = delete;      // move constructor
  UM982ROSDriver &operator=(const UM982ROSDriver &) = delete; // copy assignment
  UM982ROSDriver &operator=(UM982ROSDriver &&) = delete;      // move assignment

private:
  // Maps LOG msg IDs to parser functions
  std::unordered_map<uint16_t, std::shared_ptr<LogHandler>> log_router_;
  void initLogHandlers();

  std::vector<uint8_t> serial_buffer_;
  std::mutex buffer_mutex_;
  std::condition_variable data_ready_;

  rclcpp::Subscription<serial_interface::msg::SerialStamped>::SharedPtr serial_data_sub_;

  std::thread driver_thread_;
  std::atomic<bool> running_{true};

  void onSerialMsg(const SerialStamped::SharedPtr msg);
  bool frameNextLog();
  void driverThreadFunction();

  std::ostringstream oss_;
  
  // ROS2 Lifecycle callbacks
  LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state);

  LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);

  LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);
};

} // namespace um982_ros_driver

#endif // ROVER_AUTONOMY_UM982ROSDRIVER_HPP

