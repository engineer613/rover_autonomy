#include "um982_ros_driver/um982_ros_driver.hpp"

namespace um982_ros_driver {
  UM982ROSDriver::UM982ROSDriver(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("um982_driver_node", options) {

    RCLCPP_INFO(this->get_logger(), "um982_driver_node started");
  }


  UM982ROSDriver::~UM982ROSDriver() {
  }

  // ROS2 Lifecycle Callbacks
  LifecycleNodeInterface::CallbackReturn UM982ROSDriver::on_configure(const rclcpp_lifecycle::State &) {
    auto ros_node = this->shared_from_this();

    log_router_[UnicoreLogMsgs::UNIHEADING_MSG_ID] =
        std::make_shared<UNIHEADINGHandler>(ros_node);

    log_router_[UnicoreLogMsgs::AGRIC_MSG_ID] =
        std::make_shared<AGRICHandler>(ros_node);

    log_router_[UnicoreLogMsgs::PVTSLN_MSG_ID] =
        std::make_shared<PVTSLNHandler>(ros_node);

    log_router_[UnicoreLogMsgs::RTKSTATUS_MSG_ID] =
        std::make_shared<RTKSTATUSHandler>(ros_node);

    return CallbackReturn::SUCCESS;
  }


  LifecycleNodeInterface::CallbackReturn UM982ROSDriver::on_activate(
    const rclcpp_lifecycle::State&) {
    // Init Subscriber
    serial_data_sub_ = this->create_subscription<SerialStamped>("serial_rx", 10,
            std::bind(&UM982ROSDriver::onSerialMsg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Activating Publishers..");

    for(auto& [id, handler] :  log_router_) {
      handler->activate();
    }

    driver_thread_ = std::thread(&UM982ROSDriver::driverThreadFunction, this);

    return CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn UM982ROSDriver::on_deactivate(const rclcpp_lifecycle::State&)
  {
    serial_data_sub_.reset();

    for (auto& [id, handler] :  log_router_) {
      handler->deactivate();
    }

    running_ = false;
    if(driver_thread_.joinable()) {
      driver_thread_.join();
    }

    return CallbackReturn::SUCCESS;
  }



  bool UM982ROSDriver::frameNextLog() {
    // Look for sync bytes
    auto erase_upto = serial_buffer_.begin();
    auto sync_it = std::search(serial_buffer_.begin(), serial_buffer_.end(),
                               UnicoreLogMsgs::SYNC_BYTES.begin(),
                               UnicoreLogMsgs::SYNC_BYTES.end());

    while (std::distance(sync_it, serial_buffer_.end()) >= MIN_LOG_LEN) {
      // data is little endian
      uint16_t msg_id = sync_it[4] | (sync_it[5] << 8);
      uint16_t payload_len = sync_it[6] | (sync_it[7] << 8);

      size_t log_len = UnicoreLogMsgs::UNICORE_HEADER_LEN + payload_len;

      // If complete log is found, then send it to parser
      if (std::distance(sync_it, serial_buffer_.end()) >= log_len) {
        std::vector<uint8_t> log_buffer(sync_it, sync_it + log_len);
        if (auto it = log_router_.find(msg_id); it != log_router_.end()) {
          log_router_.at(msg_id)->handleLog(std::move(log_buffer));
        } else {
          std::cerr << "Missing ID in unordered map: " << msg_id << "\n";
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Incomplete LOG in buffer");
        break;
      }
      erase_upto = sync_it + log_len;

      // Find the next set of sync bytes
      sync_it = std::search(erase_upto, serial_buffer_.end(),
                            UnicoreLogMsgs::SYNC_BYTES.begin(),
                            UnicoreLogMsgs::SYNC_BYTES.end());
    }

    // if (erase_upto == serial_buffer_.begin()) return false;

    serial_buffer_.erase(serial_buffer_.begin(), erase_upto);
    return true;
  }



  // Copy serial data into buffer in callback and notify driver thread
  void UM982ROSDriver::onSerialMsg(const SerialStamped::SharedPtr msg) {
    // Acquire lock and copy over data
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    serial_buffer_.insert(serial_buffer_.end(), msg->data.begin(),
                          msg->data.end());

    data_ready_.notify_one(); // Notify driver thread
  }


  // Driver thread frames logs from buffer
  // and passes over to parsers to parse & publish
  void UM982ROSDriver::driverThreadFunction() {
    while (running_) {
      std::unique_lock<std::mutex> lock(buffer_mutex_);

      data_ready_.wait(lock, [this]() {
        return (!serial_buffer_.empty() || !running_);
      });

      if (!frameNextLog()) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to frame LOG!");
      }
    }
  }


} // namespace um982_ros_driver
// For composing as a composable node
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(um982_ros_driver::UM982ROSDriver)