#pragma once

#include <cstring>
#include <iostream>
#include <string>

#include "unicore_log_messages.hpp"

#include "rclcpp/rclcpp.hpp"
#include "unicore_msgs/msg/agric_short.hpp"
#include "unicore_msgs/msg/pvtsln_short.hpp"
#include "unicore_msgs/msg/uniheading_short.hpp"
#include "unicore_msgs/msg/rtk_status_short.hpp"


using UnicoreLogMsgs::UNICORE_HEADER_LEN;
using unicore_msgs::msg::UniheadingShort;
using unicore_msgs::msg::AgricShort;
using unicore_msgs::msg::PvtslnShort;
using unicore_msgs::msg::RTKStatusShort;

// Helper template to read different datatypes from binary LOGs
template<typename T>
T readValue(std::vector<uint8_t>& log_buffer, size_t offset) {
  T val;
  std::memcpy(&val, log_buffer.data() + offset, sizeof(T));
  return val;
}

namespace um982_ros_driver {

  class UM982ROSDriver;

  class LogHandler {
  public:
    virtual void handleLog( std::vector<uint8_t>&& log_buffer) = 0;
    virtual ~LogHandler() = default;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
  };



  class UNIHEADINGHandler : public LogHandler {
  public:
    explicit UNIHEADINGHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void handleLog(std::vector<uint8_t>&& log_buffer) override;
    void activate() override {
      if (ros_pub_) ros_pub_->on_activate();
    }

    void deactivate() override {
      if (ros_pub_) ros_pub_->on_deactivate();
    }

  private:
    rclcpp_lifecycle::LifecyclePublisher<unicore_msgs::msg::UniheadingShort>::SharedPtr ros_pub_;
  };




  class AGRICHandler : public LogHandler {
  public:
    explicit AGRICHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void handleLog(std::vector<uint8_t>&& log_buffer) override;
    void activate() override {
      if (ros_pub_) ros_pub_->on_activate();
    }

    void deactivate() override {
      if (ros_pub_) ros_pub_->on_deactivate();
    }

  private:
    rclcpp_lifecycle::LifecyclePublisher<unicore_msgs::msg::AgricShort>::SharedPtr ros_pub_;
  };




  class PVTSLNHandler : public LogHandler {
  public:
    explicit PVTSLNHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void handleLog(std::vector<uint8_t>&& log_buffer) override;
    void activate() override {
      if (ros_pub_) ros_pub_->on_activate();
    }

    void deactivate() override {
      if (ros_pub_) ros_pub_->on_deactivate();
    }

  private:
    rclcpp_lifecycle::LifecyclePublisher<unicore_msgs::msg::PvtslnShort>::SharedPtr ros_pub_;
  };





  class RTKSTATUSHandler : public LogHandler {
  public:
    explicit RTKSTATUSHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void handleLog(std::vector<uint8_t>&& log_buffer) override;
    void activate() override {
      if (ros_pub_) ros_pub_->on_activate();
    }

    void deactivate() override {
      if (ros_pub_) ros_pub_->on_deactivate();
    }

  private:
    rclcpp_lifecycle::LifecyclePublisher<unicore_msgs::msg::RTKStatusShort>::SharedPtr ros_pub_;
  };

}