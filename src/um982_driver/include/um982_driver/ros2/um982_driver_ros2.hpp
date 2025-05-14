#ifndef UM982_DRIVER_UM982_DRIVER_ROS2_HPP
#define UM982_DRIVER_UM982_DRIVER_ROS2_HPP

#include "um982_driver/core/UnicoreObservers.hpp"
#include "um982_driver/core/UnicoreMsgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "unicore_msgs/msg/uniheading_short.hpp"
#include "unicore_msgs/msg/agric_short.hpp"
#include "unicore_msgs/msg/pvtsln_short.hpp"

template <typename MSG_TYPE>
class ROS2Publisher : public UnicoreObserver<MSG_TYPE> {
public:
  explicit ROS2Publisher(rclcpp::Node::SharedPtr node, const std::string& topic)
  {
    pub_ = node->create_publisher<MSG_TYPE>(topic, 10); // Publisher Queue depth = 10
  }

  void notify(const MSG_TYPE& msg) override
  {
    pub_->publish(msg);
  }

private:
  typename rclcpp::Publisher<MSG_TYPE>::SharedPtr pub_;
};

#endif // UM982_DRIVER_UM982_DRIVER_ROS2_HPP
