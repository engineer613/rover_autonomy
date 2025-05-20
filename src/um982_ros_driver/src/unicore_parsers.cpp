#include "um982_ros_driver/um982_ros_driver.hpp"
#include "um982_ros_driver/unicore_parsers.hpp"

using um982_ros_driver::UM982ROSDriver;
using std::enable_shared_from_this;

namespace um982_ros_driver {

  UNIHEADINGHandler::UNIHEADINGHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)  {
    ros_pub_ = node->create_publisher<unicore_msgs::msg::UniheadingShort>(
        "uniheading", 10);
  }

  void UNIHEADINGHandler::handleLog(std::vector <uint8_t>&& log_buffer)  {

    unicore_msgs::msg::UniheadingShort msg;

    const size_t H = UNICORE_HEADER_LEN;

    msg.pos_type = readValue<uint32_t>(log_buffer, H + 4);
    msg.baseline = readValue<float>(log_buffer, H + 8);
    msg.heading = readValue<float>(log_buffer, H + 12);
    msg.pitch = readValue<float>(log_buffer, H + 16);
    msg.heading_sigma = readValue<float>(log_buffer, H + 20);
    msg.pitch_sigma = readValue<float>(log_buffer, H + 24);

    ros_pub_->publish(msg);
  }





  AGRICHandler::AGRICHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    ros_pub_ = node->create_publisher<unicore_msgs::msg::AgricShort>(
        "agric", 10);
  }

  void AGRICHandler::handleLog(std::vector<uint8_t>&& log_buffer)  {

    unicore_msgs::msg::AgricShort msg;

    const size_t H = UNICORE_HEADER_LEN;

    msg.lat = readValue<double>(log_buffer, H + 80);
    msg.lon = readValue<double>(log_buffer, H + 88);
    msg.alt = readValue<double>(log_buffer, H + 96);

    msg.lat_sigma = readValue<double>(log_buffer, H + 128);
    msg.lon_sigma = readValue<double>(log_buffer, H + 132);
    msg.alt_sigma = readValue<double>(log_buffer, H + 136);

    msg.speed = readValue<float>(log_buffer, H + 52);
    msg.vel_n = readValue<float>(log_buffer, H + 56);
    msg.vel_e = readValue<float>(log_buffer, H + 60);
    msg.vel_u = readValue<float>(log_buffer, H + 64);

    msg.vel_n_sigma = readValue<float>(log_buffer, H + 68);
    msg.vel_e_sigma = readValue<float>(log_buffer, H + 72);
    msg.vel_u_sigma = readValue<float>(log_buffer, H + 76);

    msg.heading = readValue<float>(log_buffer, H + 40);
    msg.pitch = readValue<float>(log_buffer, H + 44);
    msg.roll = readValue<float>(log_buffer, H + 48);

    msg.speed_heading = readValue<float>(log_buffer, 208);

    ros_pub_->publish(msg);
  }




  PVTSLNHandler::PVTSLNHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    ros_pub_ = node->create_publisher<PvtslnShort>("pvtsln", 10);
  }

  void PVTSLNHandler::handleLog(std::vector<uint8_t>&& log_buffer)  {

    unicore_msgs::msg::PvtslnShort msg;

    const size_t H = UNICORE_HEADER_LEN;

    msg.pos_type = readValue<uint32_t>(log_buffer, H);
    msg.lat = readValue<double>(log_buffer, H + 8);
    msg.lon = readValue<double>(log_buffer, H + 16);
    msg.alt = readValue<float>(log_buffer, H + 4);

    msg.alt_sigma = readValue<float>(log_buffer, H + 24);
    msg.lat_sigma = readValue<float>(log_buffer, H + 28);
    msg.lon_sigma = readValue<float>(log_buffer, H + 32);

    msg.vel_n = readValue<double>(log_buffer, H + 72);
    msg.vel_e = readValue<double>(log_buffer, H + 80);
    msg.vel_gnd = readValue<double>(log_buffer, H + 88);

    msg.heading_type = readValue<uint32_t>(log_buffer, H + 96);
    msg.baseline = readValue<float>(log_buffer, H + 100);
    msg.heading_deg = readValue<float>(log_buffer, H + 104);
    msg.pitch_deg = readValue<float>(log_buffer, H + 108);

    ros_pub_->publish(msg);
  }




  RTKSTATUSHandler::RTKSTATUSHandler(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    ros_pub_ = node->create_publisher
               <unicore_msgs::msg::RTKStatusShort>("rtkstatus", 10);
  }

  void RTKSTATUSHandler::handleLog(std::vector<uint8_t>&& log_buffer) {
    const UnicoreLogMsgs::RTKSTATUS* log =
        reinterpret_cast<const UnicoreLogMsgs::RTKSTATUS*>(log_buffer.data());

    unicore_msgs::msg::RTKStatusShort msg;

    msg.pos_type = log->positionType;
    msg.calc_status = log->calcStatus;
    msg.rtk_good = true;
    msg.rtcm_good = true;
    ros_pub_->publish(msg);
  }

}


