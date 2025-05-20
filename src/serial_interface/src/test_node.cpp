#include "rclcpp/rclcpp.hpp"
#include "serial_interface/serial_interface.hpp"

using serial_interface::SerialInterface;

void signalHandler(int signum) {
  if (signum == SIGINT) {
    std::cout << "Exiting Program" << std::endl;
    std::exit(signum);
  }
}

int main(int argc, char **argv) {
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<SerialInterface>(options);
  while(node->isOpen()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}