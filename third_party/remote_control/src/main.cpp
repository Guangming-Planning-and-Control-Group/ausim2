#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "remote_control_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<remote_control::RemoteControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
