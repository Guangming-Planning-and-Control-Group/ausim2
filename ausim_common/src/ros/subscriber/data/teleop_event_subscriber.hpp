#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros/subscriber/i_command_subscriber.hpp"

namespace ausim {

class TeleopEventSubscriber : public ICommandSubscriber {
 public:
  TeleopEventSubscriber(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::function<void(const std::string&)> on_message);

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace ausim
