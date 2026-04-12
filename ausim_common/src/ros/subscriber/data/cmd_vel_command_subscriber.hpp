#pragma once

#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "data/cmd_vel.hpp"
#include "ros/subscriber/i_command_subscriber.hpp"

namespace ausim {

class CmdVelCommandSubscriber : public ICommandSubscriber {
 public:
  CmdVelCommandSubscriber(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::function<void(const data::CmdVelData&)> on_message);

 private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace ausim
