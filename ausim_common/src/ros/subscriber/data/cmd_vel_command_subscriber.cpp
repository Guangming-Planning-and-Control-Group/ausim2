#include "ros/subscriber/data/cmd_vel_command_subscriber.hpp"

#include <functional>
#include <utility>

#include "converts/data/cmd_vel.hpp"

namespace ausim {

CmdVelCommandSubscriber::CmdVelCommandSubscriber(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name,
    std::function<void(const data::CmdVelData&)> on_message)
    : subscription_(node->create_subscription<geometry_msgs::msg::Twist>(
          std::move(topic_name),
          10,
          [on_message = std::move(on_message)](
              const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (on_message) {
              on_message(converts::ToCmdVelData(*msg));
            }
          })) {}

}  // namespace ausim
