#include "ros/subscriber/data/teleop_event_subscriber.hpp"

namespace ausim {

TeleopEventSubscriber::TeleopEventSubscriber(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name,
                                             std::function<void(const std::string&)> on_message) {
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      std::move(topic_name), 10,
      [on_message = std::move(on_message)](const std_msgs::msg::String::SharedPtr message) {
        if (!message || !on_message) {
          return;
        }
        on_message(message->data);
      });
}

}  // namespace ausim
