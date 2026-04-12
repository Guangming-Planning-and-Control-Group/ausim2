#include "ros/publisher/data/clock_data_publisher.hpp"

#include <utility>

#include "converts/data/clock.hpp"
#include "converts/ipc/bridge_packets.hpp"

namespace ausim {

ClockDataPublisher::ClockDataPublisher(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name)
    : publisher_(node->create_publisher<rosgraph_msgs::msg::Clock>(std::move(topic_name), 10)) {}

void ClockDataPublisher::Publish(const ipc::TelemetryPacket& packet) {
  publisher_->publish(converts::ToRosMessage(converts::ToClockData(packet)));
}

}  // namespace ausim
