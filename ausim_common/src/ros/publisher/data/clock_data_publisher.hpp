#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

#include "ros/publisher/i_telemetry_publisher.hpp"

namespace ausim {

class ClockDataPublisher : public ITelemetryPublisher {
 public:
  ClockDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name);

  void Publish(const ipc::TelemetryPacket& packet) override;

 private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
};

}  // namespace ausim
