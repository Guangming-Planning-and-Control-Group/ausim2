#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros/publisher/i_telemetry_publisher.hpp"

namespace ausim {

class OdomDataPublisher : public ITelemetryPublisher {
 public:
  OdomDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id, std::string child_frame_id);

  void Publish(const ipc::TelemetryPacket& packet) override;

 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::string frame_id_;
  std::string child_frame_id_;
};

}  // namespace ausim
