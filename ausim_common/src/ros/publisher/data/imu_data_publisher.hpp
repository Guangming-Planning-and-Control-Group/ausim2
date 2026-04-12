#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

#include "ros/publisher/i_telemetry_publisher.hpp"

namespace ausim {

class ImuDataPublisher : public ITelemetryPublisher {
 public:
  ImuDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string topic_name,
      std::string frame_id);

  void Publish(const ipc::TelemetryPacket& packet) override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::string frame_id_;
};

}  // namespace ausim
