#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

#include "ros/publisher/i_telemetry_publisher.hpp"

namespace ausim {

class TransformDataPublisher : public ITelemetryPublisher {
 public:
  TransformDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string frame_id,
      std::string child_frame_id);

  void Publish(const ipc::TelemetryPacket& packet) override;

 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::string frame_id_;
  std::string child_frame_id_;
};

}  // namespace ausim
