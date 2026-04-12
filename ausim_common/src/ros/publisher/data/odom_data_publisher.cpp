#include "ros/publisher/data/odom_data_publisher.hpp"

#include <utility>

#include "converts/data/odom.hpp"
#include "converts/ipc/bridge_packets.hpp"

namespace ausim {

OdomDataPublisher::OdomDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id,
                                     std::string child_frame_id)
    : publisher_(node->create_publisher<nav_msgs::msg::Odometry>(std::move(topic_name), 10)),
      frame_id_(std::move(frame_id)),
      child_frame_id_(std::move(child_frame_id)) {}

void OdomDataPublisher::Publish(const ipc::TelemetryPacket& packet) {
  publisher_->publish(converts::ToRosMessage(converts::ToOdomData(packet, frame_id_, child_frame_id_)));
}

}  // namespace ausim
