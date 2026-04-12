#include "ros/publisher/data/transform_data_publisher.hpp"

#include <utility>

#include "converts/data/transform.hpp"
#include "converts/ipc/bridge_packets.hpp"

namespace ausim {

TransformDataPublisher::TransformDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string frame_id, std::string child_frame_id)
    : broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)),
      frame_id_(std::move(frame_id)),
      child_frame_id_(std::move(child_frame_id)) {}

void TransformDataPublisher::Publish(const ipc::TelemetryPacket& packet) {
  broadcaster_->sendTransform(converts::ToRosMessage(converts::ToTransformData(packet, frame_id_, child_frame_id_)));
}

}  // namespace ausim
