#include "ros/publisher/data/dyn_obstacle_data_publisher.hpp"

#include <utility>

#include "converts/data/common.hpp"

namespace ausim {

DynObstacleDataPublisher::DynObstacleDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string default_frame_id)
    : publisher_(node->create_publisher<ausim_msg::msg::BoundingBox3DArray>(std::move(topic_name), 10)),
      default_frame_id_(std::move(default_frame_id)) {}

void DynObstacleDataPublisher::Publish(const DynamicObstaclesSnapshot& snapshot) {
  ausim_msg::msg::BoundingBox3DArray message;
  message.header.stamp = converts::ToRosTime(snapshot.sim_time);
  message.header.frame_id = snapshot.frame_id.empty() ? default_frame_id_ : snapshot.frame_id;
  message.boxes.reserve(snapshot.entries.size());

  for (const DynamicObstacleEntry& entry : snapshot.entries) {
    ausim_msg::msg::BoundingBox3D box;
    box.center.position.x = entry.pos[0];
    box.center.position.y = entry.pos[1];
    box.center.position.z = entry.pos[2];
    box.center.orientation.w = entry.quat[0];
    box.center.orientation.x = entry.quat[1];
    box.center.orientation.y = entry.quat[2];
    box.center.orientation.z = entry.quat[3];
    box.size.x = entry.size[0];
    box.size.y = entry.size[1];
    box.size.z = entry.size[2];
    message.boxes.push_back(std::move(box));
  }

  publisher_->publish(std::move(message));
}

}  // namespace ausim
