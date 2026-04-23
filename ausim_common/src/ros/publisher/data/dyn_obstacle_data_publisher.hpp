#pragma once

#include <memory>
#include <string>

#include <ausim_msg/msg/bounding_box3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros/publisher/id_dyn_obstacle_publisher.hpp"

namespace ausim {

class DynObstacleDataPublisher : public IDynObstaclePublisher {
 public:
  DynObstacleDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string default_frame_id);

  void Publish(const DynamicObstaclesSnapshot& snapshot) override;

 private:
  rclcpp::Publisher<ausim_msg::msg::BoundingBox3DArray>::SharedPtr publisher_;
  std::string default_frame_id_;
};

}  // namespace ausim
