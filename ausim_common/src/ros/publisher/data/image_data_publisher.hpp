#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "runtime/runtime_types.hpp"

namespace ausim {

class ImageDataPublisher {
 public:
  ImageDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string topic_name,
      std::string frame_id);

  void Publish(const CameraFrame& frame);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::string frame_id_;
};

}  // namespace ausim
