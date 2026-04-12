#include "ros/publisher/data/image_data_publisher.hpp"

#include <utility>

#include "converts/data/image.hpp"
#include "converts/ipc/bridge_packets.hpp"

namespace ausim {

ImageDataPublisher::ImageDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id)
    : publisher_(node->create_publisher<sensor_msgs::msg::Image>(std::move(topic_name), 10)), frame_id_(std::move(frame_id)) {}

void ImageDataPublisher::Publish(const CameraFrame& frame) { publisher_->publish(converts::ToRosMessage(converts::ToImageData(frame, frame_id_))); }

}  // namespace ausim
