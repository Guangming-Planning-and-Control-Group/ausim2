#include "ros/publisher/data/imu_data_publisher.hpp"

#include <utility>

#include "converts/data/imu.hpp"
#include "converts/ipc/bridge_packets.hpp"

namespace ausim {

ImuDataPublisher::ImuDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id)
    : publisher_(node->create_publisher<sensor_msgs::msg::Imu>(std::move(topic_name), 10)), frame_id_(std::move(frame_id)) {}

void ImuDataPublisher::Publish(const ipc::TelemetryPacket& packet) {
  publisher_->publish(converts::ToRosMessage(converts::ToImuData(packet, frame_id_)));
}

}  // namespace ausim
