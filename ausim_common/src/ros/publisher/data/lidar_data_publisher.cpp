#include "converts/data/lidar.hpp"

#include "ros/publisher/data/lidar_data_publisher.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace ausim {

LidarDataPublisher::LidarDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id)
    : publisher_(node->create_publisher<sensor_msgs::msg::PointCloud2>(std::move(topic_name), rclcpp::QoS(10))), frame_id_(std::move(frame_id)) {}

void LidarDataPublisher::Publish(const ipc::LidarPacket& packet) {
  const int h = packet.h_ray_num;
  const int v = packet.v_ray_num;
  const int total = h * v;

  if (total <= 0 || total > ipc::LidarPacket::kMaxRays) {
    return;
  }

  const std::vector<float> xyz = converts::BuildLidarPointCloudXYZ(packet);

  const uint32_t n_points = static_cast<uint32_t>(xyz.size() / 3);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = static_cast<int32_t>(packet.sim_time);
  msg.header.stamp.nanosec = static_cast<uint32_t>((packet.sim_time - msg.header.stamp.sec) * 1e9);
  msg.header.frame_id = frame_id_;
  msg.height = 1;
  msg.width = n_points;
  msg.is_dense = false;
  msg.is_bigendian = false;

  // Define x, y, z fields (float32).
  msg.fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    msg.fields[i].name = std::string(1, static_cast<char>('x' + i));
    msg.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.point_step = 3 * sizeof(float);
  msg.row_step = msg.point_step * n_points;
  msg.data.resize(msg.row_step);

  if (n_points > 0) {
    std::memcpy(msg.data.data(), xyz.data(), msg.data.size());
  }

  publisher_->publish(msg);
}

}  // namespace ausim
