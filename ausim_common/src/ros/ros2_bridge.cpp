#include "ros/ros2_bridge.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cstring>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "converts/data/image.hpp"
#include "converts/ipc/bridge_packets.hpp"
#include "ipc/lidar_packet.hpp"
#include "ipc/socket_packet.hpp"
#include "ipc/socket_stream.hpp"
#include "ros/publisher/data/clock_data_publisher.hpp"
#include "ros/publisher/data/dyn_obstacle_data_publisher.hpp"
#include "ros/publisher/data/image_data_publisher.hpp"
#include "ros/publisher/data/imu_data_publisher.hpp"
#include "ros/publisher/data/lidar_data_publisher.hpp"
#include "ros/publisher/data/odom_data_publisher.hpp"
#include "ros/publisher/data/transform_data_publisher.hpp"
#include "ros/publisher/id_dyn_obstacle_publisher.hpp"
#include "ros/publisher/i_telemetry_publisher.hpp"
#include "ros/publisher/semantic/robot_mode_publisher.hpp"
#include "ros/subscriber/data/cmd_vel_command_subscriber.hpp"
#include "ros/subscriber/i_command_subscriber.hpp"
#include "runtime/robot_mode_state_machine.hpp"

namespace ausim {
namespace {

namespace fs = std::filesystem;
using TriggerService = std_srvs::srv::Trigger;

constexpr std::chrono::milliseconds kDiscreteCommandAckPollPeriod(10);
constexpr std::chrono::milliseconds kDiscreteCommandAckTimeout(1000);
constexpr std::size_t kMaxBridgeMessageSize = 64 * 1024;

struct RosBridgeConfig {
  VehicleIdentity identity;
  Ros2Config ros2;
  RosInterfaceConfig interfaces;
  RosFrameConfig frames;
  std::vector<SensorConfig> sensors;
  DynamicObstacleConfig dynamic_obstacle;
};

std::string NormalizeNamespace(std::string value) {
  if (value.empty() || value == "/") {
    return "";
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string TrimSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string ResolveFrameId(const RosBridgeConfig& config, const std::string& frame_name) {
  const std::string trimmed_frame = TrimSlashes(frame_name);
  const std::string prefix = TrimSlashes(config.identity.frame_prefix);
  if (prefix.empty()) {
    return trimmed_frame;
  }
  if (trimmed_frame.empty()) {
    return prefix;
  }
  return prefix + "/" + trimmed_frame;
}

std::string ResolveTopicName(const RosBridgeConfig& config, const std::string& topic_name) {
  if (topic_name.empty()) {
    return NormalizeNamespace(config.identity.ros_namespace);
  }
  if (!topic_name.empty() && topic_name.front() == '/') {
    return topic_name;
  }

  const std::string ros_namespace = NormalizeNamespace(config.identity.ros_namespace);
  if (ros_namespace.empty()) {
    return "/" + TrimSlashes(topic_name);
  }
  return ros_namespace + "/" + TrimSlashes(topic_name);
}

std::string ResolveServiceName(const RosBridgeConfig& config, const std::string& service_name) {
  return ResolveTopicName(config, service_name);
}

RosBridgeConfig BuildRosBridgeConfig(const QuadrotorConfig& config) {
  RosBridgeConfig bridge_config;
  bridge_config.identity = config.identity;
  bridge_config.ros2 = config.ros2;
  bridge_config.interfaces = config.interfaces;
  bridge_config.frames = config.frames;
  bridge_config.sensors = config.sensors;
  bridge_config.dynamic_obstacle = config.dynamic_obstacle;
  return bridge_config;
}

std::optional<ipc::BridgeMessageType> ParseBridgeMessageType(std::uint8_t wire_value) {
  const auto type = static_cast<ipc::BridgeMessageType>(wire_value);
  switch (type) {
    case ipc::BridgeMessageType::kTelemetry:
    case ipc::BridgeMessageType::kDynamicObstacles:
      return type;
  }
  return std::nullopt;
}

bool IsKnownCameraFrameFormat(std::uint32_t wire_value) {
  switch (static_cast<CameraFrameFormat>(wire_value)) {
    case CameraFrameFormat::kRgb8:
    case CameraFrameFormat::kDepth32F:
      return true;
  }
  return false;
}

bool ValidateImageMetadata(const ipc::CameraImageMetadataPacket& metadata) {
  if (!IsKnownCameraFrameFormat(metadata.pixel_format)) {
    return false;
  }

  const auto format = static_cast<CameraFrameFormat>(metadata.pixel_format);
  const std::uint32_t bytes_per_pixel = CameraFrameBytesPerPixel(format);
  if (bytes_per_pixel == 0) {
    return false;
  }

  return metadata.step == metadata.width * bytes_per_pixel && metadata.data_size == metadata.step * metadata.height;
}

std::string AckStatusMessage(DiscreteCommandAckStatus status, const std::string& event_name) {
  switch (status) {
    case DiscreteCommandAckStatus::kSuccess:
      return event_name + " command accepted";
    case DiscreteCommandAckStatus::kUnsupported:
      return event_name + " command is not supported by this simulator";
    case DiscreteCommandAckStatus::kFailed:
      return event_name + " command failed";
    case DiscreteCommandAckStatus::kNone:
      break;
  }
  return event_name + " command acknowledgement was not received";
}

void EnsureWritableRosHome() {
#if defined(__linux__)
  if (std::getenv("ROS_HOME") == nullptr) {
    const fs::path ros_home = fs::path("/tmp") / "ausim_ros_home";
    std::error_code error_code;
    fs::create_directories(ros_home, error_code);
    setenv("ROS_HOME", ros_home.string().c_str(), 0);
  }
#endif
}

std::string ActiveRmwImplementation() {
  const char* value = std::getenv("RMW_IMPLEMENTATION");
  return value == nullptr ? std::string("<unset>") : std::string(value);
}

bool RosStartupDebugEnabled() {
  const char* value = std::getenv("AUSIM_DEBUG_ROS2_STARTUP");
  return value != nullptr && std::string(value) != "0" && std::string(value) != "false";
}

void RosStartupDebugLog(const std::string& message) {
  if (RosStartupDebugEnabled()) {
    std::cerr << "[ausim][ros2-child] " << message << std::endl;
  }
}

// Builds the fixed set of telemetry publishers from config.
// Each publisher captures its own frame strings and handles conversion internally.
std::vector<std::unique_ptr<ITelemetryPublisher>> BuildPublishers(const std::shared_ptr<rclcpp::Node>& node, const RosBridgeConfig& config,
                                                                  const std::string& odom_frame, const std::string& base_frame,
                                                                  const std::string& imu_frame) {
  std::vector<std::unique_ptr<ITelemetryPublisher>> publishers;

  publishers.push_back(std::make_unique<OdomDataPublisher>(node, ResolveTopicName(config, config.interfaces.odom_topic), odom_frame, base_frame));

  publishers.push_back(std::make_unique<ImuDataPublisher>(node, ResolveTopicName(config, config.interfaces.imu_topic), imu_frame));

  if (config.ros2.publish_clock) {
    publishers.push_back(std::make_unique<ClockDataPublisher>(node, ResolveTopicName(config, config.interfaces.clock_topic)));
  }

  if (config.ros2.publish_tf) {
    publishers.push_back(std::make_unique<TransformDataPublisher>(node, odom_frame, base_frame));
  }

  if (!config.interfaces.robot_mode_structured_topic.empty()) {
    publishers.push_back(std::make_unique<RobotModePublisher>(node, ResolveTopicName(config, config.interfaces.robot_mode_structured_topic),
                                                              config.identity));
  }

  return publishers;
}

class RosBridgeProcess {
 public:
  RosBridgeProcess(RosBridgeConfig config, int telemetry_fd, int command_fd, int discrete_command_fd, int image_fd, int lidar_fd)
      : config_(std::move(config)),
        telemetry_fd_(telemetry_fd),
        command_fd_(command_fd),
        discrete_command_fd_(discrete_command_fd),
        image_fd_(image_fd),
        lidar_fd_(lidar_fd) {}

  ~RosBridgeProcess() { Stop(); }

  int Run() {
    Start();
    executor_->spin();
    Stop();
    return 0;
  }

 private:
  void Start() {
    if (started_) {
      return;
    }
    if (config_.ros2.publish_rate_hz <= 0.0) {
      throw std::runtime_error("ros2.publish_rate_hz must be positive.");
    }

    EnsureWritableRosHome();

    try {
      RosStartupDebugLog("starting ROS bridge child");
      if (!rclcpp::ok()) {
        int argc = 1;
        char program_name[] = "ausim_ros_bridge";
        char* argv[] = {program_name, nullptr};
        rclcpp::init(argc, argv);
        owns_rclcpp_runtime_ = true;
      }

      RosStartupDebugLog("creating rclcpp::Node");
      node_ = std::make_shared<rclcpp::Node>(config_.ros2.node_name);

      const std::string odom_frame = ResolveFrameId(config_, config_.frames.odom);
      const std::string base_frame = ResolveFrameId(config_, config_.frames.base);
      const std::string imu_frame = ResolveFrameId(config_, config_.frames.imu);
      const std::string cmd_vel_topic = ResolveTopicName(config_, config_.interfaces.cmd_vel_topic);
      const std::string joy_cmd_vel_topic =
          config_.interfaces.joy_cmd_vel_topic.empty() ? "" : ResolveTopicName(config_, config_.interfaces.joy_cmd_vel_topic);

      publishers_ = BuildPublishers(node_, config_, odom_frame, base_frame, imu_frame);

      // Publish one-shot static TF for each sensor that declares a transform.
      bool any_sensor_tf = false;
      for (const SensorConfig& s : config_.sensors) {
        if (s.enabled && s.publish_tf) {
          any_sensor_tf = true;
          break;
        }
      }
      if (any_sensor_tf) {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
        for (const SensorConfig& sensor : config_.sensors) {
          if (!sensor.enabled || !sensor.publish_tf) {
            continue;
          }
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = rclcpp::Time(0);
          t.header.frame_id = base_frame;
          t.child_frame_id = ResolveFrameId(config_, sensor.frame_id);
          t.transform.translation.x = sensor.transform.translation[0];
          t.transform.translation.y = sensor.transform.translation[1];
          t.transform.translation.z = sensor.transform.translation[2];
          t.transform.rotation.x = sensor.transform.rotation[0];
          t.transform.rotation.y = sensor.transform.rotation[1];
          t.transform.rotation.z = sensor.transform.rotation[2];
          t.transform.rotation.w = sensor.transform.rotation[3];
          static_transforms.push_back(t);
        }
        static_tf_broadcaster_->sendTransform(static_transforms);
      }

      subscribers_.push_back(
          std::make_unique<CmdVelCommandSubscriber>(node_, cmd_vel_topic, [this](const data::CmdVelData& message) { PublishCommand(message); }));
      if (!joy_cmd_vel_topic.empty() && joy_cmd_vel_topic != cmd_vel_topic) {
        subscribers_.push_back(
            std::make_unique<CmdVelCommandSubscriber>(node_, joy_cmd_vel_topic, [this](const data::CmdVelData& message) {
              PublishCommand(message);
            }));
      }

      for (const JoyActionServiceConfig& action_service : config_.interfaces.joy_action_services) {
        joy_action_services_.push_back(node_->create_service<TriggerService>(
            ResolveServiceName(config_, action_service.service),
            [this, event_name = action_service.event](const TriggerService::Request::SharedPtr,
                                                      TriggerService::Response::SharedPtr response) {
              ExecuteDiscreteCommand(event_name, response.get());
            }));
      }

      if (!config_.interfaces.robot_mode_topic.empty()) {
        legacy_robot_mode_publisher_ =
            node_->create_publisher<std_msgs::msg::String>(ResolveTopicName(config_, config_.interfaces.robot_mode_topic), 10);
      }

      if (config_.dynamic_obstacle.enabled && config_.dynamic_obstacle.publish.enabled) {
        dyn_obstacle_publisher_ = std::make_unique<DynObstacleDataPublisher>(
            node_, ResolveTopicName(config_, config_.dynamic_obstacle.publish.topic), config_.dynamic_obstacle.publish.frame_id);
      }

      // Sensor publishers driven by sensors[] config.
      // Add a new else-if branch here when implementing a new sensor type.
      for (const SensorConfig& sensor : config_.sensors) {
        if (!sensor.enabled) {
          continue;
        }
        const std::string topic = ResolveTopicName(config_, sensor.topic);
        const std::string frame = ResolveFrameId(config_, sensor.frame_id);

        if (sensor.type == "imu") {
          publishers_.push_back(std::make_unique<ImuDataPublisher>(node_, topic, frame));
        } else if (sensor.type == "camera") {
          continue;
        } else if (sensor.type == "lidar") {
          if (lidar_fd_ >= 0) {
            lidar_publisher_ = std::make_unique<LidarDataPublisher>(node_, topic, frame);
          } else {
            RCLCPP_WARN(node_->get_logger(), "Lidar sensor '%s' configured but no --lidar-fd provided.", sensor.name.c_str());
          }
        } else {
          RCLCPP_WARN(node_->get_logger(), "Sensor '%s' of type '%s' is not implemented yet.", sensor.name.c_str(), sensor.type.c_str());
        }
      }

      for (const CameraStreamConfig& stream : BuildCameraStreamConfigs(config_.sensors)) {
        image_publishers_.push_back(
            std::make_unique<ImageDataPublisher>(node_, ResolveTopicName(config_, stream.topic), ResolveFrameId(config_, stream.frame_id)));
        latest_camera_frames_.push_back(nullptr);
        last_published_camera_sequences_.push_back(0);
        camera_frame_published_.push_back(false);
      }

      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);

      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / config_.ros2.publish_rate_hz));
      telemetry_timer_ = node_->create_wall_timer(period, [this]() {
        PublishTelemetry();
        PublishCameraFrames();
        PublishLidar();
      });

      telemetry_thread_ = std::thread([this]() { TelemetryLoop(); });
      if (!image_publishers_.empty()) {
        image_thread_ = std::thread([this]() { ImageLoop(); });
      }
      if (lidar_publisher_ && lidar_fd_ >= 0) {
        lidar_thread_ = std::thread([this]() { LidarLoop(); });
      }
      started_ = true;
      RosStartupDebugLog("ROS bridge child started");
    } catch (const std::exception& error) {
      Stop();
      throw std::runtime_error("Failed to start ROS bridge child (RMW_IMPLEMENTATION=" + ActiveRmwImplementation() + "): " + error.what());
    }
  }

  void Stop() {
    if (!started_ && !owns_rclcpp_runtime_) {
      ipc::ShutdownAndClose(&telemetry_fd_);
      ipc::ShutdownAndClose(&command_fd_);
      ipc::ShutdownAndClose(&discrete_command_fd_);
      ipc::ShutdownAndClose(&image_fd_);
      ipc::ShutdownAndClose(&lidar_fd_);
      return;
    }

    stop_requested_.store(true);
    if (executor_) {
      executor_->cancel();
    }

    ipc::ShutdownAndClose(&telemetry_fd_);
    ipc::ShutdownAndClose(&command_fd_);
    ipc::ShutdownAndClose(&discrete_command_fd_);
    ipc::ShutdownAndClose(&image_fd_);
    ipc::ShutdownAndClose(&lidar_fd_);

    if (telemetry_thread_.joinable()) {
      telemetry_thread_.join();
    }
    if (image_thread_.joinable()) {
      image_thread_.join();
    }
    if (lidar_thread_.joinable()) {
      lidar_thread_.join();
    }

    telemetry_timer_.reset();
    joy_action_services_.clear();
    legacy_robot_mode_publisher_.reset();
    subscribers_.clear();
    publishers_.clear();
    image_publishers_.clear();
    lidar_publisher_.reset();
    dyn_obstacle_publisher_.reset();
    static_tf_broadcaster_.reset();
    latest_camera_frames_.clear();
    last_published_camera_sequences_.clear();
    camera_frame_published_.clear();

    if (executor_ && node_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();

    if (owns_rclcpp_runtime_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }

    owns_rclcpp_runtime_ = false;
    started_ = false;
  }

  void TelemetryLoop() {
    while (!stop_requested_.load()) {
      std::vector<std::uint8_t> message;
      switch (ipc::ReceivePacketBytes(telemetry_fd_, &message, kMaxBridgeMessageSize)) {
        case ipc::PacketReceiveStatus::kPacket: {
          if (message.empty()) {
            break;
          }
          const std::optional<ipc::BridgeMessageType> type = ParseBridgeMessageType(message[0]);
          if (!type.has_value()) {
            if (!stop_requested_.load()) {
              std::cerr << "ausim_ros_bridge warning: received unknown bridge message type\n";
            }
            break;
          }

          const std::uint8_t* payload = message.data() + 1;
          const std::size_t payload_size = message.size() - 1;
          if (*type == ipc::BridgeMessageType::kTelemetry) {
            if (payload_size != sizeof(ipc::TelemetryPacket)) {
              if (!stop_requested_.load()) {
                std::cerr << "ausim_ros_bridge warning: malformed telemetry packet size\n";
              }
              break;
            }

            ipc::TelemetryPacket packet;
            std::memcpy(&packet, payload, sizeof(packet));
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            latest_telemetry_ = packet;
          } else if (*type == ipc::BridgeMessageType::kDynamicObstacles) {
            if (!dyn_obstacle_publisher_) {
              break;
            }

            DynamicObstaclesSnapshot snapshot;
            if (!converts::FromDynObstaclePacketBytes(payload, payload_size, snapshot)) {
              if (!stop_requested_.load()) {
                std::cerr << "ausim_ros_bridge warning: malformed dynamic obstacle packet\n";
              }
              break;
            }
            dyn_obstacle_publisher_->Publish(snapshot);
          }
          break;
        }
        case ipc::PacketReceiveStatus::kClosed:
          RequestStop();
          return;
        case ipc::PacketReceiveStatus::kWouldBlock:
          continue;
        case ipc::PacketReceiveStatus::kError:
          if (!stop_requested_.load()) {
            std::cerr << "ausim_ros_bridge warning: telemetry socket receive failed\n";
          }
          RequestStop();
          return;
      }
    }
  }

  void PublishCommand(const data::CmdVelData& message) {
    const ipc::VelocityCommandPacket packet = converts::ToVelocityCommandPacket(message);
    ipc::SendPacket(command_fd_, packet, true);
  }

  // Builds an outgoing DiscreteCommand from an event name. kind is derived for
  // fast sim-side dispatch (reset vs generic).
  DiscreteCommand MakeDiscreteCommand(const std::string& event_name, std::uint64_t sequence) const {
    DiscreteCommand command;
    command.event_name = event_name;
    command.kind = ClassifyDiscreteEvent(event_name);
    command.sequence = sequence;
    command.received_time = std::chrono::steady_clock::now();
    return command;
  }

  void PublishDiscreteCommand(const std::string& event_name) {
    if (event_name.empty()) {
      return;
    }
    const std::uint64_t sequence = next_discrete_command_sequence_++;
    const DiscreteCommand command = MakeDiscreteCommand(event_name, sequence);
    const ipc::DiscreteCommandPacket packet = converts::ToDiscreteCommandPacket(command);
    if (!ipc::SendPacket(discrete_command_fd_, packet, true)) {
      RCLCPP_WARN(node_->get_logger(), "failed to send teleop event '%s' to simulator", event_name.c_str());
    }
  }

  void ExecuteDiscreteCommand(const std::string& event_name, TriggerService::Response* response) {
    if (response == nullptr) {
      return;
    }

    const std::uint64_t sequence = next_discrete_command_sequence_++;
    const DiscreteCommand command = MakeDiscreteCommand(event_name, sequence);
    const ipc::DiscreteCommandPacket packet = converts::ToDiscreteCommandPacket(command);
    if (!ipc::SendPacket(discrete_command_fd_, packet, true)) {
      response->success = false;
      response->message = "failed to send " + event_name + " command to simulator";
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() + kDiscreteCommandAckTimeout;
    while (!stop_requested_.load() && std::chrono::steady_clock::now() < deadline) {
      std::optional<ipc::TelemetryPacket> telemetry;
      {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry = latest_telemetry_;
      }

      if (telemetry.has_value() && telemetry->last_discrete_command_sequence == sequence) {
        const auto status = static_cast<DiscreteCommandAckStatus>(telemetry->last_discrete_command_status);
        response->success = status == DiscreteCommandAckStatus::kSuccess;
        response->message = AckStatusMessage(status, event_name);
        return;
      }

      std::this_thread::sleep_for(kDiscreteCommandAckPollPeriod);
    }

    response->success = false;
    response->message = AckStatusMessage(DiscreteCommandAckStatus::kNone, event_name);
  }

  void ImageLoop() {
    while (!stop_requested_.load()) {
      ipc::CameraImageMetadataPacket metadata;
      if (!ipc::ReadFully(image_fd_, &metadata, sizeof(metadata))) {
        RequestStop();
        return;
      }

      if (metadata.sensor_index >= latest_camera_frames_.size()) {
        std::vector<std::uint8_t> discarded(metadata.data_size);
        if (!discarded.empty() && !ipc::ReadFully(image_fd_, discarded.data(), discarded.size())) {
          RequestStop();
          return;
        }
        if (!stop_requested_.load()) {
          std::cerr << "ausim_ros_bridge warning: received image for unknown sensor index " << metadata.sensor_index << '\n';
        }
        continue;
      }

      std::vector<std::uint8_t> pixels(metadata.data_size);
      if (!pixels.empty() && !ipc::ReadFully(image_fd_, pixels.data(), pixels.size())) {
        RequestStop();
        return;
      }
      if (!ValidateImageMetadata(metadata)) {
        if (!stop_requested_.load()) {
          std::cerr << "ausim_ros_bridge warning: received malformed image frame\n";
        }
        continue;
      }

      auto frame = std::make_shared<CameraFrame>(converts::ToCameraFrame(metadata, std::move(pixels)));
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_camera_frames_[metadata.sensor_index] = std::move(frame);
    }
  }

  void PublishTelemetry() {
    std::optional<ipc::TelemetryPacket> packet;
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      packet = latest_telemetry_;
    }
    if (!packet.has_value()) {
      return;
    }
    for (auto& pub : publishers_) {
      pub->Publish(*packet);
    }
    if (legacy_robot_mode_publisher_) {
      std_msgs::msg::String mode_message;
      mode_message.data = std::string("{\"top_state\":\"") +
                          RobotTopLevelStateName(static_cast<RobotTopLevelState>(packet->robot_top_level_state)) +
                          "\",\"sub_state\":\"" + packet->robot_mode_sub_state.data() + "\",\"accepts_motion\":" +
                          (packet->robot_mode_accepts_motion != 0 ? "true" : "false") + "}";
      legacy_robot_mode_publisher_->publish(mode_message);
    }
  }

  void PublishCameraFrames() {
    std::vector<std::shared_ptr<CameraFrame>> frames;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      frames = latest_camera_frames_;
    }

    for (std::size_t i = 0; i < image_publishers_.size(); ++i) {
      const std::shared_ptr<CameraFrame>& frame = frames[i];
      if (frame == nullptr) {
        continue;
      }
      if (camera_frame_published_[i] && frame->sequence == last_published_camera_sequences_[i]) {
        continue;
      }
      image_publishers_[i]->Publish(*frame);
      last_published_camera_sequences_[i] = frame->sequence;
      camera_frame_published_[i] = true;
    }
  }

  void LidarLoop() {
    while (!stop_requested_.load()) {
      ipc::LidarPacket packet;
      switch (ipc::ReceivePacket(lidar_fd_, &packet)) {
        case ipc::PacketReceiveStatus::kPacket: {
          std::lock_guard<std::mutex> lock(lidar_mutex_);
          latest_lidar_ = packet;
          break;
        }
        case ipc::PacketReceiveStatus::kClosed:
          RequestStop();
          return;
        case ipc::PacketReceiveStatus::kWouldBlock:
          continue;
        case ipc::PacketReceiveStatus::kError:
          if (!stop_requested_.load()) {
            std::cerr << "ausim_ros_bridge warning: lidar socket receive failed\n";
          }
          RequestStop();
          return;
      }
    }
  }

  void PublishLidar() {
    if (!lidar_publisher_) {
      return;
    }
    std::optional<ipc::LidarPacket> packet;
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      packet = latest_lidar_;
    }
    if (packet.has_value()) {
      lidar_publisher_->Publish(*packet);
      latest_lidar_.reset();  // Publish each scan exactly once.
    }
  }

  void RequestStop() {
    stop_requested_.store(true);
    if (executor_) {
      executor_->cancel();
    }
  }

  RosBridgeConfig config_;
  int telemetry_fd_ = -1;
  int command_fd_ = -1;
  int discrete_command_fd_ = -1;
  int image_fd_ = -1;
  int lidar_fd_ = -1;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::unique_ptr<ITelemetryPublisher>> publishers_;
  std::vector<std::unique_ptr<ImageDataPublisher>> image_publishers_;
  std::unique_ptr<LidarDataPublisher> lidar_publisher_;
  std::unique_ptr<IDynObstaclePublisher> dyn_obstacle_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::vector<std::unique_ptr<ICommandSubscriber>> subscribers_;
  std::vector<rclcpp::Service<TriggerService>::SharedPtr> joy_action_services_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_robot_mode_publisher_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  std::thread telemetry_thread_;
  std::thread image_thread_;
  std::thread lidar_thread_;
  std::mutex telemetry_mutex_;
  std::mutex image_mutex_;
  std::mutex lidar_mutex_;
  std::optional<ipc::TelemetryPacket> latest_telemetry_;
  std::optional<ipc::LidarPacket> latest_lidar_;
  std::vector<std::shared_ptr<CameraFrame>> latest_camera_frames_;
  std::vector<std::uint32_t> last_published_camera_sequences_;
  std::vector<bool> camera_frame_published_;
  std::atomic<std::uint64_t> next_discrete_command_sequence_{1};
  std::atomic_bool stop_requested_ = false;
  bool owns_rclcpp_runtime_ = false;
  bool started_ = false;
};

}  // namespace

int RunRosBridgeProcess(const QuadrotorConfig& config, int telemetry_fd, int command_fd, int discrete_command_fd, int image_fd, int lidar_fd) {
  RosBridgeProcess process(BuildRosBridgeConfig(config), telemetry_fd, command_fd, discrete_command_fd, image_fd, lidar_fd);
  return process.Run();
}

}  // namespace ausim
