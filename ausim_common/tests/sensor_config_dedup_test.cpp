#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "config/quadrotor_config.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  namespace fs = std::filesystem;

  const fs::path config_path = fs::temp_directory_path() / "sensor_config_dedup_test.yaml";
  std::ofstream output(config_path);
  output << R"yaml(
sensors:
  - name: front_camera
    type: camera
    enabled: true
    frame_id: camera_link
    topic: camera/image_raw
    rate_hz: 30.0
    depth:
      enabled: true
      topic: camera/depth/image_raw
  - name: lidar16
    type: lidar
    enabled: true
    frame_id: lidar_link
    topic: lidar/points
    rate_hz: 10.0
)yaml";
  output.close();

  const ausim::QuadrotorConfig config = ausim::LoadConfigFromYaml(config_path.string());
  Expect(config.sensors.size() == 2, "expected two sensors to be loaded");
  Expect(config.sensors[0].name == "front_camera", "expected camera sensor name");
  Expect(config.sensors[0].enabled, "expected camera sensor to stay enabled");
  Expect(config.sensors[0].depth.enabled, "expected depth stream to stay enabled");
  Expect(config.sensors[1].name == "lidar16", "expected lidar sensor name");

  const std::vector<ausim::CameraStreamConfig> streams = ausim::BuildCameraStreamConfigs(config.sensors);
  Expect(streams.size() == 2, "expected one color and one depth camera stream");
  Expect(streams[0].name == "front_camera", "expected color stream name to match config sensor name");
  Expect(streams[0].channel_name == "front_camera", "expected color stream channel to use logical sensor name");
  Expect(streams[0].camera_name.empty(), "expected camera source binding to be deferred to the MJCF model");
  Expect(streams[0].topic == "camera/image_raw", "expected color stream topic");
  Expect(streams[1].name == "front_camera_depth", "expected derived depth stream name");
  Expect(streams[1].channel_name == "front_camera_depth", "expected depth stream channel to use logical sensor name");
  Expect(streams[1].sensor_name.empty(), "expected depth plugin binding to be deferred to the MJCF model");
  Expect(streams[1].topic == "camera/depth/image_raw", "expected depth stream topic override");

  fs::remove(config_path);
  return 0;
}
