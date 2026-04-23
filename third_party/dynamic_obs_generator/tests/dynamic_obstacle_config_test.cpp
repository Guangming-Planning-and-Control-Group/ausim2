#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "obstacle_config.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void WriteTextFile(const fs::path& path, const std::string& content) {
  std::ofstream out(path);
  Expect(out.good(), "failed to open temp obstacle config for writing");
  out << content;
  Expect(out.good(), "failed to write temp obstacle config");
}

void ExpectThrowsInvalidConfig(const fs::path& path, const std::string& expected_substring) {
  try {
    (void)dynamic_obstacle::LoadConfigFromYaml(path.string());
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    Expect(message.find(expected_substring) != std::string::npos,
           "expected error message to contain '" + expected_substring + "', got: " + message);
    return;
  }

  std::cerr << "expected LoadConfigFromYaml to reject invalid publish config\n";
  std::exit(1);
}

void TestPublishRateMustBePositiveWhenEnabled() {
  const fs::path path = fs::temp_directory_path() / "dynamic_obstacle_publish_invalid.yaml";
  WriteTextFile(path, R"yaml(
random_seed: 1
dynamic: true
debug: false
mode: "2d"
radius: 0.3
box_size: 0.5
range:
  x_min: -1.0
  x_max: 1.0
  y_min: -1.0
  y_max: 1.0
  z_min: 0.0
  z_max: 1.0
obstacle_count: 2
min_speed: 0.0
max_speed: 0.5
publish:
  enabled: true
  topic: /dyn_obstacle
  frame_id: world
  rate_hz: 0.0
)yaml");

  ExpectThrowsInvalidConfig(path, "publish_rate_hz");
  fs::remove(path);
}

}  // namespace

int main() {
  TestPublishRateMustBePositiveWhenEnabled();
  return 0;
}
