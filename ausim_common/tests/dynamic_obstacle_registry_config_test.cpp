#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void WriteFile(const fs::path& path, const std::string& contents) {
  std::ofstream output(path);
  output << contents;
  output.close();
  Expect(output.good(), "failed to write test file: " + path.string());
}

void SetRegistryOverride(const fs::path& path) {
  Expect(setenv("AUSIM_MODEL_REGISTRY_OVERRIDE", path.string().c_str(), 1) == 0, "failed to set AUSIM_MODEL_REGISTRY_OVERRIDE");
}

void ClearRegistryOverride() { unsetenv("AUSIM_MODEL_REGISTRY_OVERRIDE"); }

void TestRegistryFallbackLoadsDynamicObstacleConfig() {
  const fs::path temp_dir = fs::temp_directory_path() / "dynamic_obstacle_registry_config_test";
  fs::create_directories(temp_dir);

  const fs::path obstacle_config_path = temp_dir / "obstacle.yaml";
  const fs::path sim_config_path = temp_dir / "sim_config.yaml";
  const fs::path robot_config_path = temp_dir / "robot_config.yaml";
  const fs::path registry_path = temp_dir / "ground_vehicle_registry.yaml";

  WriteFile(obstacle_config_path,
            R"yaml(
publish:
  enabled: true
  topic: /dyn_obstacle
  frame_id: world
  rate_hz: 12.5
)yaml");
  WriteFile(sim_config_path, "simulation:\n  dt: 0.001\n");
  WriteFile(robot_config_path, "model:\n  scene_xml: /tmp/scene.xml\n");
  WriteFile(registry_path,
            "models:\n"
            "  - id: scout_v2\n"
            "    sim_config: " +
                sim_config_path.string() +
                "\n"
                "    robot_config: " +
                robot_config_path.string() +
                "\n"
                "    scene_xml: /tmp/scene.xml\n"
                "    dynamic_obstacle:\n"
                "      enabled: true\n"
                "      config_path: " +
                obstacle_config_path.string() + "\n");

  SetRegistryOverride(registry_path);
  const ausim::QuadrotorConfig config = ausim::LoadConfigFromYaml(sim_config_path.string(), robot_config_path.string());
  ClearRegistryOverride();

  Expect(config.dynamic_obstacle.enabled, "expected dynamic_obstacle.enabled to fall back from registry");
  Expect(config.dynamic_obstacle.config_path == fs::absolute(obstacle_config_path).string(),
         "expected dynamic_obstacle.config_path to resolve from registry");
  Expect(config.dynamic_obstacle.publish.enabled, "expected publish.enabled to load from obstacle config");
  Expect(config.dynamic_obstacle.publish.topic == "/dyn_obstacle", "expected publish.topic from obstacle config");
  Expect(config.dynamic_obstacle.publish.frame_id == "world", "expected publish.frame_id from obstacle config");
  Expect(std::abs(config.dynamic_obstacle.publish.rate_hz - 12.5) < 1e-9, "expected publish.rate_hz from obstacle config");

  fs::remove_all(temp_dir);
}

void TestExplicitConfigOverridesRegistryFallback() {
  const fs::path temp_dir = fs::temp_directory_path() / "dynamic_obstacle_registry_override_test";
  fs::create_directories(temp_dir);

  const fs::path obstacle_config_path = temp_dir / "obstacle.yaml";
  const fs::path explicit_obstacle_config_path = temp_dir / "explicit_obstacle.yaml";
  const fs::path sim_config_path = temp_dir / "sim_config.yaml";
  const fs::path robot_config_path = temp_dir / "robot_config.yaml";
  const fs::path registry_path = temp_dir / "ground_vehicle_registry.yaml";

  WriteFile(obstacle_config_path,
            R"yaml(
publish:
  enabled: true
  topic: /from_registry
  frame_id: registry
  rate_hz: 8.0
)yaml");
  WriteFile(explicit_obstacle_config_path,
            R"yaml(
publish:
  enabled: true
  topic: /from_sim
  frame_id: sim
  rate_hz: 20.0
)yaml");
  WriteFile(sim_config_path,
            "dynamic_obstacle:\n"
            "  enabled: false\n"
            "  config_path: " +
                explicit_obstacle_config_path.string() + "\n");
  WriteFile(robot_config_path, "model:\n  scene_xml: /tmp/scene.xml\n");
  WriteFile(registry_path,
            "models:\n"
            "  - id: scout_v2\n"
            "    sim_config: " +
                sim_config_path.string() +
                "\n"
                "    robot_config: " +
                robot_config_path.string() +
                "\n"
                "    scene_xml: /tmp/scene.xml\n"
                "    dynamic_obstacle:\n"
                "      enabled: true\n"
                "      config_path: " +
                obstacle_config_path.string() + "\n");

  SetRegistryOverride(registry_path);
  const ausim::QuadrotorConfig config = ausim::LoadConfigFromYaml(sim_config_path.string(), robot_config_path.string());
  ClearRegistryOverride();

  Expect(!config.dynamic_obstacle.enabled, "expected explicit sim config enabled=false to override registry");
  Expect(config.dynamic_obstacle.config_path == fs::absolute(explicit_obstacle_config_path).string(),
         "expected explicit sim config path to override registry");
  Expect(config.dynamic_obstacle.publish.topic == "/from_sim", "expected publish config to load from explicit obstacle yaml");

  fs::remove_all(temp_dir);
}

}  // namespace

int main() {
  TestRegistryFallbackLoadsDynamicObstacleConfig();
  TestExplicitConfigOverridesRegistryFallback();
  return 0;
}
