#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/scout_config.hpp"
#include "runtime/data_board_interface.hpp"
#include "sim/scout_sim.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class ScopedCleanup {
 public:
  explicit ScopedCleanup(std::vector<fs::path> paths) : paths_(std::move(paths)) {}

  ~ScopedCleanup() {
    for (const fs::path& path : paths_) {
      std::error_code error;
      fs::remove(path, error);
    }
  }

 private:
  std::vector<fs::path> paths_;
};

fs::path ResolveRepoRoot() {
  fs::path current = fs::current_path();
  for (int i = 0; i < 4; ++i) {
    if (fs::exists(current / "ground_vehicle" / "cfg" / "sim_config.yaml")) {
      return current;
    }
    if (!current.has_parent_path()) {
      break;
    }
    current = current.parent_path();
  }
  return {};
}

void WriteObstacleConfig(const fs::path& path) {
  std::ofstream output(path);
  output << "random_seed: 1\n"
         << "dynamic: true\n"
         << "collision_enabled: true\n"
         << "debug: false\n"
         << "mode: \"2d\"\n"
         << "range:\n"
         << "  x_min: 0.5\n"
         << "  x_max: 20.5\n"
         << "  y_min: -5.0\n"
         << "  y_max: 5.0\n"
         << "  z_min: 0.0\n"
         << "  z_max: 1.5\n"
         << "obstacle_count: 20\n"
         << "min_speed: 0.3\n"
         << "max_speed: 0.3\n"
         << "update_threads: 1\n"
         << "parallel_threshold: 16\n"
         << "publish:\n"
         << "  enabled: true\n"
         << "  topic: /dyn_obstacle\n"
         << "  frame_id: world\n"
         << "  rate_hz: 20.0\n";
  Expect(output.good(), "failed to write viewer obstacle test config");
}

void GenerateSceneWithObstacles(const fs::path& repo_root, const fs::path& obstacle_config_path, const fs::path& output_scene_path) {
  const fs::path script_path = repo_root / "third_party" / "dynamic_obs_generator" / "generate_scene_obstacles.py";
  const fs::path input_scene_path = repo_root / "assets" / "scout_v2" / "scene.xml";

  std::ostringstream command;
  command << "python3 \"" << script_path.string() << "\""
          << " --input \"" << input_scene_path.string() << "\""
          << " --obstacle-config \"" << obstacle_config_path.string() << "\""
          << " --output \"" << output_scene_path.string() << "\""
          << " --enable-dynamic-obstacles > /dev/null 2>&1";

  const int exit_code = std::system(command.str().c_str());
  Expect(exit_code == 0, "failed to generate scout viewer test scene with dynamic obstacles");
}

ground_vehicle::ScoutConfig LoadViewerConfig(const fs::path& repo_root, const fs::path& scene_path, const fs::path& obstacle_config_path) {
  ground_vehicle::ScoutConfig config =
      ground_vehicle::LoadScoutConfigFromYaml((repo_root / "ground_vehicle" / "cfg" / "sim_config.yaml").string(), "");
  config.common.viewer.enabled = true;
  config.common.viewer.fallback_to_headless = false;
  config.common.simulation.duration = 0.2;
  config.common.model.scene_xml = scene_path;
  config.common.dynamic_obstacle.enabled = true;
  config.common.dynamic_obstacle.config_path = obstacle_config_path.string();
  config.common.dynamic_obstacle.publish.enabled = true;
  config.common.dynamic_obstacle.publish.topic = "/dyn_obstacle";
  config.common.dynamic_obstacle.publish.frame_id = "world";
  config.common.dynamic_obstacle.publish.rate_hz = 20.0;
  return config;
}

void ExpectPublishedSnapshot(const std::optional<ausim::DynamicObstaclesSnapshot>& snapshot, double min_sim_time) {
  Expect(snapshot.has_value(), "expected viewer path to publish a dynamic obstacle snapshot");
  Expect(!snapshot->entries.empty(), "expected viewer snapshot to contain obstacle entries");
  Expect(snapshot->frame_id == "world", "expected viewer snapshot frame_id to be 'world'");
  Expect(snapshot->sim_time >= min_sim_time, "expected viewer snapshot sim_time to meet minimum");
}

void ExpectViewerRunningStepPublishes(const fs::path& repo_root, const fs::path& scene_path, const fs::path& obstacle_config_path) {
  ground_vehicle::ScoutSim sim(LoadViewerConfig(repo_root, scene_path, obstacle_config_path));
  sim.LoadModel();
  sim.TestViewerStepOnce();
  ExpectPublishedSnapshot(ausim::ReadDynamicObstaclesSnapshot(), 1e-9);
}

void ExpectViewerPausedTickPublishes(const fs::path& repo_root, const fs::path& scene_path, const fs::path& obstacle_config_path) {
  ground_vehicle::ScoutSim sim(LoadViewerConfig(repo_root, scene_path, obstacle_config_path));
  sim.LoadModel();
  sim.TestViewerPauseTick(true);
  ExpectPublishedSnapshot(ausim::ReadDynamicObstaclesSnapshot(), 0.0);
}

}  // namespace

int main() {
  try {
    const fs::path repo_root = ResolveRepoRoot();
    Expect(!repo_root.empty(), "failed to locate repo root for scout viewer publish test");

    const fs::path obstacle_config_path = fs::temp_directory_path() / "scout_dynamic_obstacle_viewer_publish_test.yaml";
    const fs::path scene_path = repo_root / "assets" / "scout_v2" / "scene.dynamic_obstacles.viewer_publish_test.xml";
    ScopedCleanup cleanup({obstacle_config_path, scene_path});

    WriteObstacleConfig(obstacle_config_path);
    GenerateSceneWithObstacles(repo_root, obstacle_config_path, scene_path);

    ExpectViewerRunningStepPublishes(repo_root, scene_path, obstacle_config_path);
    ExpectViewerPausedTickPublishes(repo_root, scene_path, obstacle_config_path);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
