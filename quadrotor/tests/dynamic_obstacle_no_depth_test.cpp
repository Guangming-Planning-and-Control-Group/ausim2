#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>

#include "config/quadrotor_config.hpp"
#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

fs::path ResolveRepoRoot() {
  fs::path current = fs::current_path();
  for (int i = 0; i < 4; ++i) {
    if (fs::exists(current / "quadrotor" / "cfg" / "sim_config.yaml")) {
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
         << "  x_min: 0.2\n"
         << "  x_max: 5.2\n"
         << "  y_min: -2.0\n"
         << "  y_max: 2.0\n"
         << "  z_min: 0.0\n"
         << "  z_max: 1.5\n"
         << "obstacle_count: 10\n"
         << "min_speed: 0.3\n"
         << "max_speed: 0.3\n"
         << "update_threads: 1\n"
         << "parallel_threshold: 16\n";
  Expect(output.good(), "failed to write quadrotor dynamic obstacle test config");
}

void GenerateSceneWithObstacles(
    const fs::path& repo_root,
    const fs::path& obstacle_config_path,
    const fs::path& output_scene_path) {
  const fs::path script_path =
      repo_root / "third_party" / "dynamic_obs_generator" / "generate_scene_obstacles.py";
  const fs::path input_scene_path = repo_root / "assets" / "crazyfile" / "scene.xml";

  std::ostringstream command;
  command << "python3 \"" << script_path.string() << "\""
          << " --input \"" << input_scene_path.string() << "\""
          << " --obstacle-config \"" << obstacle_config_path.string() << "\""
          << " --output \"" << output_scene_path.string() << "\""
          << " --enable-dynamic-obstacles > /dev/null 2>&1";

  const int exit_code = std::system(command.str().c_str());
  Expect(exit_code == 0, "failed to generate quadrotor test scene with dynamic obstacles");
}

}  // namespace

int main() {
  const fs::path repo_root = ResolveRepoRoot();
  Expect(!repo_root.empty(), "failed to locate repo root for dynamic_obstacle_no_depth_test");

  const fs::path obstacle_config_path =
      fs::temp_directory_path() / "quadrotor_dynamic_obstacle_no_depth_test.yaml";
  const fs::path scene_path =
      repo_root / "assets" / "crazyfile" / "scene.dynamic_obstacles.no_depth_test.xml";
  WriteObstacleConfig(obstacle_config_path);
  GenerateSceneWithObstacles(repo_root, obstacle_config_path, scene_path);

  quadrotor::QuadrotorConfig config =
      quadrotor::LoadConfigFromYaml((repo_root / "quadrotor" / "cfg" / "sim_config.yaml").string(), "");
  config.viewer.enabled = false;
  config.viewer.fallback_to_headless = true;
  config.model.scene_xml = scene_path;
  config.dynamic_obstacle.enabled = true;
  config.dynamic_obstacle.config_path = obstacle_config_path.string();
  for (auto& sensor : config.sensors) {
    sensor.depth.enabled = false;
  }

  quadrotor::QuadrotorSim sim(config);
  sim.LoadModel();

  const mjModel* model = sim.model();
  const mjData* data = sim.data();
  Expect(model != nullptr, "expected quadrotor model to load");
  Expect(data != nullptr, "expected quadrotor mjData to be allocated");

  const int body_id = mj_name2id(model, mjOBJ_BODY, "dynamic_obs_0");
  Expect(body_id >= 0, "expected generated scene to contain dynamic_obs_0 mocap body");
  const int mocap_id = model->body_mocapid[body_id];
  Expect(mocap_id >= 0, "expected dynamic_obs_0 to resolve to a mocap id");

  const int mocap_adr = mocap_id * 3;
  const double start_x = data->mocap_pos[mocap_adr + 0];
  const double start_y = data->mocap_pos[mocap_adr + 1];
  const double start_z = data->mocap_pos[mocap_adr + 2];

  for (int i = 0; i < 20; ++i) {
    sim.Step();
  }

  const double end_x = data->mocap_pos[mocap_adr + 0];
  const double end_y = data->mocap_pos[mocap_adr + 1];
  const double end_z = data->mocap_pos[mocap_adr + 2];

  Expect(start_x != end_x || start_y != end_y || start_z != end_z,
         "expected collidable mocap obstacle to move even when no depth stream is enabled");

  fs::remove(obstacle_config_path);
  fs::remove(scene_path);
  return 0;
}
