#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>

#include "config/scout_config.hpp"
#include "sim/scout_sim.hpp"

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

int FindDynamicObstacleGeom(const mjModel* model) {
  if (model == nullptr) {
    return -1;
  }
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
    if (name != nullptr && std::string(name).rfind("dynamic_obs_", 0) == 0) {
      return geom_id;
    }
  }
  return -1;
}

void WriteObstacleConfig(const fs::path& path, bool collision_enabled) {
  std::ofstream output(path);
  output << "random_seed: 1\n"
         << "dynamic: true\n"
         << "collision_enabled: " << (collision_enabled ? "true" : "false") << "\n"
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
         << "parallel_threshold: 16\n";
  Expect(output.good(), "failed to write obstacle test config");
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
  Expect(exit_code == 0, "failed to generate test scene with dynamic obstacles");
}

void ExpectDirectGeomPathMovesGeom(const fs::path& repo_root) {
  const fs::path obstacle_config_path = fs::temp_directory_path() / "scout_dynamic_obstacle_direct_test.yaml";
  const fs::path scene_path = repo_root / "assets" / "scout_v2" / "scene.dynamic_obstacles.direct_test.xml";
  WriteObstacleConfig(obstacle_config_path, false);
  GenerateSceneWithObstacles(repo_root, obstacle_config_path, scene_path);

  ground_vehicle::ScoutConfig config =
      ground_vehicle::LoadScoutConfigFromYaml((repo_root / "ground_vehicle" / "cfg" / "sim_config.yaml").string(), "");
  config.common.viewer.enabled = false;
  config.common.viewer.fallback_to_headless = true;
  config.common.model.scene_xml = scene_path;
  config.common.dynamic_obstacle.enabled = true;
  config.common.dynamic_obstacle.config_path = obstacle_config_path.string();

  ground_vehicle::ScoutSim sim(config);
  sim.LoadModel();
  const mjModel* model = sim.model();
  Expect(model != nullptr, "expected Scout model to load for direct obstacle path");
  const int geom_id = FindDynamicObstacleGeom(model);
  Expect(geom_id >= 0, "expected generated scene to contain dynamic_obs_* geoms");

  const int adr = geom_id * 3;
  const double start_x = model->geom_pos[adr + 0];
  const double start_y = model->geom_pos[adr + 1];
  const double start_z = model->geom_pos[adr + 2];

  for (int i = 0; i < 20; ++i) {
    sim.Step();
  }

  const double end_x = model->geom_pos[adr + 0];
  const double end_y = model->geom_pos[adr + 1];
  const double end_z = model->geom_pos[adr + 2];

  Expect(start_x != end_x || start_y != end_y || start_z != end_z,
         "expected direct-path dynamic obstacle geom position to change after stepping Scout simulation");

  fs::remove(obstacle_config_path);
  fs::remove(scene_path);
}

void ExpectMocapPathMovesMocapBody(const fs::path& repo_root) {
  const fs::path obstacle_config_path = fs::temp_directory_path() / "scout_dynamic_obstacle_mocap_test.yaml";
  const fs::path scene_path = repo_root / "assets" / "scout_v2" / "scene.dynamic_obstacles.mocap_test.xml";
  WriteObstacleConfig(obstacle_config_path, true);
  GenerateSceneWithObstacles(repo_root, obstacle_config_path, scene_path);

  ground_vehicle::ScoutConfig config =
      ground_vehicle::LoadScoutConfigFromYaml((repo_root / "ground_vehicle" / "cfg" / "sim_config.yaml").string(), "");
  config.common.viewer.enabled = false;
  config.common.viewer.fallback_to_headless = true;
  config.common.model.scene_xml = scene_path;
  config.common.dynamic_obstacle.enabled = true;
  config.common.dynamic_obstacle.config_path = obstacle_config_path.string();

  ground_vehicle::ScoutSim sim(config);
  sim.LoadModel();
  const mjModel* model = sim.model();
  const mjData* data = sim.data();
  Expect(model != nullptr, "expected Scout model to load for mocap obstacle path");
  Expect(data != nullptr, "expected Scout mjData to be available for mocap obstacle path");

  const int body_id = mj_name2id(model, mjOBJ_BODY, "dynamic_obs_0");
  Expect(body_id >= 0, "expected generated scene to contain a dynamic_obs_0 mocap body");
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
         "expected mocap-path dynamic obstacle pose to change after stepping Scout simulation");

  fs::remove(obstacle_config_path);
  fs::remove(scene_path);
}

}  // namespace

int main() {
  const fs::path repo_root = ResolveRepoRoot();
  Expect(!repo_root.empty(), "failed to locate repo root for dynamic_obstacle_test");
  ExpectDirectGeomPathMovesGeom(repo_root);
  ExpectMocapPathMovesMocapBody(repo_root);
  return 0;
}
