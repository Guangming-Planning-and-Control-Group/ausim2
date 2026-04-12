#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "config/scout_config.hpp"
#include "manager/ros_bridge_process_manager.hpp"
#include "sim/scout_sim.hpp"

namespace fs = std::filesystem;

namespace {

struct CliOptions {
  fs::path merged_config_path;
  fs::path sim_config_path;
  fs::path robot_config_path;
  bool no_ros = false;
  bool force_viewer = false;
  bool force_headless = false;
};

fs::path ResolveExistingPath(const std::vector<fs::path>& candidates) {
  for (const fs::path& candidate : candidates) {
    if (!candidate.empty() && fs::exists(candidate)) {
      return candidate.lexically_normal();
    }
  }
  return {};
}

fs::path ResolveDefaultConfigPath(const char* filename) {
  return ResolveExistingPath({
      fs::current_path() / "ground_vehicle" / "cfg" / filename,
      fs::current_path() / "cfg" / filename,
      fs::current_path() / ".." / "ground_vehicle" / "cfg" / filename,
      fs::path(GROUND_VEHICLE_SOURCE_DIR) / "cfg" / filename,
  });
}

std::vector<std::string> BuildBridgeConfigArguments(const CliOptions& cli) {
  std::vector<std::string> args;
  if (!cli.merged_config_path.empty()) {
    args.push_back("--config");
    args.push_back(cli.merged_config_path.string());
    return args;
  }

  if (!cli.sim_config_path.empty()) {
    args.push_back("--sim-config");
    args.push_back(cli.sim_config_path.string());
  }
  if (!cli.robot_config_path.empty()) {
    args.push_back("--robot-config");
    args.push_back(cli.robot_config_path.string());
  }
  return args;
}

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name
            << " [--sim-config <path>] [--robot-config <override-path>] "
               "[--config <legacy.yaml>] [--viewer|--headless] [--no-ros]\n";
}

ground_vehicle::ScoutConfig LoadConfig(const CliOptions& cli) {
  if (!cli.merged_config_path.empty()) {
    return ground_vehicle::LoadScoutConfigFromYaml(cli.merged_config_path.string());
  }

  fs::path sim_config_path = cli.sim_config_path;
  if (sim_config_path.empty()) {
    sim_config_path = ResolveDefaultConfigPath("sim_config.yaml");
  }
  if (sim_config_path.empty()) {
    throw std::runtime_error(
        "Unable to locate default simulation config. Expected ground_vehicle/cfg/sim_config.yaml.");
  }
  return ground_vehicle::LoadScoutConfigFromYaml(
      sim_config_path.string(),
      cli.robot_config_path.string());
}

}  // namespace

int main(int argc, char** argv) {
  try {
    CliOptions cli;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        PrintUsage(argv[0]);
        return 0;
      } else if (arg == "--config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--config requires a file path.");
        }
        cli.merged_config_path = argv[++i];
      } else if (arg == "--sim-config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--sim-config requires a file path.");
        }
        cli.sim_config_path = argv[++i];
      } else if (arg == "--robot-config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--robot-config requires a file path.");
        }
        cli.robot_config_path = argv[++i];
      } else if (arg == "--no-ros") {
        cli.no_ros = true;
      } else if (arg == "--viewer") {
        cli.force_viewer = true;
      } else if (arg == "--headless") {
        cli.force_headless = true;
      } else {
        if (!cli.merged_config_path.empty()) {
          throw std::runtime_error("Only one positional legacy config path is supported.");
        }
        cli.merged_config_path = arg;
      }
    }

    if (cli.merged_config_path.empty() && cli.sim_config_path.empty()) {
      cli.sim_config_path = ResolveDefaultConfigPath("sim_config.yaml");
      if (cli.sim_config_path.empty()) {
        throw std::runtime_error(
            "Unable to locate default simulation config. Expected ground_vehicle/cfg/sim_config.yaml.");
      }
    }

    ground_vehicle::ScoutConfig config = LoadConfig(cli);
    if (cli.force_viewer) {
      config.common.viewer.enabled = true;
    }
    if (cli.force_headless) {
      config.common.viewer.enabled = false;
    }

    if (const char* scene_override = std::getenv("AUSIM_SCENE_XML_OVERRIDE");
        scene_override != nullptr && scene_override[0] != '\0') {
      config.common.model.scene_xml = fs::absolute(scene_override);
    }

    std::unique_ptr<ausim::RosBridgeProcessManager> bridge_manager;
    if (!cli.no_ros) {
      fs::path self_executable = fs::absolute(argv[0]);
      if (!fs::exists(self_executable)) {
        self_executable = ResolveExistingPath({
            fs::current_path() / argv[0],
            fs::current_path() / "build" / "bin" / "scout",
        });
      }
      if (self_executable.empty()) {
        throw std::runtime_error("Unable to resolve current executable path.");
      }

      ausim::RosBridgeLaunchConfig bridge_launch_config;
      bridge_launch_config.executable_path =
          self_executable.parent_path() / "ausim_ros_bridge";
      bridge_launch_config.config_arguments = BuildBridgeConfigArguments(cli);

      bridge_manager = std::make_unique<ausim::RosBridgeProcessManager>(
          config.common, std::move(bridge_launch_config));
      bridge_manager->Start();
    }

    try {
      ground_vehicle::ScoutSim sim(std::move(config));
      sim.Run();
    } catch (...) {
      if (bridge_manager) {
        bridge_manager->Stop();
      }
      throw;
    }

    if (bridge_manager) {
      bridge_manager->Stop();
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "scout error: " << error.what() << '\n';
    return 1;
  }
}
