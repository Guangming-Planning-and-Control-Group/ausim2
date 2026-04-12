#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "app/quadrotor_app.hpp"

namespace fs = std::filesystem;

namespace {

struct CliOptions {
  fs::path merged_config_path;
  fs::path sim_config_path;
  fs::path robot_config_path;
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
      fs::current_path() / "quadrotor" / "cfg" / filename,
      fs::current_path() / "cfg" / filename,
      fs::current_path() / ".." / "quadrotor" / "cfg" / filename,
      fs::path(QUADROTOR_SOURCE_DIR) / "cfg" / filename,
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
            << " [--sim-config <path>] [--robot-config <override-path>] [--config <legacy.yaml>] "
               "[--viewer|--headless]\n";
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

    if (!cli.merged_config_path.empty() && (!cli.sim_config_path.empty() || !cli.robot_config_path.empty())) {
      throw std::runtime_error("Use either --config or --sim-config, with optional --robot-config override.");
    }

    quadrotor::QuadrotorConfig config;
    if (!cli.merged_config_path.empty()) {
      config = quadrotor::LoadConfigFromYaml(cli.merged_config_path.string());
    } else {
      if (cli.sim_config_path.empty()) {
        cli.sim_config_path = ResolveDefaultConfigPath("sim_config.yaml");
      }
      if (cli.sim_config_path.empty()) {
        throw std::runtime_error("Unable to locate default simulation config. Expected quadrotor/cfg/sim_config.yaml.");
      }
      config = quadrotor::LoadConfigFromYaml(cli.sim_config_path.string(), cli.robot_config_path.string());
    }

    if (cli.force_viewer) {
      config.viewer.enabled = true;
    }
    if (cli.force_headless) {
      config.viewer.enabled = false;
    }

    if (const char* scene_override = std::getenv("AUSIM_SCENE_XML_OVERRIDE"); scene_override != nullptr && scene_override[0] != '\0') {
      config.model.scene_xml = fs::absolute(scene_override);
    }

    fs::path self_executable = fs::absolute(argv[0]);
    if (!fs::exists(self_executable)) {
      self_executable = ResolveExistingPath({
          fs::current_path() / argv[0],
          fs::current_path() / "build" / "bin" / "quadrotor",
      });
    }
    if (self_executable.empty()) {
      throw std::runtime_error("Unable to resolve current executable path.");
    }

    quadrotor::RosBridgeLaunchConfig bridge_launch_config;
    bridge_launch_config.executable_path = self_executable.parent_path() / "ausim_ros_bridge";
    bridge_launch_config.config_arguments = BuildBridgeConfigArguments(cli);

    quadrotor::QuadrotorApp app(std::move(config), std::move(bridge_launch_config));
    return app.Run();
  } catch (const std::exception& error) {
    std::cerr << "quadrotor error: " << error.what() << '\n';
    return 1;
  }
}
