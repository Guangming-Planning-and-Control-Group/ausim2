#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/quadrotor_config.hpp"
#include "ros/ros2_bridge.hpp"

namespace fs = std::filesystem;

namespace {

struct CliOptions {
  fs::path merged_config_path;
  fs::path sim_config_path;
  fs::path robot_config_path;
  int telemetry_fd = -1;
  int command_fd = -1;
  int image_fd = -1;
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

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name
            << " [--sim-config <path>] [--robot-config <override-path>] [--config <legacy.yaml>] "
               "--telemetry-fd <fd> --command-fd <fd> --image-fd <fd>\n";
}

ausim::QuadrotorConfig LoadConfig(const CliOptions& cli) {
  if (!cli.merged_config_path.empty()) {
    return ausim::LoadConfigFromYaml(cli.merged_config_path.string());
  }

  fs::path sim_config_path = cli.sim_config_path;
  if (sim_config_path.empty()) {
    sim_config_path = ResolveDefaultConfigPath("sim_config.yaml");
  }
  if (sim_config_path.empty()) {
    throw std::runtime_error("Unable to locate default simulation config. Expected quadrotor/cfg/sim_config.yaml.");
  }
  return ausim::LoadConfigFromYaml(sim_config_path.string(), cli.robot_config_path.string());
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
      } else if (arg == "--telemetry-fd") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--telemetry-fd requires a file descriptor.");
        }
        cli.telemetry_fd = std::stoi(argv[++i]);
      } else if (arg == "--command-fd") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--command-fd requires a file descriptor.");
        }
        cli.command_fd = std::stoi(argv[++i]);
      } else if (arg == "--image-fd") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--image-fd requires a file descriptor.");
        }
        cli.image_fd = std::stoi(argv[++i]);
      } else {
        if (!cli.merged_config_path.empty()) {
          throw std::runtime_error("Only one positional legacy config path is supported.");
        }
        cli.merged_config_path = arg;
      }
    }

    if (cli.telemetry_fd < 0 || cli.command_fd < 0 || cli.image_fd < 0) {
      throw std::runtime_error("--telemetry-fd, --command-fd, and --image-fd are all required.");
    }

    const ausim::QuadrotorConfig config = LoadConfig(cli);
    return ausim::RunRosBridgeProcess(config, cli.telemetry_fd, cli.command_fd, cli.image_fd);
  } catch (const std::exception& error) {
    std::cerr << "ausim_ros_bridge error: " << error.what() << '\n';
    return 1;
  }
}
