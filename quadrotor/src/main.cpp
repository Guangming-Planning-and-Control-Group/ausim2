#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

fs::path DefaultConfigPath(const char* argv0) {
  const fs::path executable_path = fs::absolute(argv0);
  const fs::path installed_config =
      (executable_path.parent_path() / "../share/quadrotor/config.yaml").lexically_normal();
  if (fs::exists(installed_config)) {
    return installed_config;
  }

  const fs::path source_config = fs::path(QUADROTOR_SOURCE_DIR) / "config.yaml";
  if (fs::exists(source_config)) {
    return source_config;
  }

  return {};
}

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [config.yaml] [--viewer|--headless]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::filesystem::path config_path = DefaultConfigPath(argv[0]);
    bool force_viewer = false;
    bool force_headless = false;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        PrintUsage(argv[0]);
        return 0;
      } else if (arg == "--viewer") {
        force_viewer = true;
      } else if (arg == "--headless") {
        force_headless = true;
      } else {
        config_path = arg;
      }
    }

    if (config_path.empty()) {
      throw std::runtime_error("Unable to locate config.yaml. Pass it explicitly on the command line.");
    }

    quadrotor::QuadrotorConfig config = quadrotor::LoadConfigFromYaml(config_path.string());
    if (force_viewer) {
      config.viewer.enabled = true;
    }
    if (force_headless) {
      config.viewer.enabled = false;
    }
    quadrotor::QuadrotorSim sim(std::move(config));
    sim.Run();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "quadrotor error: " << error.what() << '\n';
    return 1;
  }
}
