#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

fs::path ResolveConfigPath(const fs::path& input_path) {
  if (!input_path.empty()) {
    return input_path;
  }

  const fs::path default_path = fs::current_path() / "config.yaml";
  if (fs::exists(default_path)) {
    return default_path.lexically_normal();
  }
  return {};
}

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [config.yaml] [--viewer|--headless]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    (void)argv;
    std::filesystem::path config_arg;
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
        config_arg = arg;
      }
    }

    const std::filesystem::path config_path = ResolveConfigPath(config_arg);

    if (config_path.empty()) {
      throw std::runtime_error("Unable to locate config.yaml in current working directory.");
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
