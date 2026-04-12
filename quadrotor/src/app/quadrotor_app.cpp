#include "app/quadrotor_app.hpp"

#include "sim/quadrotor_sim.hpp"

namespace quadrotor {

QuadrotorApp::QuadrotorApp(QuadrotorConfig config, RosBridgeLaunchConfig bridge_launch_config)
    : config_(std::move(config)), bridge_launch_config_(std::move(bridge_launch_config)) {}

int QuadrotorApp::Run() {
  RosBridgeProcessManager bridge_manager(config_, bridge_launch_config_);
  bridge_manager.Start();

  try {
    QuadrotorSim sim(config_);
    sim.Run();
  } catch (...) {
    bridge_manager.Stop();
    throw;
  }

  bridge_manager.Stop();
  return 0;
}

}  // namespace quadrotor
