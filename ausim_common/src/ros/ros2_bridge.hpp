#pragma once

#include "config/quadrotor_config.hpp"

namespace ausim {

int RunRosBridgeProcess(
    const QuadrotorConfig& config,
    int telemetry_fd,
    int command_fd,
    int image_fd);

}  // namespace ausim

namespace quadrotor {

using ::ausim::RunRosBridgeProcess;

}  // namespace quadrotor
