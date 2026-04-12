#pragma once

#include "ipc/bridge_packets.hpp"

namespace ausim {

class ITelemetryPublisher {
 public:
  virtual ~ITelemetryPublisher() = default;
  virtual void Publish(const ipc::TelemetryPacket& packet) = 0;
};

}  // namespace ausim
