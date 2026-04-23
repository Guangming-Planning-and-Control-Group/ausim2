#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "converts/ipc/bridge_packets.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string& label) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  ausim::DynamicObstaclesSnapshot input;
  input.sim_time = 12.5;
  input.frame_id = "world";

  ausim::DynamicObstacleEntry first;
  first.name = "dynamic_obs_0";
  first.pos[0] = 1.0;
  first.pos[1] = -2.0;
  first.pos[2] = 0.5;
  first.quat[0] = 1.0;
  first.size[0] = 0.6;
  first.size[1] = 0.8;
  first.size[2] = 1.2;
  input.entries.push_back(first);

  ausim::DynamicObstacleEntry second;
  second.name = "dynamic_obs_1";
  second.pos[0] = -3.0;
  second.pos[1] = 4.0;
  second.pos[2] = 1.5;
  second.quat[0] = 0.70710678;
  second.quat[3] = 0.70710678;
  second.size[0] = 1.0;
  second.size[1] = 1.5;
  second.size[2] = 2.0;
  input.entries.push_back(second);

  std::vector<std::uint8_t> bytes;
  Expect(ausim::converts::ToDynObstaclePacket(input, bytes), "expected packet serialization to succeed");
  Expect(!bytes.empty(), "expected serialized packet bytes");

  ausim::DynamicObstaclesSnapshot output;
  Expect(ausim::converts::FromDynObstaclePacketBytes(bytes.data(), bytes.size(), output), "expected packet deserialization to succeed");
  ExpectNear(output.sim_time, input.sim_time, 1e-9, "sim_time");
  Expect(output.frame_id == "world", "expected frame_id to round-trip");
  Expect(output.entries.size() == 2, "expected two obstacle entries");
  Expect(output.entries[0].name == "dynamic_obs_0", "expected first obstacle name");
  ExpectNear(output.entries[0].size[2], 1.2, 1e-9, "first obstacle size.z");
  Expect(output.entries[1].name == "dynamic_obs_1", "expected second obstacle name");
  ExpectNear(output.entries[1].quat[3], 0.70710678, 1e-9, "second obstacle quat.z");
  return 0;
}
