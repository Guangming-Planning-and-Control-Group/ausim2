#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "converts/data/lidar.hpp"
#include "ipc/lidar_packet.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void ExpectNear(float actual, float expected, float tolerance, const std::string& label) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void ExpectPoint(const std::vector<float>& xyz, std::size_t point_index, float x, float y, float z, float tolerance) {
  const std::size_t offset = point_index * 3;
  Expect(offset + 2 < xyz.size(), "point index out of range");
  ExpectNear(xyz[offset + 0], x, tolerance, "point.x");
  ExpectNear(xyz[offset + 1], y, tolerance, "point.y");
  ExpectNear(xyz[offset + 2], z, tolerance, "point.z");
}

float Degrees(float degrees) { return degrees * static_cast<float>(M_PI) / 180.0f; }

void TestSingleHitUsesPluginLayoutAndAngles() {
  ausim::ipc::LidarPacket packet;
  packet.h_ray_num = 3;
  packet.v_ray_num = 2;
  packet.fov_h_deg = 90.0f;
  packet.fov_v_deg = 30.0f;
  packet.range_min_m = 0.1f;
  packet.range_max_m = 10.0f;

  // Plugin layout: index = v * h + h.
  packet.data[0] = 2.0f;   // (v=0, h=0)
  packet.data[1] = 10.0f;  // no hit
  packet.data[2] = 10.0f;  // no hit
  packet.data[3] = 10.0f;  // no hit
  packet.data[4] = 10.0f;  // no hit
  packet.data[5] = 10.0f;  // no hit

  const std::vector<float> xyz = ausim::converts::BuildLidarPointCloudXYZ(packet);
  Expect(xyz.size() == 3, "expected exactly one xyz point from one valid hit");

  constexpr float kTolerance = 1e-3f;
  const float expected_x = 2.0f * std::cos(Degrees(15.0f)) * std::cos(Degrees(45.0f));
  const float expected_y = 2.0f * std::cos(Degrees(15.0f)) * std::sin(Degrees(45.0f));
  const float expected_z = 2.0f * std::sin(Degrees(15.0f));
  ExpectPoint(xyz, 0, expected_x, expected_y, expected_z, kTolerance);
}

void TestNoHitRangesAreDropped() {
  ausim::ipc::LidarPacket packet;
  packet.h_ray_num = 4;
  packet.v_ray_num = 1;
  packet.fov_h_deg = 180.0f;
  packet.fov_v_deg = 0.0f;
  packet.range_min_m = 0.1f;
  packet.range_max_m = 10.0f;

  packet.data[0] = 10.0f;
  packet.data[1] = 9.99995f;
  packet.data[2] = 0.0f;
  packet.data[3] = 10.0f;

  const std::vector<float> xyz = ausim::converts::BuildLidarPointCloudXYZ(packet);
  Expect(xyz.empty(), "expected no-hit samples at max range to be filtered out");
}

void TestVerticalMajorOrderingIsPreserved() {
  ausim::ipc::LidarPacket packet;
  packet.h_ray_num = 2;
  packet.v_ray_num = 2;
  packet.fov_h_deg = 90.0f;
  packet.fov_v_deg = 30.0f;
  packet.range_min_m = 0.1f;
  packet.range_max_m = 10.0f;

  packet.data[0] = 10.0f;  // (v=0, h=0) no hit
  packet.data[1] = 3.0f;   // (v=0, h=1) valid
  packet.data[2] = 4.0f;   // (v=1, h=0) valid
  packet.data[3] = 10.0f;  // (v=1, h=1) no hit

  const std::vector<float> xyz = ausim::converts::BuildLidarPointCloudXYZ(packet);
  Expect(xyz.size() == 6, "expected two valid hits");

  constexpr float kTolerance = 1e-3f;
  // First valid hit should be (v=0, h=1): front-right/up.
  ExpectPoint(xyz, 0, 3.0f * std::cos(Degrees(15.0f)) * std::cos(Degrees(-45.0f)), 3.0f * std::cos(Degrees(15.0f)) * std::sin(Degrees(-45.0f)),
              3.0f * std::sin(Degrees(15.0f)), kTolerance);
  // Second valid hit should be (v=1, h=0): front-left/down.
  ExpectPoint(xyz, 1, 4.0f * std::cos(Degrees(-15.0f)) * std::cos(Degrees(45.0f)), 4.0f * std::cos(Degrees(-15.0f)) * std::sin(Degrees(45.0f)),
              4.0f * std::sin(Degrees(-15.0f)), kTolerance);
}

}  // namespace

int main() {
  TestSingleHitUsesPluginLayoutAndAngles();
  TestNoHitRangesAreDropped();
  TestVerticalMajorOrderingIsPreserved();
  return 0;
}
