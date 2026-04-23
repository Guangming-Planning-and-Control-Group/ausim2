#pragma once

#include <string>
#include <vector>

namespace ausim {

struct DynamicObstacleEntry {
  std::string name;
  double pos[3] = {0.0, 0.0, 0.0};
  double quat[4] = {1.0, 0.0, 0.0, 0.0};
  double size[3] = {0.0, 0.0, 0.0};
};

struct DynamicObstaclesSnapshot {
  double sim_time = 0.0;
  std::vector<DynamicObstacleEntry> entries;
  std::string frame_id;
};

}  // namespace ausim
