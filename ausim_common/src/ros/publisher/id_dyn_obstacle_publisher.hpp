#pragma once

#include "runtime/dynamic_obstacles_snapshot.hpp"

namespace ausim {

class IDynObstaclePublisher {
 public:
  virtual ~IDynObstaclePublisher() = default;
  virtual void Publish(const DynamicObstaclesSnapshot& snapshot) = 0;
};

}  // namespace ausim
