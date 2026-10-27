#pragma once

#include "custom_corridor_plugins/types.hpp"

namespace custom_corridor
{

struct CorridorRecessConfig
{
  bool corridorMode{false};
  double corridorHalfWidth{0.45};
  double recessXMin{1.35};
  double recessXMax{2.65};
  double recessXCenter{2.00};
  double recessY{0.75};
  double recessApproachDist{5.0};
  double recessThreshold{0.40};
};

struct RobotTrackerState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double speed{0.0};
  bool hasPrevPose{false};
};

bool UpdateCorridorRecessYielding(
    HumanAgent &human,
    const CorridorRecessConfig &cfg,
    const RobotTrackerState &robot,
    double dt,
    double &outDesiredVx,
    double &outDesiredVy);

void ClampCorridorWallBoundaries(
    HumanAgent &human,
    const CorridorRecessConfig &cfg);

} // namespace custom_corridor
