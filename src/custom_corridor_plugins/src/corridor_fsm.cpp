#include "custom_corridor_plugins/corridor_fsm.hpp"
#include <algorithm>
#include <cmath>

namespace custom_corridor
{

bool UpdateCorridorRecessYielding(
    HumanAgent &human,
    const CorridorRecessConfig &cfg,
    const RobotTrackerState &robot,
    double dt,
    double &outDesiredVx,
    double &outDesiredVy)
{
  if (!cfg.corridorMode || !robot.hasPrevPose || human.persona == HumanPersona::H3_RobotBlind)
  {
    return false;
  }

  const double rx = robot.x - human.x;
  const double ry = robot.y - human.y;
  const double distToRobot = std::hypot(rx, ry);

  double patrolDirX = -1.0;
  if (!human.waypoints.empty())
  {
    const auto &wp = human.waypoints[human.waypointIndex];
    patrolDirX = (wp.x - human.x >= 0.0) ? 1.0 : -1.0;
  }

  // Robot is in front of human's travel direction
  const bool robotInFront = (patrolDirX * rx > 0.0);
  // Robot is actively moving towards human (closing distance)
  const bool robotMovingTowards = (robot.speed > 0.03 && (rx * robot.vx < 0.0));
  // Is corridor a single-lane bottleneck (< 1.10m)? (corridor_070 and corridor_090)
  const bool isNarrowCorridor = (cfg.corridorHalfWidth < 0.55);

  const double maxAllowedY = std::max(0.05, cfg.corridorHalfWidth - human.radius - 0.02);

  if (!isNarrowCorridor)
  {
    // Wide corridor (1.20m): agents can pass each other side-by-side!
    // Human shifts to their right lane (+Y if heading -X, -Y if heading +X) up to maxAllowedY
    if (robotInFront && distToRobot < 4.5)
    {
      const double rightY = (patrolDirX < 0.0) ? maxAllowedY : -maxAllowedY;
      const double dyToRight = rightY - human.y;
      outDesiredVy += std::clamp(dyToRight * 1.5, -0.35, 0.35);
    }
    return false; // Continuous patrol continues with lateral nudge added
  }

  // Single-lane bottleneck (corridor 0.70m and 0.90m)
  if (human.corridorState == CorridorState::Patrol)
  {
    human.waitDuration = 0.0;

    // ONLY enter recess if human is right at the recess (x in [1.15, 2.95])
    // AND robot is ACTUALLY CLOSE (distToRobot < 3.5m) and approaching!
    const bool nearRecessAhead = (human.x >= (cfg.recessXMin - 0.2) && human.x <= (cfg.recessXMax + 0.3));
    const bool robotCloseAndApproaching = robotInFront && (robotMovingTowards || distToRobot < 2.5) && (distToRobot < 3.5);

    // Or if human is just past the recess (x in [-1.5, 1.15]) and meets robot close (< 2.8m)
    const bool insideBottleneck = (human.x < (cfg.recessXMin - 0.2) && human.x >= (cfg.recessXMin - 2.5));
    const bool confrontationNearAlcove = insideBottleneck && robotInFront && (robotMovingTowards || distToRobot < 2.5) && (distToRobot < 2.8);

    if ((nearRecessAhead && robotCloseAndApproaching) || confrontationNearAlcove)
    {
      human.corridorState = CorridorState::YieldingEnter;
      return true;
    }
    else if (robotInFront && distToRobot < 4.0)
    {
      // When walking down the corridor, step to the right wall up to maxAllowedY
      // If heading -X (patrolDirX < 0), right is +Y. If heading +X, right is -Y.
      const double rightY = (patrolDirX < 0.0) ? maxAllowedY : -maxAllowedY;
      const double dyToRight = rightY - human.y;
      outDesiredVy += std::clamp(dyToRight * 1.5, -0.35, 0.35);
      return false;
    }
    return false;
  }
  else if (human.corridorState == CorridorState::YieldingEnter)
  {
    const double targetX = cfg.recessXCenter;
    const double targetY = cfg.recessY;
    const double dxRecess = targetX - human.x;
    const double dyRecess = targetY - human.y;
    const double dRecess = std::hypot(dxRecess, dyRecess);

    if (dRecess < 0.25)
    {
      human.corridorState = CorridorState::YieldingWait;
      human.waitDuration = 0.0;
      outDesiredVx = 0.0;
      outDesiredVy = 0.0;
    }
    else
    {
      outDesiredVx = (dxRecess / dRecess) * human.speed * 0.85;
      outDesiredVy = (dyRecess / dRecess) * human.speed * 0.85;
    }
    return true;
  }
  else if (human.corridorState == CorridorState::YieldingWait)
  {
    outDesiredVx = 0.0;
    outDesiredVy = 0.0;
    human.waitDuration += dt;

    // Human turns to face the corridor (-pi/2) while waiting for robot to pass
    const double targetHeading = -1.5708;
    const double headingDiff = NormalizeAngle(targetHeading - human.yaw);
    human.yaw = NormalizeAngle(human.yaw + std::clamp(headingDiff, -4.0 * dt, 4.0 * dt));

    // Robot has cleared if it has driven past the recess in the direction of travel
    const bool robotPassedX = (robot.vx >= -0.01) ?
        (robot.x > (cfg.recessXMax + 0.35)) :
        (robot.x < (cfg.recessXMin - 0.35));

    // Fail-safe: if robot stopped or turned around for > 8.0s far away (dist > 4.5m), resume patrol
    const bool robotGaveUp = (human.waitDuration > 8.0 && robot.speed < 0.03 && distToRobot > 4.5);

    if (robotPassedX || robotGaveUp)
    {
      human.corridorState = CorridorState::YieldingExit;
    }
    return true;
  }
  else if (human.corridorState == CorridorState::YieldingExit)
  {
    const double targetX = cfg.recessXCenter;
    const double targetY = 0.0;
    const double dxExit = targetX - human.x;
    const double dyExit = targetY - human.y;
    const double dExit = std::hypot(dxExit, dyExit);

    if (std::abs(human.y) < 0.12)
    {
      human.corridorState = CorridorState::Patrol;
      human.waitDuration = 0.0;
      return false;
    }
    else
    {
      outDesiredVx = (dxExit / std::max(0.01, dExit)) * human.speed * 0.85;
      outDesiredVy = (dyExit / std::max(0.01, dExit)) * human.speed * 0.85;
      return true;
    }
  }

  return false;
}

void ClampCorridorWallBoundaries(
    HumanAgent &human,
    const CorridorRecessConfig &cfg)
{
  if (!cfg.corridorMode)
    return;

  const double allowedY = std::max(0.05, cfg.corridorHalfWidth - human.radius - 0.02);
  double minWallY = -allowedY;
  double maxWallY = allowedY;

  // If human is actively maneuvering within recess alcove
  if (human.x >= (cfg.recessXMin - 0.1) && human.x <= (cfg.recessXMax + 0.1) &&
      (human.corridorState == CorridorState::YieldingEnter ||
       human.corridorState == CorridorState::YieldingWait ||
       human.corridorState == CorridorState::YieldingExit))
  {
    maxWallY = cfg.recessY + 0.10;
  }

  human.totalStepsCount++;
  if (human.y < minWallY || human.y > maxWallY)
  {
    human.clampedStepsCount++;
    human.y = std::clamp(human.y, minWallY, maxWallY);
  }

  if (std::abs(human.y) > human.maxLateralDev)
  {
    human.maxLateralDev = std::abs(human.y);
  }

  // Track wall penetration (outer body boundary = |y| + radius)
  const double outerEdgeY = std::abs(human.y) + human.radius;
  if (outerEdgeY > cfg.corridorHalfWidth)
  {
    human.penetrationStepsCount++;
    const double depth = outerEdgeY - cfg.corridorHalfWidth;
    if (depth > human.maxPenetrationDepth)
    {
      human.maxPenetrationDepth = depth;
    }
  }
}

} // namespace custom_corridor
