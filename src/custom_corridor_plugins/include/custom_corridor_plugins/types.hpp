#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <sdf/Element.hh>

namespace custom_corridor
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-6;

inline double NormalizeAngle(double val)
{
  return std::atan2(std::sin(val), std::cos(val));
}

inline double Distance2D(double x0, double y0, double x1, double y1)
{
  return std::hypot(x1 - x0, y1 - y0);
}

inline double HeadingTo(double x0, double y0, double x1, double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}

inline std::string TwoDigit(int id)
{
  std::ostringstream ss;
  ss << std::setw(2) << std::setfill('0') << id;
  return ss.str();
}

template <typename T>
inline T SdfValueOr(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const T &_defaultValue)
{
  if (_sdf && _sdf->HasElement(_name))
    return _sdf->Get<T>(_name);
  return _defaultValue;
}

enum class HumanMode
{
  Hidden,
  Stationary,
  Autonomous
};

enum class HumanPersona
{
  H1_Cooperative = 1,     // Reciprocal ORCA (alpha = 0.50, ~40%)
  H2_Unyielding = 2,      // Low-responsibility ORCA (alpha in [0.10, 0.35], ~10%)
  H3_RobotBlind = 3,      // Completely ignores robot (neighbor filtering, ~25%)
  H4_Static = 4,          // Stationary / conversational group (v_pref = 0, ~15%)
  H5_SocialForce = 5      // Soft exponential repulsion + LPF (~10%)
};

inline std::string PersonaToString(HumanPersona p)
{
  switch (p)
  {
    case HumanPersona::H1_Cooperative: return "H1";
    case HumanPersona::H2_Unyielding: return "H2";
    case HumanPersona::H3_RobotBlind: return "H3";
    case HumanPersona::H4_Static: return "H4";
    case HumanPersona::H5_SocialForce: return "H5";
    default: return "H1";
  }
}

inline HumanPersona StringToPersona(const std::string &str)
{
  if (str == "H1" || str == "cooperative" || str == "1") return HumanPersona::H1_Cooperative;
  if (str == "H2" || str == "unyielding" || str == "2") return HumanPersona::H2_Unyielding;
  if (str == "H3" || str == "blind" || str == "robot_blind" || str == "3") return HumanPersona::H3_RobotBlind;
  if (str == "H4" || str == "static" || str == "4") return HumanPersona::H4_Static;
  if (str == "H5" || str == "social_force" || str == "sfm" || str == "5") return HumanPersona::H5_SocialForce;
  return HumanPersona::H1_Cooperative;
}

enum class CorridorState
{
  Patrol,
  YieldingEnter,
  YieldingWait,
  YieldingExit
};

struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

struct HumanAgent
{
  int id{0};
  std::string name;
  HumanMode mode{HumanMode::Hidden};
  HumanPersona persona{HumanPersona::H1_Cooperative};
  bool active{false};

  // Kinematic state
  double x{0.0};
  double y{0.0};
  double z{0.85};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double prefVx{0.0};
  double prefVy{0.0};

  // Navigation targets
  double speed{0.85};
  double maxSpeed{1.2};
  double radius{0.28};
  bool loop{true};
  bool waitForRobot{false};
  std::size_t waypointIndex{0};
  std::vector<Waypoint> waypoints;
  double waypointTolerance{0.35};

  // Elevation and Scale
  double visualZ{1.0};
  double proxyZ{0.85};
  double scale{1.0};

  // Social traits & Personas H1-H5
  double politeness{0.5}; // Yielding / politeness weight in [0.0 = distracted/stubborn, 1.0 = highly cooperative]
  double responsibilityWeight{0.50}; // Continuous responsibility weight alpha (e.g. 0.50 for H1, 0.10..0.35 for H2)
  double latencyTau{0.3}; // Reaction latency time constant (seconds)

  // Corridor Recess State
  CorridorState corridorState{CorridorState::Patrol};
  double waitDuration{0.0};

  // Gazebo entities
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity actorEntity{gz::sim::kNullEntity};
  gz::math::Pose3d actorOrigin{0, 0, 0, 0, 0, 0};
  bool isSingleModel{true};

  // RVO agent index & animation
  std::size_t rvoIndex{0};
  double animTimeSec{0.0};
  std::chrono::steady_clock::duration animationTime{0};

  // Wall-clearance and safety tracking
  uint64_t totalStepsCount{0};
  uint64_t clampedStepsCount{0};
  uint64_t penetrationStepsCount{0};
  double maxPenetrationDepth{0.0};
  double maxLateralDev{0.0};

  void Reset()
  {
    this->vx = 0.0;
    this->vy = 0.0;
    this->prefVx = 0.0;
    this->prefVy = 0.0;
    this->waypointIndex = 0;
    this->corridorState = CorridorState::Patrol;
    this->waitDuration = 0.0;
    this->animTimeSec = 0.0;
    this->animationTime = std::chrono::steady_clock::duration{0};
    this->totalStepsCount = 0;
    this->clampedStepsCount = 0;
    this->penetrationStepsCount = 0;
    this->maxPenetrationDepth = 0.0;
    this->maxLateralDev = 0.0;
  }
};

} // namespace custom_corridor
