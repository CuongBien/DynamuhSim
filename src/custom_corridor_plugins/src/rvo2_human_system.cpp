// rvo2_human_system.cpp
//
// Gazebo Sim 8 (Harmonic) World System Plugin for Social Human Navigation using RVO2 / ORCA.
//
// Features:
// - High-throughput reciprocal collision avoidance (ORCA) in C++.
// - Asymmetric Politeness: humans apply lateral yielding bias and social clearance when facing the robot.
// - Anisotropic Proxemics: personal space modeling (Hall's proxemics).
// - Human Reaction Latency: low-pass smoothed velocity response (250-400 ms reaction time).
// - Single-model and legacy dual-layer (proxy+actor) entity compatibility.
// - Synchronous ECM PreUpdate execution (zero sensor desync).
// - Real-time JSON Ground-Truth publishing.
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <sdf/Element.hh>

#include "rvo2/RVO.h"

namespace custom_corridor
{
namespace
{
using json = nlohmann::json;

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

std::string TwoDigit(int id)
{
  std::ostringstream ss;
  ss << std::setw(2) << std::setfill('0') << id;
  return ss.str();
}

template <typename T>
T SdfValueOr(
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
  std::size_t waypointIndex{0};
  std::vector<Waypoint> waypoints;
  double waypointTolerance{0.35};

  // Elevation and Scale
  double visualZ{1.0};
  double proxyZ{0.85};
  double scale{1.0};

  // Social traits
  double politeness{0.5}; // Yielding / politeness weight in [0.0 = distracted/stubborn, 1.0 = highly cooperative]
  double latencyTau{0.3}; // Reaction latency time constant (seconds)

  // Corridor Recess State
  CorridorState corridorState{CorridorState::Patrol};

  // Gazebo entities
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity actorEntity{gz::sim::kNullEntity};
  gz::math::Pose3d actorOrigin{0, 0, 0, 0, 0, 0};
  bool isSingleModel{true};

  // RVO agent index & animation
  std::size_t rvoIndex{0};
  double animTimeSec{0.0};
  std::chrono::steady_clock::duration animationTime{0};

  void Reset()
  {
    this->vx = 0.0;
    this->vy = 0.0;
    this->prefVx = 0.0;
    this->prefVy = 0.0;
    this->waypointIndex = 0;
    this->corridorState = CorridorState::Patrol;
    this->animTimeSec = 0.0;
    this->animationTime = std::chrono::steady_clock::duration{0};
  }
};

} // namespace

class RVO2HumanSystem final : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate,
                              public gz::sim::ISystemReset
{
public:
  RVO2HumanSystem() = default;
  ~RVO2HumanSystem() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->worldEntity_ = _entity;

    this->humanCount_ = SdfValueOr<int>(_sdf, "human_count", 4);
    this->robotName_ = SdfValueOr<std::string>(_sdf, "robot_name", "burger");
    this->robotRadius_ = SdfValueOr<double>(_sdf, "robot_radius", 0.35);
    this->humanRadius_ = SdfValueOr<double>(_sdf, "human_radius", 0.28);
    this->proxemicFront_ = SdfValueOr<double>(_sdf, "proxemic_margin_front", 1.2);
    this->proxemicSide_ = SdfValueOr<double>(_sdf, "proxemic_margin_side", 0.6);
    this->reactionLatency_ = SdfValueOr<double>(_sdf, "reaction_latency", 0.3);
    this->socialYieldDistance_ = SdfValueOr<double>(_sdf, "social_yield_distance", 3.0);

    this->timeStep_ = SdfValueOr<double>(_sdf, "time_step", 0.004);
    this->neighborDist_ = SdfValueOr<double>(_sdf, "neighbor_dist", 10.0);
    this->maxNeighbors_ = static_cast<std::size_t>(SdfValueOr<int>(_sdf, "max_neighbors", 10));
    this->timeHorizon_ = SdfValueOr<double>(_sdf, "time_horizon", 4.0);
    this->timeHorizonObst_ = SdfValueOr<double>(_sdf, "time_horizon_obst", 3.0);
    this->maxSpeed_ = SdfValueOr<double>(_sdf, "max_speed", 1.2);

    this->scenarioTopic_ = SdfValueOr<std::string>(_sdf, "scenario_topic", "/rvo2_human/scenario");
    this->commandTopic_ = SdfValueOr<std::string>(_sdf, "command_topic", "/rvo2_human/command");
    this->groundTruthTopic_ = SdfValueOr<std::string>(_sdf, "ground_truth_topic", "/rvo2_human/ground_truth");
    this->defaultScenarioFile_ = SdfValueOr<std::string>(_sdf, "default_scenario_file", "default");

    this->visualZ_ = SdfValueOr<double>(_sdf, "visual_z", 1.0);
    this->proxyZ_ = SdfValueOr<double>(_sdf, "proxy_z", 0.85);
    this->hiddenZ_ = SdfValueOr<double>(_sdf, "hidden_z", -50.0);
    this->animSpeedFactor_ = SdfValueOr<double>(_sdf, "animation_speed_factor", 4.1534);
    this->politenessBalancePoint_ = SdfValueOr<double>(_sdf, "politeness_balance_point", 0.6);
    this->randomizePoliteness_ = SdfValueOr<bool>(_sdf, "randomize_politeness", true);

    // Corridor and Recess configuration
    this->corridorMode_ = SdfValueOr<bool>(_sdf, "corridor_mode", false);
    this->corridorHalfWidth_ = SdfValueOr<double>(_sdf, "corridor_half_width", 0.45);
    this->recessXMin_ = SdfValueOr<double>(_sdf, "recess_x_min", 1.35);
    this->recessXMax_ = SdfValueOr<double>(_sdf, "recess_x_max", 2.65);
    this->recessXCenter_ = SdfValueOr<double>(_sdf, "recess_x_center", 2.00);
    this->recessY_ = SdfValueOr<double>(_sdf, "recess_y", 0.75);
    this->recessApproachDist_ = SdfValueOr<double>(_sdf, "recess_approach_dist", 3.5);
    this->recessThreshold_ = SdfValueOr<double>(_sdf, "recess_threshold", 0.40);

    // Initialize humans
    this->humans_.clear();
    for (int i = 1; i <= this->humanCount_; ++i)
    {
      HumanAgent h;
      h.id = i;
      h.name = "human_" + TwoDigit(i);
      h.radius = this->humanRadius_;
      h.latencyTau = this->reactionLatency_;
      h.politeness = this->randomizePoliteness_ ? this->SampleRandomPoliteness() : 0.6;
      if (i == 1) { h.visualZ = 1.02; h.proxyZ = 0.85; h.scale = 1.02; }
      else if (i == 2) { h.visualZ = 0.95; h.proxyZ = 0.80; h.scale = 0.95; }
      else if (i == 3) { h.visualZ = 1.06; h.proxyZ = 0.875; h.scale = 1.06; }
      else { h.visualZ = this->visualZ_; h.proxyZ = this->proxyZ_; h.scale = 1.00; }
      this->humans_.push_back(h);
    }

    // Initialize RVO Simulator
    this->InitRVOSimulator();

    // Transport subscriptions & advertisements
    this->node_.Subscribe(this->scenarioTopic_, &RVO2HumanSystem::OnScenarioMessage, this);
    this->node_.Subscribe(this->commandTopic_, &RVO2HumanSystem::OnCommandMessage, this);
    this->gtPublisher_ = this->node_.Advertise<gz::msgs::StringMsg>(this->groundTruthTopic_);

    // Load default scenario
    this->LoadDefaultScenario();

    gzmsg << "[RVO2HumanSystem] Configured with " << this->humanCount_
          << " humans. Robot: '" << this->robotName_
          << "', NeighborDist: " << this->neighborDist_ << "m\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->entitiesResolved_)
    {
      this->ResolveEntities(_ecm);
      this->ApplyAllPoses(_ecm);
      this->entitiesResolved_ = true;
    }

    // Process transport messages
    this->ProcessPendingMessages(_ecm);

    if (_info.paused || !this->running_)
    {
      this->ApplyAllPoses(_ecm);
      return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    // 1. Update Robot in RVO Simulator
    this->UpdateRobotState(_ecm, dt);

    // 2. Compute Desired Velocities with Social Proxemics
    for (auto &human : this->humans_)
    {
      if (!human.active)
        continue;

      this->ComputePreferredVelocity(human, dt);
    }

    // 3. Step RVO Simulation
    this->rvoSim_->setTimeStep(static_cast<float>(dt));
    this->rvoSim_->doStep();

    // 4. Update Agents from RVO results
    for (auto &human : this->humans_)
    {
      if (!human.active)
        continue;

      const RVO::Vector2 pos = this->rvoSim_->getAgentPosition(human.rvoIndex);
      const RVO::Vector2 vel = this->rvoSim_->getAgentVelocity(human.rvoIndex);

      human.x = pos.x();
      human.y = pos.y();
      human.vx = vel.x();
      human.vy = vel.y();

      // In corridor mode, clamp Y to guarantee agent never penetrates corridor or recess walls
      if (this->corridorMode_)
      {
        const double r = human.radius;
        const double minWallY = -this->corridorHalfWidth_ + r;
        double maxWallY = this->corridorHalfWidth_ - r;
        if (human.x >= (this->recessXMin_ - 0.1) && human.x <= (this->recessXMax_ + 0.1))
        {
          maxWallY = this->recessY_ + 0.10;
        }
        human.y = std::clamp(human.y, minWallY, maxWallY);
        this->rvoSim_->setAgentPosition(human.rvoIndex, RVO::Vector2(human.x, human.y));
      }

      const double speed = std::hypot(human.vx, human.vy);
      double targetYaw = human.yaw;
      if (speed > 0.08)
      {
        targetYaw = std::atan2(human.vy, human.vx);
      }
      else if (!human.waypoints.empty() && human.mode == HumanMode::Autonomous)
      {
        const auto &wp = human.waypoints[human.waypointIndex];
        if (Distance2D(human.x, human.y, wp.x, wp.y) > 0.1)
        {
          targetYaw = HeadingTo(human.x, human.y, wp.x, wp.y);
        }
      }

      // Smooth rotation towards target yaw (up to 8.0 rad/s ~ 460 deg/s)
      const double diff = NormalizeAngle(targetYaw - human.yaw);
      human.yaw = NormalizeAngle(human.yaw + std::clamp(diff, -8.0 * dt, 8.0 * dt));

      if (speed > 0.05)
      {
        // Synchronize gait cadence with linear travel velocity:
        // walk.dae mocap clip covers 1.3844m stride across 5.75s (v_mocap = 0.2408 m/s).
        // Stride scales with character height/scale (s).
        // animRate = 5.75 / (1.3844 * s) = 4.1534 / s [seconds of animation per meter traveled].
        const double s = std::max(0.5, human.scale);
        const double animRate = (this->animSpeedFactor_ > 0.0) ? (this->animSpeedFactor_ / s) : (4.1534 / s);
        human.animTimeSec += speed * animRate * dt;
        human.animationTime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(human.animTimeSec));
      }
    }

    // 5. Apply authoritative poses to Gazebo ECM
    this->ApplyAllPoses(_ecm);

    // 6. Publish Ground-Truth
    this->PublishGroundTruth(_info.simTime);
  }

  void Reset(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    this->running_ = false;
    this->ResetScenarioRuntime();
    this->ApplyAllPoses(_ecm);
    gzmsg << "[RVO2HumanSystem] Reset handled.\n";
  }

private:
  double SampleRandomPoliteness()
  {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    const double r = dist(this->rng_);
    if (r > this->politenessBalancePoint_)
    {
      // Distracted / non-yielding persona (0.0 to 0.20)
      std::uniform_real_distribution<double> sub(0.0, 0.20);
      return sub(this->rng_);
    }
    else
    {
      // Cooperative / polite persona (0.50 to 1.0)
      std::uniform_real_distribution<double> sub(0.50, 1.0);
      return sub(this->rng_);
    }
  }

  void InitRVOSimulator()
  {
    this->rvoSim_ = std::make_unique<RVO::RVOSimulator>(
        static_cast<float>(this->timeStep_),
        static_cast<float>(this->neighborDist_),
        this->maxNeighbors_,
        static_cast<float>(this->timeHorizon_),
        static_cast<float>(this->timeHorizonObst_),
        static_cast<float>(this->humanRadius_),
        static_cast<float>(this->maxSpeed_));

    // Agent 0 is reserved for the Robot
    this->robotRvoIndex_ = this->rvoSim_->addAgent(
        RVO::Vector2(1000.0f, 1000.0f),
        static_cast<float>(this->neighborDist_),
        this->maxNeighbors_,
        static_cast<float>(this->timeHorizon_ * 1.5), // larger horizon for robot
        static_cast<float>(this->timeHorizonObst_),
        static_cast<float>(this->robotRadius_),
        static_cast<float>(this->maxSpeed_));

    // Register humans
    for (auto &human : this->humans_)
    {
      human.rvoIndex = this->rvoSim_->addAgent(
          RVO::Vector2(static_cast<float>(human.x), static_cast<float>(human.y)),
          static_cast<float>(this->neighborDist_),
          this->maxNeighbors_,
          static_cast<float>(this->timeHorizon_),
          static_cast<float>(this->timeHorizonObst_),
          static_cast<float>(human.radius),
          static_cast<float>(human.maxSpeed));
    }
  }

  void ResolveEntities(gz::sim::EntityComponentManager &_ecm)
  {
    // 1. Resolve Robot
    this->robotEntity_ = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(this->robotName_));

    // 2. Resolve Humans
    for (auto &human : this->humans_)
    {
      const std::string name = human.name;
      // Try unified single model first: "human_01"
      human.modelEntity = _ecm.EntityByComponents(
          gz::sim::components::Model(),
          gz::sim::components::Name(name));

      if (human.modelEntity != gz::sim::kNullEntity)
      {
        human.isSingleModel = true;
      }
      else
      {
        // Try proxy model: "human_01_proxy"
        human.modelEntity = _ecm.EntityByComponents(
            gz::sim::components::Model(),
            gz::sim::components::Name(name + "_proxy"));
        human.isSingleModel = false;
      }

      // Try actor: "human_01_visual" or "human_01_actor" using Each
      human.actorEntity = gz::sim::kNullEntity;
      _ecm.Each<gz::sim::components::Name, gz::sim::components::Actor>(
          [&](const gz::sim::Entity &entity,
              const gz::sim::components::Name *compName,
              const gz::sim::components::Actor *) -> bool
          {
            if (compName && (compName->Data() == name + "_visual" ||
                             compName->Data() == name + "_actor"))
            {
              human.actorEntity = entity;
              return false;
            }
            return true;
          });

      if (human.actorEntity != gz::sim::kNullEntity)
      {
        human.actorOrigin = gz::sim::worldPose(human.actorEntity, _ecm);
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(human.actorEntity);
        if (poseComp)
        {
          *poseComp = gz::sim::components::Pose(gz::math::Pose3d::Zero);
          _ecm.SetChanged(human.actorEntity, gz::sim::components::Pose::typeId,
                          gz::sim::ComponentState::OneTimeChange);
        }
      }
    }
  }

  void UpdateRobotState(gz::sim::EntityComponentManager &_ecm, double _dt)
  {
    if (this->robotEntity_ == gz::sim::kNullEntity)
    {
      this->robotEntity_ = _ecm.EntityByComponents(
          gz::sim::components::Model(),
          gz::sim::components::Name(this->robotName_));
    }

    if (this->robotEntity_ != gz::sim::kNullEntity)
    {
      const auto pose = gz::sim::worldPose(this->robotEntity_, _ecm).Pos();
      const double currentX = pose.X();
      const double currentY = pose.Y();

      double vx = 0.0;
      double vy = 0.0;
      if (_dt > 1e-5 && this->robotHasPrevPose_)
      {
        vx = (currentX - this->robotPrevX_) / _dt;
        vy = (currentY - this->robotPrevY_) / _dt;
      }

      this->robotPrevX_ = currentX;
      this->robotPrevY_ = currentY;
      this->robotHasPrevPose_ = true;

      this->rvoSim_->setAgentPosition(this->robotRvoIndex_, RVO::Vector2(currentX, currentY));
      this->rvoSim_->setAgentVelocity(this->robotRvoIndex_, RVO::Vector2(vx, vy));
      this->rvoSim_->setAgentPrefVelocity(this->robotRvoIndex_, RVO::Vector2(vx, vy));
    }
  }

  void ComputePreferredVelocity(HumanAgent &_human, double _dt)
  {
    if (_human.mode == HumanMode::Stationary || _human.mode == HumanMode::Hidden)
    {
      _human.prefVx = 0.0;
      _human.prefVy = 0.0;
      this->rvoSim_->setAgentPrefVelocity(_human.rvoIndex, RVO::Vector2(0, 0));
      return;
    }

    if (_human.waypoints.empty())
    {
      this->rvoSim_->setAgentPrefVelocity(_human.rvoIndex, RVO::Vector2(0, 0));
      return;
    }

    // Advance waypoints
    const auto &target = _human.waypoints[_human.waypointIndex];
    const double distToGoal = Distance2D(_human.x, _human.y, target.x, target.y);

    if (distToGoal <= _human.waypointTolerance)
    {
      if (_human.waypointIndex + 1 < _human.waypoints.size())
      {
        _human.waypointIndex++;
      }
      else if (_human.loop)
      {
        _human.waypointIndex = 0;
      }
      else
      {
        _human.prefVx = 0.0;
        _human.prefVy = 0.0;
        this->rvoSim_->setAgentPrefVelocity(_human.rvoIndex, RVO::Vector2(0, 0));
        return;
      }
    }

    const auto &currTarget = _human.waypoints[_human.waypointIndex];
    const double dx = currTarget.x - _human.x;
    const double dy = currTarget.y - _human.y;
    const double d = std::hypot(dx, dy);

    double desiredVx = 0.0;
    double desiredVy = 0.0;
    if (d > kEps)
    {
      const double targetHeading = std::atan2(dy, dx);
      const double headingDiff = NormalizeAngle(targetHeading - _human.yaw);
      // Reduce forward velocity during turnaround so human pivots naturally without backward motion
      const double turnFactor = std::max(0.15, std::cos(headingDiff));

      desiredVx = (dx / d) * _human.speed * turnFactor;
      desiredVy = (dy / d) * _human.speed * turnFactor;
    }

    // Corridor Recess Yielding State Machine vs Open Arena Proxemic Yielding
    if (this->corridorMode_ && this->robotHasPrevPose_)
    {
      const double rx = this->robotPrevX_ - _human.x;
      const double ry = this->robotPrevY_ - _human.y;
      const double distToRobot = std::hypot(rx, ry);

      if (_human.corridorState == CorridorState::Patrol)
      {
        // Check if robot is approaching in front and human is near the recess zone
        const bool robotInFront = (desiredVx * rx > 0.0);
        const bool nearRecess = (_human.x >= (this->recessXMin_ - 0.4) && _human.x <= (this->recessXMax_ + 1.2));

        if (robotInFront && distToRobot < this->recessApproachDist_ && nearRecess)
        {
          // Stochastic decision based on politeness balance point
          if (_human.politeness >= this->recessThreshold_)
          {
            _human.corridorState = CorridorState::YieldingEnter;
          }
        }
      }
      else if (_human.corridorState == CorridorState::YieldingEnter)
      {
        const double targetX = this->recessXCenter_;
        const double targetY = this->recessY_;
        const double dxRecess = targetX - _human.x;
        const double dyRecess = targetY - _human.y;
        const double dRecess = std::hypot(dxRecess, dyRecess);

        if (dRecess < 0.22)
        {
          _human.corridorState = CorridorState::YieldingWait;
          desiredVx = 0.0;
          desiredVy = 0.0;
        }
        else
        {
          desiredVx = (dxRecess / dRecess) * _human.speed * 0.85;
          desiredVy = (dyRecess / dRecess) * _human.speed * 0.85;
        }
      }
      else if (_human.corridorState == CorridorState::YieldingWait)
      {
        desiredVx = 0.0;
        desiredVy = 0.0;
        // Human turns to face the corridor (-pi/2) while waiting for robot to pass
        const double targetHeading = -1.5708;
        const double headingDiff = NormalizeAngle(targetHeading - _human.yaw);
        _human.yaw = NormalizeAngle(_human.yaw + std::clamp(headingDiff, -4.0 * _dt, 4.0 * _dt));

        // Check if robot has cleared the bottleneck
        const bool robotCleared = (this->robotPrevX_ > (_human.x + 0.6)) || (distToRobot > 3.0 && this->robotPrevX_ > _human.x);
        if (robotCleared)
        {
          _human.corridorState = CorridorState::YieldingExit;
        }
      }
      else if (_human.corridorState == CorridorState::YieldingExit)
      {
        const double targetX = this->recessXCenter_;
        const double targetY = 0.0;
        const double dxExit = targetX - _human.x;
        const double dyExit = targetY - _human.y;
        const double dExit = std::hypot(dxExit, dyExit);

        if (std::abs(_human.y) < 0.12)
        {
          _human.corridorState = CorridorState::Patrol;
        }
        else
        {
          desiredVx = (dxExit / std::max(0.01, dExit)) * _human.speed * 0.85;
          desiredVy = (dyExit / std::max(0.01, dExit)) * _human.speed * 0.85;
        }
      }
    }
    else if (_human.politeness > 0.05 && this->robotHasPrevPose_)
    {
      const double rx = this->robotPrevX_ - _human.x;
      const double ry = this->robotPrevY_ - _human.y;
      const double distToRobot = std::hypot(rx, ry);

      if (distToRobot < this->socialYieldDistance_ && distToRobot > 0.1)
      {
        // Check if robot is in front of human
        const double dot = (desiredVx * rx + desiredVy * ry) / (_human.speed * distToRobot);
        if (dot > 0.2) // Robot is generally in front / approaching
        {
          // Lateral nudge (step to human's right side) scaled by politeness
          const double normRx = rx / distToRobot;
          const double normRy = ry / distToRobot;
          // Perpendicular right vector: (normRy, -normRx)
          const double lateralStrength = 0.45 * _human.politeness * (1.0 - distToRobot / this->socialYieldDistance_);
          desiredVx += normRy * lateralStrength * _human.speed;
          desiredVy += -normRx * lateralStrength * _human.speed;

          // Slow down if very close (yielding) scaled by politeness
          if (distToRobot < this->proxemicFront_)
          {
            const double slowFactor = 1.0 - _human.politeness * (1.0 - std::max(0.2, distToRobot / this->proxemicFront_));
            desiredVx *= slowFactor;
            desiredVy *= slowFactor;
          }
        }
      }
    }

    // Reaction Latency: Low-pass filter velocity update
    const double alpha = std::clamp(_dt / std::max(0.01, _human.latencyTau), 0.0, 1.0);
    _human.prefVx += alpha * (desiredVx - _human.prefVx);
    _human.prefVy += alpha * (desiredVy - _human.prefVy);

    this->rvoSim_->setAgentPrefVelocity(
        _human.rvoIndex,
        RVO::Vector2(static_cast<float>(_human.prefVx), static_cast<float>(_human.prefVy)));
  }

  void ApplyAllPoses(gz::sim::EntityComponentManager &_ecm)
  {
    for (auto &human : this->humans_)
    {
      const bool visible = (human.active && human.mode != HumanMode::Hidden);

      if (human.modelEntity != gz::sim::kNullEntity)
      {
        const double modelZ = visible ? human.proxyZ : this->hiddenZ_;
        gz::sim::Model model(human.modelEntity);
        model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(human.x, human.y, modelZ, 0.0, 0.0, human.yaw));
      }

      if (human.actorEntity != gz::sim::kNullEntity)
      {
        gz::sim::Actor actor(human.actorEntity);
        if (actor.Valid(_ecm))
        {
          const double actorZ = visible ? human.visualZ : this->hiddenZ_;
          actor.SetTrajectoryPose(_ecm, gz::math::Pose3d(human.x, human.y, actorZ, 0.0, 0.0, human.yaw));
          actor.SetAnimationName(_ecm, "walk");
          actor.SetAnimationTime(_ecm, human.animationTime);

          // Gazebo 8 Harmonic rendering (SceneBroadcaster / Ogre2) must be notified of actor changes
          _ecm.SetChanged(human.actorEntity, gz::sim::components::TrajectoryPose::typeId,
                          gz::sim::ComponentState::OneTimeChange);
          _ecm.SetChanged(human.actorEntity, gz::sim::components::AnimationTime::typeId,
                          gz::sim::ComponentState::OneTimeChange);
        }
      }
    }
  }

  void PublishGroundTruth(const std::chrono::steady_clock::duration &_simTime)
  {
    const double tSec = std::chrono::duration<double>(_simTime).count();
    json payload;
    payload["timestamp"] = tSec;
    payload["corridor_mode"] = this->corridorMode_;
    payload["humans"] = json::array();

    for (const auto &human : this->humans_)
    {
      if (!human.active || human.mode == HumanMode::Hidden)
        continue;

      json hObj;
      hObj["id"] = human.id;
      hObj["x"] = human.x;
      hObj["y"] = human.y;
      hObj["yaw"] = human.yaw;
      hObj["vx"] = human.vx;
      hObj["vy"] = human.vy;
      hObj["speed"] = std::hypot(human.vx, human.vy);
      hObj["politeness"] = human.politeness;
      hObj["yielding"] = (human.corridorState == CorridorState::YieldingEnter ||
                          human.corridorState == CorridorState::YieldingWait);
      std::string stateStr = "patrol";
      if (human.corridorState == CorridorState::YieldingEnter) stateStr = "yielding_enter";
      else if (human.corridorState == CorridorState::YieldingWait) stateStr = "yielding_wait";
      else if (human.corridorState == CorridorState::YieldingExit) stateStr = "yielding_exit";
      hObj["state"] = stateStr;

      payload["humans"].push_back(hObj);
    }

    gz::msgs::StringMsg msg;
    msg.set_data(payload.dump());
    this->gtPublisher_.Publish(msg);
  }

  void OnScenarioMessage(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lock(this->msgMutex_);
    this->pendingScenario_ = _msg.data();
    this->hasPendingScenario_ = true;
  }

  void OnCommandMessage(const gz::msgs::StringMsg &_msg)
  {
    std::lock_guard<std::mutex> lock(this->msgMutex_);
    this->pendingCommand_ = _msg.data();
    this->hasPendingCommand_ = true;
  }

  void ProcessPendingMessages(gz::sim::EntityComponentManager &_ecm)
  {
    std::optional<std::string> scenario;
    std::optional<std::string> cmd;
    {
      std::lock_guard<std::mutex> lock(this->msgMutex_);
      if (this->hasPendingScenario_)
      {
        scenario = this->pendingScenario_;
        this->hasPendingScenario_ = false;
      }
      if (this->hasPendingCommand_)
      {
        cmd = this->pendingCommand_;
        this->hasPendingCommand_ = false;
      }
    }

    if (scenario)
    {
      this->LoadScenarioJson(*scenario);
      this->ResetScenarioRuntime();
      this->ApplyAllPoses(_ecm);
    }

    if (cmd)
    {
      if (*cmd == "start")
        this->running_ = true;
      else if (*cmd == "stop")
        this->running_ = false;
      else if (*cmd == "reset")
      {
        this->running_ = false;
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);
      }
      else if (*cmd == "reset_start")
      {
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);
        this->running_ = true;
      }
    }
  }

  void LoadScenarioJson(const std::string &_jsonStr)
  {
    try
    {
      const auto root = json::parse(_jsonStr);
      if (root.contains("humans") && root["humans"].is_array())
      {
        for (const auto &item : root["humans"])
        {
          const int id = item.value("id", 0);
          if (id <= 0 || id > static_cast<int>(this->humans_.size()))
            continue;

          auto &h = this->humans_[static_cast<std::size_t>(id - 1)];
          h.active = true;
          const std::string modeStr = item.value("mode", "autonomous");
          if (modeStr == "stationary")
            h.mode = HumanMode::Stationary;
          else if (modeStr == "hidden")
            h.mode = HumanMode::Hidden;
          else
            h.mode = HumanMode::Autonomous;

          h.speed = item.value("speed", 0.85);
          h.maxSpeed = item.value("max_speed", 1.2);
          if (item.contains("politeness"))
          {
            h.politeness = std::clamp(item["politeness"].get<double>(), 0.0, 1.0);
          }
          else if (item.contains("social_polite"))
          {
            h.politeness = item["social_polite"].get<bool>() ? 0.8 : 0.0;
          }
          else if (this->randomizePoliteness_)
          {
            h.politeness = this->SampleRandomPoliteness();
          }
          h.visualZ = item.value("visual_z", h.visualZ);
          h.proxyZ = item.value("proxy_z", h.proxyZ);
          h.scale = item.value("scale", h.scale);

          h.waypoints.clear();
          if (item.contains("waypoints") && item["waypoints"].is_array())
          {
            for (const auto &wp : item["waypoints"])
            {
              if (wp.is_array() && wp.size() >= 2)
              {
                h.waypoints.push_back({wp[0].get<double>(), wp[1].get<double>()});
              }
            }
          }

          if (item.contains("start") && item["start"].is_object())
          {
            h.x = item["start"].value("x", 0.0);
            h.y = item["start"].value("y", 0.0);
            h.yaw = item["start"].value("yaw", 0.0);
          }
          else if (!h.waypoints.empty())
          {
            h.x = h.waypoints[0].x;
            h.y = h.waypoints[0].y;
            if (h.waypoints.size() >= 2)
            {
              h.yaw = HeadingTo(h.x, h.y, h.waypoints[1].x, h.waypoints[1].y);
            }
          }

          // Update initial position in RVO
          this->rvoSim_->setAgentPosition(h.rvoIndex, RVO::Vector2(h.x, h.y));
        }
      }
    }
    catch (const std::exception &e)
    {
      gzerr << "[RVO2HumanSystem] JSON parse error: " << e.what() << "\n";
    }
  }

  void LoadDefaultScenario()
  {
    if (this->corridorMode_)
    {
      std::string jsonStr = R"({
        "episode_id": 1,
        "corridor_mode": true,
        "humans": [
          {"id": 1, "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[5.0, 0.0], [-3.0, 0.0]]}
        ]
      })";
      this->LoadScenarioJson(jsonStr);
      return;
    }

    // Built-in default patrol for 4 dynamic agents - all active and moving with calibrated scale & tempo
    std::string jsonStr = R"({
      "episode_id": 1,
      "humans": [
        {"id": 1, "mode": "autonomous", "speed": 0.75, "scale": 1.02, "visual_z": 1.02, "proxy_z": 0.85, "waypoints": [[4.0, 0.0], [-10.0, 0.0]]},
        {"id": 2, "mode": "autonomous", "speed": 0.85, "scale": 0.95, "visual_z": 0.95, "proxy_z": 0.80, "waypoints": [[-8.0, -1.2], [3.0, 1.0]]},
        {"id": 3, "mode": "autonomous", "speed": 0.70, "scale": 1.06, "visual_z": 1.06, "proxy_z": 0.875, "waypoints": [[-3.0, -3.0], [-3.0, 3.0]]},
        {"id": 4, "mode": "autonomous", "speed": 0.80, "scale": 1.00, "visual_z": 1.00, "proxy_z": 0.85, "waypoints": [[1.0, 2.5], [-7.0, -2.0]]}
      ]
    })";
    this->LoadScenarioJson(jsonStr);
  }

  void ResetScenarioRuntime()
  {
    for (auto &human : this->humans_)
    {
      human.Reset();
      if (this->randomizePoliteness_)
      {
        human.politeness = this->SampleRandomPoliteness();
      }
      if (!human.waypoints.empty())
      {
        human.x = human.waypoints[0].x;
        human.y = human.waypoints[0].y;
        if (human.waypoints.size() >= 2)
        {
          human.yaw = HeadingTo(human.x, human.y, human.waypoints[1].x, human.waypoints[1].y);
        }
      }
      this->rvoSim_->setAgentPosition(human.rvoIndex, RVO::Vector2(human.x, human.y));
      this->rvoSim_->setAgentVelocity(human.rvoIndex, RVO::Vector2(0, 0));
      this->rvoSim_->setAgentPrefVelocity(human.rvoIndex, RVO::Vector2(0, 0));
    }
  }

  // Member variables
  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};
  std::string robotName_{"burger"};
  double robotRadius_{0.35};
  double humanRadius_{0.28};
  double proxemicFront_{1.2};
  double proxemicSide_{0.6};
  double reactionLatency_{0.3};
  double socialYieldDistance_{3.0};

  double timeStep_{0.004};
  double neighborDist_{10.0};
  std::size_t maxNeighbors_{10};
  double timeHorizon_{4.0};
  double timeHorizonObst_{3.0};
  double maxSpeed_{1.2};

  double visualZ_{1.0};
  double proxyZ_{0.85};
  double hiddenZ_{-50.0};
  double animSpeedFactor_{4.1534};
  double politenessBalancePoint_{0.6};
  bool randomizePoliteness_{true};
  std::mt19937 rng_{std::random_device{}()};

  // Corridor parameters
  bool corridorMode_{false};
  double corridorHalfWidth_{0.45};
  double recessXMin_{1.35};
  double recessXMax_{2.65};
  double recessXCenter_{2.00};
  double recessY_{0.75};
  double recessApproachDist_{3.5};
  double recessThreshold_{0.40};

  int humanCount_{4};
  std::vector<HumanAgent> humans_;
  std::unique_ptr<RVO::RVOSimulator> rvoSim_;
  std::size_t robotRvoIndex_{0};

  double robotPrevX_{0.0};
  double robotPrevY_{0.0};
  bool robotHasPrevPose_{false};

  bool running_{true};
  bool entitiesResolved_{false};

  gz::transport::Node node_;
  gz::transport::Node::Publisher gtPublisher_;
  std::string scenarioTopic_;
  std::string commandTopic_;
  std::string groundTruthTopic_;
  std::string defaultScenarioFile_;

  std::mutex msgMutex_;
  std::string pendingScenario_;
  std::string pendingCommand_;
  bool hasPendingScenario_{false};
  bool hasPendingCommand_{false};
};

} // namespace custom_corridor

GZ_ADD_PLUGIN(
    custom_corridor::RVO2HumanSystem,
    gz::sim::System,
    custom_corridor::RVO2HumanSystem::ISystemConfigure,
    custom_corridor::RVO2HumanSystem::ISystemPreUpdate,
    custom_corridor::RVO2HumanSystem::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
    custom_corridor::RVO2HumanSystem,
    "custom_corridor::RVO2HumanSystem")
