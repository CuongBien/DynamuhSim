// multi_human_scenario_system.cpp
//
// Gazebo Sim 8 / Harmonic world system plugin for synchronized multi-human motion.
//
// Controls pairs:
//   human_01_visual (Actor) + human_01_proxy (Model)
//   human_02_visual (Actor) + human_02_proxy (Model)
//   ...
//
// Transport interface:
//   /multi_human/scenario   gz.msgs.StringMsg  JSON scenario
//   /multi_human/command    gz.msgs.StringMsg  reset|start|stop|reset_start|hide_all
//
// Important:
// - Actor visuals are moved with Actor::SetTrajectoryPose().
// - Collision proxies are moved with Model::SetWorldPoseCmd().
// - All ECM writes happen in PreUpdate(), never in transport callback threads.
// - Scenario time uses Gazebo simulation dt, not wall-clock time.
//
// Dependencies:
//   Gazebo Harmonic: gz-sim8, gz-transport13, gz-msgs10
//   nlohmann-json3-dev
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
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
#include <gz/transport/Node.hh>
#include <nlohmann/json.hpp>
#include <sdf/Element.hh>

namespace custom_corridor
{
namespace
{
using json = nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double NormalizeAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double Distance(double x0, double y0, double x1, double y1)
{
  return std::hypot(x1 - x0, y1 - y0);
}

double HeadingTo(double x0, double y0, double x1, double y1)
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
  Autonomous,
  Scripted
};

std::string ModeName(HumanMode mode)
{
  switch (mode)
  {
    case HumanMode::Hidden:
      return "hidden";
    case HumanMode::Stationary:
      return "stationary";
    case HumanMode::Autonomous:
      return "autonomous";
    case HumanMode::Scripted:
      return "scripted";
  }
  return "unknown";
}

HumanMode ParseMode(const std::string &value)
{
  if (value == "hidden")
    return HumanMode::Hidden;
  if (value == "stationary")
    return HumanMode::Stationary;
  if (value == "autonomous")
    return HumanMode::Autonomous;
  if (value == "scripted")
    return HumanMode::Scripted;

  throw std::runtime_error(
      "Unsupported human mode '" + value +
      "'. Expected hidden/stationary/autonomous/scripted.");
}

struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

struct ScriptSegment
{
  double duration{0.0};
  double speed{0.0};
  double heading{0.0};
};

struct HumanState
{
  int id{0};

  HumanMode mode{HumanMode::Hidden};
  std::string behavior;

  bool active{false};

  // Initial episode state.
  double initialX{0.0};
  double initialY{0.0};
  double initialYaw{0.0};

  // Current runtime state.
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};

  // Optional simple scripted duration.
  std::optional<double> duration;
  double totalElapsed{0.0};

  // Autonomous mode.
  std::vector<Waypoint> waypoints;
  std::size_t waypointIndex{0};
  bool loop{true};
  double waypointTolerance{0.10};

  // Scripted segments.
  std::vector<ScriptSegment> segments;
  std::size_t segmentIndex{0};
  double segmentElapsed{0.0};

  // Entity bindings.
  gz::sim::Entity visualEntity{gz::sim::kNullEntity};
  gz::sim::Entity proxyEntity{gz::sim::kNullEntity};

  // V4: proxy is the required physical + visible human representation.
  // Actor is optional cosmetic animation only.
  bool entitiesResolved{false};
  bool actorAvailable{false};

  // Actor trajectory pose is relative to this SDF / actor origin.
  gz::math::Pose3d actorOrigin{gz::math::Pose3d::Zero};

  // Deterministic walk animation time for this episode.
  std::chrono::steady_clock::duration animationTime{
      std::chrono::steady_clock::duration::zero()};

  void ResetRuntime()
  {
    this->x = this->initialX;
    this->y = this->initialY;
    this->yaw = this->initialYaw;

    this->totalElapsed = 0.0;
    this->segmentIndex = 0;
    this->segmentElapsed = 0.0;
    this->animationTime = std::chrono::steady_clock::duration::zero();

    // For autonomous mode, waypointIndex is configured at parse time.
    // It is restored from resetWaypointIndex below.
    this->waypointIndex = this->resetWaypointIndex;
  }

  std::size_t resetWaypointIndex{0};
};

HumanState ParseHuman(
    const json &_human,
    int _humanCount)
{
  HumanState h;

  if (!_human.contains("id"))
    throw std::runtime_error("Each human requires integer field 'id'.");

  h.id = _human.at("id").get<int>();

  if (h.id < 1 || h.id > _humanCount)
  {
    throw std::runtime_error(
        "Human id " + std::to_string(h.id) +
        " outside configured range [1, " +
        std::to_string(_humanCount) + "].");
  }

  const std::string modeStr =
      _human.value("mode", std::string("stationary"));
  h.mode = ParseMode(modeStr);
  h.active = (h.mode != HumanMode::Hidden);
  h.behavior = _human.value("behavior", std::string());

  h.speed = _human.value("speed", 0.0);
  if (h.speed < 0.0)
    throw std::runtime_error("Human speed must be >= 0.");

  bool hasStart = false;
  if (_human.contains("start"))
  {
    const auto &start = _human.at("start");
    h.initialX = start.value("x", 0.0);
    h.initialY = start.value("y", 0.0);
    h.initialYaw = start.value("yaw", 0.0);
    hasStart = true;
  }

  h.x = h.initialX;
  h.y = h.initialY;
  h.yaw = h.initialYaw;

  if (_human.contains("duration"))
  {
    const double d = _human.at("duration").get<double>();
    if (d < 0.0)
      throw std::runtime_error("Human duration must be >= 0.");
    h.duration = d;
  }

  if (h.mode == HumanMode::Autonomous)
  {
    if (!_human.contains("waypoints") ||
        !_human.at("waypoints").is_array() ||
        _human.at("waypoints").size() < 2)
    {
      throw std::runtime_error(
          "Autonomous human " + std::to_string(h.id) +
          " requires at least 2 waypoints.");
    }

    for (const auto &wp : _human.at("waypoints"))
    {
      if (!wp.is_array() || wp.size() < 2)
      {
        throw std::runtime_error(
            "Waypoint must be [x, y].");
      }

      h.waypoints.push_back(
          Waypoint{
              wp.at(0).get<double>(),
              wp.at(1).get<double>()});
    }

    h.loop = _human.value("loop", true);
    h.waypointTolerance =
        _human.value("waypoint_tolerance", 0.10);

    if (h.waypointTolerance <= 0.0)
      throw std::runtime_error("waypoint_tolerance must be > 0.");

    if (!hasStart)
    {
      // Start at the first waypoint and move toward waypoint 2.
      h.initialX = h.waypoints.front().x;
      h.initialY = h.waypoints.front().y;
      h.initialYaw = HeadingTo(
          h.waypoints[0].x,
          h.waypoints[0].y,
          h.waypoints[1].x,
          h.waypoints[1].y);

      h.x = h.initialX;
      h.y = h.initialY;
      h.yaw = h.initialYaw;

      h.waypointIndex = 1;
      h.resetWaypointIndex = 1;
    }
    else
    {
      // Start from explicit pose and target waypoint 0.
      h.waypointIndex = 0;
      h.resetWaypointIndex = 0;
    }
  }

  if (h.mode == HumanMode::Scripted &&
      _human.contains("segments"))
  {
    const auto &segments = _human.at("segments");

    if (!segments.is_array())
      throw std::runtime_error("'segments' must be an array.");

    for (const auto &segment : segments)
    {
      ScriptSegment seg;
      seg.duration = segment.value("duration", 0.0);
      seg.speed = segment.value("speed", h.speed);
      seg.heading = segment.value("heading", h.initialYaw);

      if (seg.duration <= 0.0)
        throw std::runtime_error(
            "Every scripted segment requires duration > 0.");
      if (seg.speed < 0.0)
        throw std::runtime_error(
            "Scripted segment speed must be >= 0.");

      h.segments.push_back(seg);
    }
  }

  h.ResetRuntime();
  return h;
}

}  // namespace


class MultiHumanScenarioSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
{
  public:
    MultiHumanScenarioSystem() = default;

  public:
    ~MultiHumanScenarioSystem() override = default;

  public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &,
        gz::sim::EventManager &) override
    {
      this->worldEntity_ = _entity;

      this->humanCount_ =
          SdfValueOr<int>(_sdf, "human_count", 4);
      this->scenarioTopic_ =
          SdfValueOr<std::string>(
              _sdf,
              "scenario_topic",
              "/multi_human/scenario");
      this->commandTopic_ =
          SdfValueOr<std::string>(
              _sdf,
              "command_topic",
              "/multi_human/command");
      this->visualZ_ =
          SdfValueOr<double>(_sdf, "visual_z", 1.0);
      this->proxyZ_ =
          SdfValueOr<double>(_sdf, "proxy_z", 0.85);
      this->hiddenZ_ =
          SdfValueOr<double>(_sdf, "hidden_z", -50.0);
      this->animationName_ =
          SdfValueOr<std::string>(
              _sdf,
              "animation_name",
              "walk");
      this->useActorVisuals_ =
          SdfValueOr<bool>(
              _sdf,
              "use_actor_visuals",
              false);
      this->autoStartOnScenario_ =
          SdfValueOr<bool>(
              _sdf,
              "auto_start_on_scenario",
              false);

      if (this->humanCount_ <= 0)
      {
        gzerr << "[MultiHumanScenarioSystem] "
              << "human_count must be > 0.\n";
        return;
      }

      this->humans_.resize(
          static_cast<std::size_t>(this->humanCount_));

      for (int i = 1; i <= this->humanCount_; ++i)
      {
        HumanState h;
        h.id = i;
        h.mode = HumanMode::Hidden;
        h.active = false;
        h.ResetRuntime();
        this->humans_[static_cast<std::size_t>(i - 1)] =
            std::move(h);
      }

      const bool scenarioOk =
          this->node_.Subscribe(
              this->scenarioTopic_,
              &MultiHumanScenarioSystem::OnScenarioMessage,
              this);

      const bool commandOk =
          this->node_.Subscribe(
              this->commandTopic_,
              &MultiHumanScenarioSystem::OnCommandMessage,
              this);

      if (!scenarioOk)
      {
        gzerr << "[MultiHumanScenarioSystem] Failed to subscribe: "
              << this->scenarioTopic_ << "\n";
      }

      if (!commandOk)
      {
        gzerr << "[MultiHumanScenarioSystem] Failed to subscribe: "
              << this->commandTopic_ << "\n";
      }

      gzmsg << "[MultiHumanScenarioSystem] Configured.\n"
            << "  human_count    : " << this->humanCount_ << "\n"
            << "  scenario_topic : " << this->scenarioTopic_ << "\n"
            << "  command_topic  : " << this->commandTopic_ << "\n"
            << "  visual_z       : " << this->visualZ_ << "\n"
            << "  proxy_z        : " << this->proxyZ_ << "\n"
            << "  actor_visuals  : "
            << (this->useActorVisuals_ ? "true" : "false") << "\n"
            << "  auto_start     : "
            << (this->autoStartOnScenario_ ? "true" : "false")
            << "\n";
    }

  public:
    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override
    {
      // IMPORTANT:
      // Entity discovery and transport commands must work even while the
      // simulator is paused. Otherwise actors left at hidden_z can never be
      // reset / shown until simulation starts running.
      this->ResolveEntities(_ecm);
      this->ProcessPendingMessages(_ecm);

      if (!this->scenarioLoaded_)
      {
        // Still keep hidden pairs hidden if entities were just resolved.
        if (this->entitiesJustResolved_)
        {
          this->ApplyAllPoses(_ecm);
          this->entitiesJustResolved_ = false;
        }
        return;
      }

      if (this->resetRequested_)
      {
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);
        this->resetRequested_ = false;
      }

      if (!this->running_)
      {
        // V4.1: stopped human proxies are still authoritative kinematic
        // ground-truth. Clamp them every simulation update so physics /
        // collision impulses cannot make them drift, tilt or fly.
        this->ApplyAllPoses(_ecm);
        this->poseRefreshRequested_ = false;
        return;
      }

      // Scenario is allowed to load/reset while paused, but motion integration
      // only advances when simulation time is advancing.
      if (_info.paused)
      {
        this->ApplyAllPoses(_ecm);
        return;
      }

      const double dt =
          std::chrono::duration<double>(_info.dt).count();

      if (dt <= 0.0)
        return;

      for (auto &human : this->humans_)
      {
        if (!human.active)
          continue;

        this->UpdateHuman(human, dt);

        if (this->IsMoving(human))
          human.animationTime += _info.dt;
      }

      this->ApplyAllPoses(_ecm);
    }

  public:
    void Reset(
        const gz::sim::UpdateInfo &,
        gz::sim::EntityComponentManager &_ecm) override
    {
      this->running_ = false;

      if (this->scenarioLoaded_)
      {
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);
      }

      gzmsg << "[MultiHumanScenarioSystem] Gazebo reset handled.\n";
    }

  private:
    void OnScenarioMessage(
        const gz::msgs::StringMsg &_msg)
    {
      gzmsg << "[MultiHumanScenarioSystem] RX scenario message ("
            << _msg.data().size() << " bytes)\n";
      std::lock_guard<std::mutex> lock(this->messageMutex_);
      this->pendingScenarioJson_ = _msg.data();
      this->hasPendingScenario_ = true;
    }

  private:
    void OnCommandMessage(
        const gz::msgs::StringMsg &_msg)
    {
      gzmsg << "[MultiHumanScenarioSystem] RX command: "
            << _msg.data() << "\n";
      std::lock_guard<std::mutex> lock(this->messageMutex_);
      this->pendingCommand_ = _msg.data();
      this->hasPendingCommand_ = true;
    }

  private:
    void ProcessPendingMessages(
        gz::sim::EntityComponentManager &_ecm)
    {
      std::optional<std::string> scenarioJson;
      std::optional<std::string> command;

      {
        std::lock_guard<std::mutex> lock(this->messageMutex_);

        if (this->hasPendingScenario_)
        {
          scenarioJson = this->pendingScenarioJson_;
          this->hasPendingScenario_ = false;
        }

        if (this->hasPendingCommand_)
        {
          command = this->pendingCommand_;
          this->hasPendingCommand_ = false;
        }
      }

      if (scenarioJson.has_value())
      {
        try
        {
          this->LoadScenarioJson(*scenarioJson);
          this->BindScenarioToResolvedEntities(_ecm);
          this->ResetScenarioRuntime();

          this->scenarioLoaded_ = true;
          this->running_ = this->autoStartOnScenario_;
          this->poseRefreshRequested_ = true;

          gzmsg << "[MultiHumanScenarioSystem] Loaded episode "
                << this->episodeId_
                << " with "
                << this->ActiveHumanCount()
                << " active humans. "
                << (this->running_ ? "RUNNING" : "READY")
                << "\n";
        }
        catch (const std::exception &e)
        {
          gzerr << "[MultiHumanScenarioSystem] Scenario rejected: "
                << e.what() << "\n";
        }
      }

      if (command.has_value())
        this->ApplyCommand(*command, _ecm);
    }

  private:
    void ApplyCommand(
        std::string command,
        gz::sim::EntityComponentManager &_ecm)
    {
      // Trim simple whitespace around command.
      command.erase(
          command.begin(),
          std::find_if(
              command.begin(),
              command.end(),
              [](unsigned char c)
              {
                return !std::isspace(c);
              }));

      command.erase(
          std::find_if(
              command.rbegin(),
              command.rend(),
              [](unsigned char c)
              {
                return !std::isspace(c);
              }).base(),
          command.end());

      if (command == "reset")
      {
        if (!this->scenarioLoaded_)
        {
          gzwarn << "[MultiHumanScenarioSystem] reset ignored: "
                 << "no scenario loaded.\n";
          return;
        }

        this->running_ = false;
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);

        gzmsg << "[MultiHumanScenarioSystem] RESET episode "
              << this->episodeId_ << "\n";
        return;
      }

      if (command == "start")
      {
        if (!this->scenarioLoaded_)
        {
          gzwarn << "[MultiHumanScenarioSystem] start ignored: "
                 << "no scenario loaded.\n";
          return;
        }

        this->running_ = true;
        gzmsg << "[MultiHumanScenarioSystem] START episode "
              << this->episodeId_ << "\n";
        return;
      }

      if (command == "stop")
      {
        this->running_ = false;
        this->poseRefreshRequested_ = true;

        gzmsg << "[MultiHumanScenarioSystem] STOP episode "
              << this->episodeId_ << "\n";
        return;
      }

      if (command == "reset_start")
      {
        if (!this->scenarioLoaded_)
        {
          gzwarn << "[MultiHumanScenarioSystem] reset_start ignored: "
                 << "no scenario loaded.\n";
          return;
        }

        this->running_ = false;
        this->ResetScenarioRuntime();
        this->ApplyAllPoses(_ecm);
        this->running_ = true;

        gzmsg << "[MultiHumanScenarioSystem] RESET+START episode "
              << this->episodeId_ << "\n";
        return;
      }

      if (command == "hide_all")
      {
        this->running_ = false;

        for (auto &human : this->humans_)
        {
          human.mode = HumanMode::Hidden;
          human.active = false;
        }

        this->ApplyAllPoses(_ecm);

        gzmsg << "[MultiHumanScenarioSystem] HIDE_ALL\n";
        return;
      }

      gzwarn << "[MultiHumanScenarioSystem] Unknown command: '"
             << command
             << "'. Use reset/start/stop/reset_start/hide_all.\n";
    }

  private:
    void LoadScenarioJson(
        const std::string &_payload)
    {
      const json root = json::parse(_payload);

      this->episodeId_ = root.value("episode_id", -1);

      if (!root.contains("humans") ||
          !root.at("humans").is_array())
      {
        throw std::runtime_error(
            "Scenario requires array field 'humans'.");
      }

      // Preserve entity bindings before replacing scenario states.
      std::vector<HumanState> next(
          static_cast<std::size_t>(this->humanCount_));

      for (int i = 1; i <= this->humanCount_; ++i)
      {
        HumanState hidden;
        hidden.id = i;
        hidden.mode = HumanMode::Hidden;
        hidden.active = false;
        hidden.ResetRuntime();

        const auto &old =
            this->humans_[static_cast<std::size_t>(i - 1)];

        hidden.visualEntity = old.visualEntity;
        hidden.proxyEntity = old.proxyEntity;
        hidden.entitiesResolved = old.entitiesResolved;
        hidden.actorAvailable = old.actorAvailable;
        hidden.actorOrigin = old.actorOrigin;

        next[static_cast<std::size_t>(i - 1)] =
            std::move(hidden);
      }

      std::vector<bool> seen(
          static_cast<std::size_t>(this->humanCount_),
          false);

      for (const auto &item : root.at("humans"))
      {
        HumanState h = ParseHuman(item, this->humanCount_);
        const std::size_t idx =
            static_cast<std::size_t>(h.id - 1);

        if (seen[idx])
        {
          throw std::runtime_error(
              "Duplicate human id " + std::to_string(h.id) + ".");
        }

        seen[idx] = true;

        const auto &old = this->humans_[idx];
        h.visualEntity = old.visualEntity;
        h.proxyEntity = old.proxyEntity;
        h.entitiesResolved = old.entitiesResolved;
        h.actorAvailable = old.actorAvailable;
        h.actorOrigin = old.actorOrigin;

        next[idx] = std::move(h);
      }

      this->humans_ = std::move(next);
    }

  private:
    void ResolveEntities(
        gz::sim::EntityComponentManager &_ecm)
    {
      bool anyNew = false;

      for (auto &human : this->humans_)
      {
        if (human.entitiesResolved)
          continue;

        const std::string suffix = TwoDigit(human.id);
        const std::string visualName =
            "human_" + suffix + "_visual";
        const std::string proxyName =
            "human_" + suffix + "_proxy";

        // ------------------------------------------------------------
        // V4 policy:
        //   - proxy Model is REQUIRED
        //   - Actor visual is OPTIONAL cosmetic animation
        //
        // The proxy itself owns visible primitive geometry in
        // arena_dataset_v4.sdf, so the simulation remains understandable
        // even when no Actor exists.
        // ------------------------------------------------------------
        gz::sim::Entity proxyEntity = gz::sim::kNullEntity;

        _ecm.Each<
            gz::sim::components::Name,
            gz::sim::components::Model>(
            [&](const gz::sim::Entity &_entity,
                const gz::sim::components::Name *_name,
                const gz::sim::components::Model *) -> bool
            {
              if (_name && _name->Data() == proxyName)
              {
                proxyEntity = _entity;
                return false;
              }
              return true;
            });

        if (proxyEntity == gz::sim::kNullEntity)
          continue;

        human.proxyEntity = proxyEntity;
        human.entitiesResolved = true;
        human.actorAvailable = false;
        human.visualEntity = gz::sim::kNullEntity;

        // Optional Actor binding. Disabled by default in V4.
        if (this->useActorVisuals_)
        {
          gz::sim::Entity actorEntity = gz::sim::kNullEntity;

          _ecm.Each<
              gz::sim::components::Name,
              gz::sim::components::Actor>(
              [&](const gz::sim::Entity &_entity,
                  const gz::sim::components::Name *_name,
                  const gz::sim::components::Actor *) -> bool
              {
                if (_name && _name->Data() == visualName)
                {
                  actorEntity = _entity;
                  return false;
                }
                return true;
              });

          if (actorEntity != gz::sim::kNullEntity)
          {
            gz::sim::Actor actor(actorEntity);
            const auto origin = actor.Pose(_ecm);

            if (origin.has_value())
            {
              human.visualEntity = actorEntity;
              human.actorOrigin = *origin;
              human.actorAvailable = true;

              actor.SetAnimationName(
                  _ecm,
                  this->animationName_);
            }
          }
        }

        anyNew = true;

        gzmsg << "[MultiHumanScenarioSystem] Bound H"
              << human.id
              << ": proxy=" << proxyName
              << ", actor="
              << (human.actorAvailable ? visualName : "optional/disabled")
              << "\n";
      }

      if (anyNew)
        this->entitiesJustResolved_ = true;
    }

  private:
    void BindScenarioToResolvedEntities(
        gz::sim::EntityComponentManager &_ecm)
    {
      // LoadScenarioJson preserves current bindings. If plugin received
      // a scenario before all entities appeared, retry here.
      this->ResolveEntities(_ecm);
    }

  private:
    void ResetScenarioRuntime()
    {
      for (auto &human : this->humans_)
        human.ResetRuntime();
    }

  private:
    void UpdateHuman(
        HumanState &_human,
        double _dt)
    {
      _human.totalElapsed += _dt;

      switch (_human.mode)
      {
        case HumanMode::Hidden:
        case HumanMode::Stationary:
          return;

        case HumanMode::Autonomous:
          this->UpdateAutonomous(_human, _dt);
          return;

        case HumanMode::Scripted:
          this->UpdateScripted(_human, _dt);
          return;
      }
    }

  private:
    void UpdateAutonomous(
        HumanState &_human,
        double _dt)
    {
      if (_human.waypoints.empty() ||
          _human.speed <= kEps)
      {
        return;
      }

      // Handle one or more already-reached waypoints without looping forever.
      for (std::size_t guard = 0;
           guard < _human.waypoints.size();
           ++guard)
      {
        const auto &target =
            _human.waypoints[_human.waypointIndex];

        const double d =
            Distance(
                _human.x,
                _human.y,
                target.x,
                target.y);

        if (d > _human.waypointTolerance)
          break;

        const std::size_t next =
            _human.waypointIndex + 1;

        if (next < _human.waypoints.size())
        {
          _human.waypointIndex = next;
        }
        else if (_human.loop)
        {
          _human.waypointIndex = 0;
        }
        else
        {
          _human.speed = 0.0;
          return;
        }
      }

      const auto &target =
          _human.waypoints[_human.waypointIndex];

      const double d =
          Distance(
              _human.x,
              _human.y,
              target.x,
              target.y);

      if (d <= kEps)
        return;

      _human.yaw =
          HeadingTo(
              _human.x,
              _human.y,
              target.x,
              target.y);

      const double step =
          std::min(_human.speed * _dt, d);

      _human.x +=
          step * std::cos(_human.yaw);
      _human.y +=
          step * std::sin(_human.yaw);
    }

  private:
    void UpdateScripted(
        HumanState &_human,
        double _dt)
    {
      if (!_human.segments.empty())
      {
        this->UpdateScriptedSegments(_human, _dt);
        return;
      }

      if (_human.duration.has_value() &&
          _human.totalElapsed > *_human.duration)
      {
        return;
      }

      if (_human.speed <= kEps)
        return;

      _human.x +=
          _human.speed * _dt * std::cos(_human.yaw);
      _human.y +=
          _human.speed * _dt * std::sin(_human.yaw);
    }

  private:
    void UpdateScriptedSegments(
        HumanState &_human,
        double _dt)
    {
      double remainingDt = _dt;

      while (remainingDt > kEps &&
             _human.segmentIndex < _human.segments.size())
      {
        auto &seg =
            _human.segments[_human.segmentIndex];

        const double segmentRemaining =
            std::max(
                0.0,
                seg.duration - _human.segmentElapsed);

        const double useDt =
            std::min(
                remainingDt,
                segmentRemaining);

        _human.yaw =
            NormalizeAngle(seg.heading);

        _human.x +=
            seg.speed * useDt * std::cos(_human.yaw);
        _human.y +=
            seg.speed * useDt * std::sin(_human.yaw);

        _human.segmentElapsed += useDt;
        remainingDt -= useDt;

        if (_human.segmentElapsed + kEps >= seg.duration)
        {
          ++_human.segmentIndex;
          _human.segmentElapsed = 0.0;
        }
      }
    }

  private:
    bool IsMoving(
        const HumanState &_human) const
    {
      if (!_human.active)
        return false;

      if (_human.mode == HumanMode::Autonomous)
        return _human.speed > kEps;

      if (_human.mode == HumanMode::Scripted)
      {
        if (!_human.segments.empty())
        {
          if (_human.segmentIndex >= _human.segments.size())
            return false;

          return _human.segments[_human.segmentIndex].speed > kEps;
        }

        if (_human.duration.has_value() &&
            _human.totalElapsed > *_human.duration)
        {
          return false;
        }

        return _human.speed > kEps;
      }

      return false;
    }

  private:
    void ApplyAllPoses(
        gz::sim::EntityComponentManager &_ecm)
    {
      for (auto &human : this->humans_)
        this->ApplyHumanPose(human, _ecm);
    }

  private:
    void ApplyHumanPose(
        HumanState &_human,
        gz::sim::EntityComponentManager &_ecm)
    {
      if (!_human.entitiesResolved)
        return;

      // Proxy is the authoritative physical + visible representation.
      gz::sim::Model proxy(_human.proxyEntity);

      if (!proxy.Valid(_ecm))
      {
        _human.entitiesResolved = false;
        _human.actorAvailable = false;
        return;
      }

      const bool visible =
          _human.active &&
          _human.mode != HumanMode::Hidden;

      const double proxyZ =
          visible ? this->proxyZ_ : this->hiddenZ_;

      const gz::math::Pose3d proxyWorld(
          _human.x,
          _human.y,
          proxyZ,
          0.0,
          0.0,
          _human.yaw);

      proxy.SetWorldPoseCmd(
          _ecm,
          proxyWorld);

      // Optional skeletal Actor: cosmetic only.
      if (!this->useActorVisuals_ ||
          !_human.actorAvailable ||
          _human.visualEntity == gz::sim::kNullEntity)
      {
        return;
      }

      gz::sim::Actor actor(_human.visualEntity);

      if (!actor.Valid(_ecm))
      {
        _human.actorAvailable = false;
        return;
      }

      const double visualZ =
          visible ? this->visualZ_ : this->hiddenZ_;

      const gz::math::Pose3d actorWorld(
          _human.x,
          _human.y,
          visualZ,
          0.0,
          0.0,
          _human.yaw);

      const gz::math::Pose3d trajectoryPose =
          _human.actorOrigin.Inverse() * actorWorld;

      actor.SetTrajectoryPose(
          _ecm,
          trajectoryPose);

      actor.SetAnimationName(
          _ecm,
          this->animationName_);

      actor.SetAnimationTime(
          _ecm,
          _human.animationTime);
    }

  private:
    std::size_t ActiveHumanCount() const
    {
      return static_cast<std::size_t>(
          std::count_if(
              this->humans_.begin(),
              this->humans_.end(),
              [](const HumanState &h)
              {
                return h.active &&
                       h.mode != HumanMode::Hidden;
              }));
    }

  private:
    gz::sim::Entity worldEntity_{gz::sim::kNullEntity};

  private:
    int humanCount_{4};

  private:
    std::string scenarioTopic_{"/multi_human/scenario"};

  private:
    std::string commandTopic_{"/multi_human/command"};

  private:
    double visualZ_{1.0};

  private:
    double proxyZ_{0.85};

  private:
    double hiddenZ_{-50.0};

  private:
    std::string animationName_{"walk"};

  private:
    bool useActorVisuals_{false};

  private:
    bool autoStartOnScenario_{false};

  private:
    std::vector<HumanState> humans_;

  private:
    int episodeId_{-1};

  private:
    bool scenarioLoaded_{false};

  private:
    bool running_{false};

  private:
    bool resetRequested_{false};

  private:
    bool poseRefreshRequested_{false};

  private:
    bool entitiesJustResolved_{false};

  private:
    gz::transport::Node node_;

  private:
    std::mutex messageMutex_;

  private:
    std::string pendingScenarioJson_;

  private:
    bool hasPendingScenario_{false};

  private:
    std::string pendingCommand_;

  private:
    bool hasPendingCommand_{false};
};

}  // namespace custom_corridor


GZ_ADD_PLUGIN(
    custom_corridor::MultiHumanScenarioSystem,
    gz::sim::System,
    custom_corridor::MultiHumanScenarioSystem::ISystemConfigure,
    custom_corridor::MultiHumanScenarioSystem::ISystemPreUpdate,
    custom_corridor::MultiHumanScenarioSystem::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
    custom_corridor::MultiHumanScenarioSystem,
    "custom_corridor::MultiHumanScenarioSystem")
