#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/Actor.hh>
#include <gz/transport/Node.hh>

namespace custom_corridor
{
enum class ObstacleMode
{
  HUMAN,
  OBJECT,
  NONE
};

class OscillatingObstacle final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &) override
  {
    if (!_ecm.EntityHasComponentType(_entity, gz::sim::components::Model::typeId))
    {
      gzerr << "OscillatingObstacle must be attached directly to a <model>.\n";
      return;
    }

    this->modelEntity = _entity;

    this->posePub =
      this->node.Advertise<gz::msgs::Pose>("/dynamic_obstacle/pose");
    this->statusPub =
      this->node.Advertise<gz::msgs::StringMsg>("/obstacle_type/status");

    this->node.Subscribe(
      "/obstacle_type", &OscillatingObstacle::OnModeCommand, this);

    if (_sdf->HasElement("min_x"))
      this->minX = _sdf->Get<double>("min_x");
    if (_sdf->HasElement("max_x"))
      this->maxX = _sdf->Get<double>("max_x");
    if (_sdf->HasElement("speed"))
      this->speed = _sdf->Get<double>("speed");
    if (_sdf->HasElement("y"))
      this->y = _sdf->Get<double>("y");
    if (_sdf->HasElement("z"))
      this->z = _sdf->Get<double>("z");

    std::string modeStr = "human";
    if (_sdf->HasElement("default_mode"))
      modeStr = _sdf->Get<std::string>("default_mode");

    const char *envMode = std::getenv("GZ_OBSTACLE_TYPE");
    if (envMode != nullptr && std::string(envMode).length() > 0)
    {
      modeStr = std::string(envMode);
    }
    this->SetModeFromString(modeStr);

    if (this->maxX <= this->minX || this->speed <= 0.0)
    {
      gzerr << "OscillatingObstacle requires max_x > min_x and speed > 0.\n";
      this->modelEntity = gz::sim::kNullEntity;
    }
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->modelEntity == gz::sim::kNullEntity)
      return;

    if (this->humanActorEntity == gz::sim::kNullEntity)
      this->humanActorEntity = _ecm.EntityByComponents(gz::sim::components::Name("human_actor"));
    if (this->dynamicObjectEntity == gz::sim::kNullEntity)
      this->dynamicObjectEntity = _ecm.EntityByComponents(gz::sim::components::Name("dynamic_object"));

    // Synchronized motion matching human_actor trajectory (18s periodic loop):
    // 0.0s -> 7.5s:  Walk forward from maxX to minX (yaw = PI)
    // 7.5s -> 9.0s:  Turn in place at minX (yaw PI -> 0)
    // 9.0s -> 16.5s: Walk back from minX to maxX (yaw = 0)
    // 16.5s -> 18.0s: Turn in place at maxX (yaw 0 -> PI)
    const double seconds = std::max(
      0.0, std::chrono::duration<double>(_info.simTime).count());
    const double span = this->maxX - this->minX;
    const double walkDuration = span / this->speed;
    const double turnDuration = 1.5;
    const double halfPeriod = walkDuration + turnDuration;
    const double totalPeriod = 2.0 * halfPeriod;
    const double t = std::fmod(seconds, totalPeriod);

    double x = this->minX;
    double yaw = 0.0;

    if (t < walkDuration)
    {
      x = this->maxX - t * this->speed;
      yaw = M_PI;
    }
    else if (t < halfPeriod)
    {
      x = this->minX;
      yaw = M_PI - ((t - walkDuration) / turnDuration) * M_PI;
    }
    else if (t < halfPeriod + walkDuration)
    {
      x = this->minX + (t - halfPeriod) * this->speed;
      yaw = 0.0;
    }
    else
    {
      x = this->maxX;
      yaw = ((t - (halfPeriod + walkDuration)) / turnDuration) * M_PI;
    }

    const double undergroundZ = -50.0;

    // 1. Manage human_actor visibility
    if (this->humanActorEntity != gz::sim::kNullEntity)
    {
      gz::sim::Actor actor(this->humanActorEntity);
      if (this->currentMode != ObstacleMode::HUMAN)
      {
        actor.SetTrajectoryPose(_ecm, gz::math::Pose3d(0, 0, undergroundZ, 0, 0, 0));
        auto actorPoseComp = _ecm.Component<gz::sim::components::Pose>(this->humanActorEntity);
        if (actorPoseComp != nullptr)
        {
          _ecm.SetComponentData<gz::sim::components::Pose>(
            this->humanActorEntity,
            gz::math::Pose3d(0, 0, undergroundZ, 0, 0, 0));
        }
      }
      else
      {
        if (_ecm.EntityHasComponentType(this->humanActorEntity,
                                        gz::sim::components::TrajectoryPose::typeId))
        {
          _ecm.RemoveComponent<gz::sim::components::TrajectoryPose>(this->humanActorEntity);
        }
        auto actorPoseComp = _ecm.Component<gz::sim::components::Pose>(this->humanActorEntity);
        if (actorPoseComp != nullptr)
        {
          _ecm.SetComponentData<gz::sim::components::Pose>(
            this->humanActorEntity,
            gz::math::Pose3d(0, 0, 0, 0, 0, 0));
        }
      }
    }

    // 2. Manage dynamic_obstacle (transparent collision)
    const double collisionZ = (this->currentMode == ObstacleMode::HUMAN) ? this->z : undergroundZ;
    const gz::math::Pose3d collisionPose{x, this->y, collisionZ, 0.0, 0.0, yaw};
    if (_ecm.Component<gz::sim::components::WorldPoseCmd>(this->modelEntity) == nullptr)
    {
      _ecm.CreateComponent(this->modelEntity,
        gz::sim::components::WorldPoseCmd(collisionPose));
    }
    else
    {
      _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
        this->modelEntity, collisionPose);
    }

    // 3. Manage dynamic_object (visible industrial object)
    if (this->dynamicObjectEntity != gz::sim::kNullEntity)
    {
      const double objectZ = (this->currentMode == ObstacleMode::OBJECT) ? this->z : undergroundZ;
      const gz::math::Pose3d objectPose{x, this->y, objectZ, 0.0, 0.0, yaw};
      if (_ecm.Component<gz::sim::components::WorldPoseCmd>(this->dynamicObjectEntity) == nullptr)
      {
        _ecm.CreateComponent(this->dynamicObjectEntity,
          gz::sim::components::WorldPoseCmd(objectPose));
      }
      else
      {
        _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
          this->dynamicObjectEntity, objectPose);
      }
    }

    // 4. Publish ground-truth pose
    const double activeZ = (this->currentMode != ObstacleMode::NONE) ? this->z : undergroundZ;
    gz::msgs::Pose msg;
    const auto sim_sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto sim_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - sim_sec);
    msg.mutable_header()->mutable_stamp()->set_sec(sim_sec.count());
    msg.mutable_header()->mutable_stamp()->set_nsec(sim_nsec.count());
    msg.mutable_position()->set_x(x);
    msg.mutable_position()->set_y(this->y);
    msg.mutable_position()->set_z(activeZ);

    const gz::math::Quaterniond q(0.0, 0.0, yaw);
    msg.mutable_orientation()->set_x(q.X());
    msg.mutable_orientation()->set_y(q.Y());
    msg.mutable_orientation()->set_z(q.Z());
    msg.mutable_orientation()->set_w(q.W());

    this->posePub.Publish(msg);

    // Periodically publish status (1 Hz)
    if (_info.simTime - this->lastStatusTime >= std::chrono::seconds(1))
    {
      this->lastStatusTime = _info.simTime;
      gz::msgs::StringMsg statusMsg;
      statusMsg.set_data(this->GetModeString());
      this->statusPub.Publish(statusMsg);
    }
  }

  private: void SetModeFromString(const std::string &_mode)
  {
    std::string lower = _mode;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower == "object" || lower == "box" || lower == "cylinder")
    {
      this->currentMode = ObstacleMode::OBJECT;
      gzmsg << "[OscillatingObstacle] Mode set to: OBJECT\n";
    }
    else if (lower == "none" || lower == "clean" || lower == "empty")
    {
      this->currentMode = ObstacleMode::NONE;
      gzmsg << "[OscillatingObstacle] Mode set to: NONE\n";
    }
    else
    {
      this->currentMode = ObstacleMode::HUMAN;
      gzmsg << "[OscillatingObstacle] Mode set to: HUMAN\n";
    }

    gz::msgs::StringMsg statusMsg;
    statusMsg.set_data(this->GetModeString());
    this->statusPub.Publish(statusMsg);
  }

  private: std::string GetModeString() const
  {
    switch (this->currentMode)
    {
      case ObstacleMode::OBJECT: return "object";
      case ObstacleMode::NONE: return "none";
      case ObstacleMode::HUMAN:
      default: return "human";
    }
  }

  private: void OnModeCommand(const gz::msgs::StringMsg &_msg)
  {
    this->SetModeFromString(_msg.data());
  }

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity humanActorEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity dynamicObjectEntity{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher posePub;
  private: gz::transport::Node::Publisher statusPub;
  private: ObstacleMode currentMode{ObstacleMode::HUMAN};
  private: double minX{-1.0};
  private: double maxX{5.0};
  private: double speed{0.8};
  private: double y{0.0};
  private: double z{0.85};
  private: std::chrono::steady_clock::duration lastStatusTime{std::chrono::seconds(-10)};
};
}  // namespace custom_corridor

GZ_ADD_PLUGIN(
  custom_corridor::OscillatingObstacle,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  custom_corridor::OscillatingObstacle,
  "custom_corridor::OscillatingObstacle")