#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

namespace custom_corridor
{
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
      gzerr << "OscillatingObstacle must be attached directly to a <model>.\\n";
      return;
    }

    this->modelEntity = _entity;

    this->posePub =
      this->node.Advertise<gz::msgs::Pose>("/dynamic_obstacle/pose");

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

    if (this->maxX <= this->minX || this->speed <= 0.0)
    {
      gzerr << "OscillatingObstacle requires max_x > min_x and speed > 0.\\n";
      this->modelEntity = gz::sim::kNullEntity;
    }
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->modelEntity == gz::sim::kNullEntity)
      return;

    // Triangle wave: minX -> maxX -> minX.  It derives solely from sim time,
    // so pauses and resets cannot accumulate pose or velocity error.
    const double seconds = std::max(
      0.0, std::chrono::duration<double>(_info.simTime).count());
    const double span = this->maxX - this->minX;
    const double periodDistance = 2.0 * span;
    const double travelled = std::fmod(seconds * this->speed, periodDistance);
    const double x = (travelled <= span)
      ? this->minX + travelled
      : this->maxX - (travelled - span);

    const gz::math::Pose3d pose{x, this->y, this->z, 0.0, 0.0, 0.0};
    gz::msgs::Pose msg;
    const auto sim_sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto sim_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - sim_sec);
    msg.mutable_header()->mutable_stamp()->set_sec(sim_sec.count());
    msg.mutable_header()->mutable_stamp()->set_nsec(sim_nsec.count());
    msg.mutable_position()->set_x(x);
    msg.mutable_position()->set_y(this->y);
    msg.mutable_position()->set_z(this->z);

    msg.mutable_orientation()->set_x(0.0);
    msg.mutable_orientation()->set_y(0.0);
    msg.mutable_orientation()->set_z(0.0);
    msg.mutable_orientation()->set_w(1.0);

    this->posePub.Publish(msg);
    if (_ecm.Component<gz::sim::components::WorldPoseCmd>(this->modelEntity) == nullptr)
    {
      _ecm.CreateComponent(this->modelEntity,
        gz::sim::components::WorldPoseCmd(pose));
    }
    else
    {
      _ecm.SetComponentData<gz::sim::components::WorldPoseCmd>(
        this->modelEntity, pose);
    }
  }

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher posePub;
  private: double minX{-1.0};
  private: double maxX{5.0};
  private: double speed{1.0};  // metres per simulated second
  private: double y{0.0};
  private: double z{0.35};
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