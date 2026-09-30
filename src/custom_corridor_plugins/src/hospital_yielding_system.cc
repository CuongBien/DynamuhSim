#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

namespace custom_corridor {
class HospitalYieldingSystem : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    if (sdf->HasElement("robot_name")) robotName = sdf->Get<std::string>("robot_name");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    if (actorEntity == gz::sim::kNullEntity) {
      // Actor data has no comparison operator in sdformat14. Query component
      // presence and compare only the entity name, as in the School plugin.
      ecm.Each<gz::sim::components::Name, gz::sim::components::Actor>(
        [&](const gz::sim::Entity &entity,
            const gz::sim::components::Name *name,
            const gz::sim::components::Actor *) -> bool {
          if (name->Data() == "medium_walker_07_yielding") {
            actorEntity = entity;
            return false;
          }
          return true;
        });
    }
    if (actorEntity == gz::sim::kNullEntity) return;
    if (robotEntity == gz::sim::kNullEntity)
      robotEntity = ecm.EntityByComponents(gz::sim::components::Model(),
        gz::sim::components::Name(robotName));
    double distance = 1e9;
    if (robotEntity != gz::sim::kNullEntity) {
      const auto p = gz::sim::worldPose(robotEntity, ecm).Pos();
      // Distance to the whole crossing, so retreat starts before the robot
      // enters it; absence of a robot leaves the normal patrol active.
      distance = std::hypot(p.X(), std::max(0.0, std::abs(p.Y()) - 1.35));
    }
    if (!yielding && distance < 3.5) {
      yielding = true; side = y >= 0 ? 1.35 : -1.35; clearTime = 0;
    }
    if (yielding) {
      clearTime = distance > 4.5 ? clearTime + dt : 0;
      if (clearTime >= 1.0) { yielding = false; pause = 0.5; }
    }
    const double goal = yielding ? side : target;
    const double previous = y;
    if (yielding || pause <= 0) {
      const double delta = goal - y;
      const double step = (yielding ? 0.75 : 0.55) * dt;
      y += std::clamp(delta, -step, step);
      if (std::abs(delta) > 1e-6) yaw = delta > 0 ? 1.57079632679 : -1.57079632679;
      if (!yielding && std::abs(y - target) < 1e-6) {
        target = -target; pause = 2.0;
      }
    } else pause = std::max(0.0, pause - dt);
    // Freeze gait when waiting; animate only for actual travelled distance.
    animationTime += std::abs(y - previous) / 0.55;
    gz::sim::Actor actor(actorEntity);
    actor.SetTrajectoryPose(ecm, gz::math::Pose3d(0, y, 1, 0, 0, yaw));
    actor.SetAnimationName(ecm, "walk");
    actor.SetAnimationTime(ecm, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(animationTime)));
    // Gazebo 8 rendering must be notified after manual actor updates.
    ecm.SetChanged(actorEntity, gz::sim::components::TrajectoryPose::typeId,
      gz::sim::ComponentState::OneTimeChange);
    ecm.SetChanged(actorEntity, gz::sim::components::AnimationTime::typeId,
      gz::sim::ComponentState::OneTimeChange);
  }
 private:
  gz::sim::Entity actorEntity{gz::sim::kNullEntity}, robotEntity{gz::sim::kNullEntity};
  std::string robotName{"burger"};
  double y{-1.35}, target{1.35}, side{-1.35}, yaw{1.57079632679};
  double animationTime{0}, clearTime{0}, pause{2};
  bool yielding{false};
};
}
GZ_ADD_PLUGIN(custom_corridor::HospitalYieldingSystem, gz::sim::System,
  custom_corridor::HospitalYieldingSystem::ISystemConfigure,
  custom_corridor::HospitalYieldingSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(custom_corridor::HospitalYieldingSystem, "custom_corridor::HospitalYieldingSystem")
