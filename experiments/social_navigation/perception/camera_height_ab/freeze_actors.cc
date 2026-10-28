// Camera experiment only: hold the existing actor pose and animation time fixed.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <chrono>
#include <vector>
#include <utility>
namespace height_experiment {
class FreezeActors : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate {
  std::vector<std::pair<std::string,gz::math::Pose3d>> targets;
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    auto config=sdf->Clone();
    auto item=config->GetElement("frozen_actor");
    while(item) {
      targets.emplace_back(item->Get<std::string>("name"), item->Get<gz::math::Pose3d>("pose"));
      item=item->GetNextElement("frozen_actor");
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &,gz::sim::EntityComponentManager &ecm) override {
    for(const auto &target:targets) {
      auto entity=ecm.EntityByComponents(gz::sim::components::Name(target.first));
      if(entity==gz::sim::kNullEntity) continue;
      gz::sim::Actor actor(entity);
      actor.SetTrajectoryPose(ecm,target.second);
      actor.SetAnimationName(ecm,"walk");
      actor.SetAnimationTime(ecm,std::chrono::steady_clock::duration::zero());
      ecm.SetChanged(entity,gz::sim::components::TrajectoryPose::typeId,gz::sim::ComponentState::OneTimeChange);
      ecm.SetChanged(entity,gz::sim::components::AnimationTime::typeId,gz::sim::ComponentState::OneTimeChange);
    }
  }
};
}
GZ_ADD_PLUGIN(height_experiment::FreezeActors,gz::sim::System,
              gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
