// Experiment-only scripted pose/animation; records actual ECM components as LABEL data.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <vector>
namespace perception_corpus {
struct Target {int id; gz::math::Vector3d start,end; double yaw,phase,rate;};
class Motion : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 std::vector<Target> targets; double duration=8.,last=-1.;std::ofstream trace;
 public:
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>& sdf,gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  auto config=sdf->Clone();duration=config->Get<double>("duration");trace.open(config->Get<std::string>("trace"));trace<<std::setprecision(17);auto item=config->GetElement("target");
  while(item){targets.push_back({item->Get<int>("id"),item->Get<gz::math::Vector3d>("start"),item->Get<gz::math::Vector3d>("end"),item->Get<double>("yaw"),item->Get<double>("phase"),item->Get<double>("rate")});item=item->GetNextElement("target");}
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(info.paused)return;const double t=std::chrono::duration<double>(info.simTime).count();const double f=std::max(0.,std::min(1.,t/duration));
  for(const auto &v:targets){const auto p=v.start+(v.end-v.start)*f;auto actor=ecm.EntityByComponents(gz::sim::components::Name("common_"+std::to_string(v.id)+"_visual"));auto proxy=ecm.EntityByComponents(gz::sim::components::Name("common_"+std::to_string(v.id)+"_proxy"));
   if(actor!=gz::sim::kNullEntity){gz::sim::Actor a(actor);a.SetTrajectoryPose(ecm,gz::math::Pose3d(p.X(),p.Y(),1.,0,0,v.yaw));a.SetAnimationName(ecm,"walk");a.SetAnimationTime(ecm,std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(v.phase+v.rate*t)));ecm.SetChanged(actor,gz::sim::components::TrajectoryPose::typeId,gz::sim::ComponentState::OneTimeChange);ecm.SetChanged(actor,gz::sim::components::AnimationTime::typeId,gz::sim::ComponentState::OneTimeChange);}
   if(proxy!=gz::sim::kNullEntity)gz::sim::Model(proxy).SetWorldPoseCmd(ecm,gz::math::Pose3d(p.X(),p.Y(),.85,0,0,v.yaw));
  }
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
  if(info.paused)return;const double t=std::chrono::duration<double>(info.simTime).count();if(last>=0&&t-last<.0999)return;last=t;
  trace<<"{\"stamp_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()<<",\"humans\":[";bool comma=false;
  for(const auto &v:targets){auto actor=ecm.EntityByComponents(gz::sim::components::Name("common_"+std::to_string(v.id)+"_visual"));auto proxy=ecm.EntityByComponents(gz::sim::components::Name("common_"+std::to_string(v.id)+"_proxy"));if(actor==gz::sim::kNullEntity||proxy==gz::sim::kNullEntity)continue;auto component=ecm.Component<gz::sim::components::TrajectoryPose>(actor);if(!component)continue;const auto a=component->Data();const auto p=gz::sim::worldPose(proxy,ecm);if(comma)trace<<",";comma=true;trace<<"{\"id\":"<<v.id<<",\"actor_xy\":["<<a.X()<<","<<a.Y()<<"],\"proxy_xy\":["<<p.X()<<","<<p.Y()<<"]}";}
  trace<<"]}\n";trace.flush();
 }
};
}
GZ_ADD_PLUGIN(perception_corpus::Motion,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
