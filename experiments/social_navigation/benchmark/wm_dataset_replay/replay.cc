// Visual replay of logged XY/yaw. No physics, navigation or human policy runs.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <stdexcept>
using json=nlohmann::json;
namespace wm_recorded {
class Replay: public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 struct Pose {double x,y,yaw;};
 struct Row {double t;std::map<std::string,Pose> poses;};
 std::vector<Row> rows; std::map<std::string,gz::sim::Entity> entities;
 gz::transport::Node node; gz::transport::Node::Publisher status;
 double begin=0,end=0,duration=0,lastPublish=-1;
 public:
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>& sdf,gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
  std::ifstream in(sdf->Get<std::string>("poses"));std::string line;
  while(std::getline(in,line)) {auto j=json::parse(line);if(j.at("phase")!="active")continue;Row r;r.t=j.at("time");
   for(auto it=j.at("poses").begin();it!=j.at("poses").end();++it) r.poses[it.key()]={it.value().at("x"),it.value().at("y"),it.value().at("yaw")};
   if(!rows.empty()&&r.t<=rows.back().t)throw std::runtime_error("Nonmonotonic recorded times");rows.push_back(r);
  }
  if(rows.size()<2)throw std::runtime_error("No usable recorded poses");
  begin=rows.front().t;end=rows.back().t;duration=end-begin;
  status=node.Advertise<gz::msgs::StringMsg>("/wm_recorded_replay/status");
 }
 void PreUpdate(const gz::sim::UpdateInfo& info,gz::sim::EntityComponentManager& ecm) override {
  double st=std::chrono::duration<double>(info.simTime).count();double t=begin+std::fmod(std::max(0.,st),duration+5.);t=std::min(t,end);
  auto b=std::upper_bound(rows.begin(),rows.end(),t,[](double v,const Row&r){return v<r.t;});
  size_t i=b==rows.begin()?0:static_cast<size_t>(b-rows.begin()-1);size_t k=std::min(i+1,rows.size()-1);
  double a=k==i?0:std::clamp((t-rows[i].t)/(rows[k].t-rows[i].t),0.,1.);
  json applied=json::object();
  for(const auto& [name,p]:rows[i].poses){
   const auto &q=rows[k].poses.at(name);double x=p.x+a*(q.x-p.x),y=p.y+a*(q.y-p.y),yaw=p.yaw+a*std::remainder(q.yaw-p.yaw,2*M_PI);
   if(!entities.count(name))entities[name]=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name(name));
   auto entity=entities[name];if(entity==gz::sim::kNullEntity)throw std::runtime_error("Missing replay model: "+name);
   ecm.SetComponentData<gz::sim::components::Pose>(entity,gz::math::Pose3d(x,y,name=="mecanum_robot"?.04:.85,0,0,yaw));
   ecm.SetChanged(entity,gz::sim::components::Pose::typeId,gz::sim::ComponentState::OneTimeChange);
   applied[name]={{"x",x},{"y",y},{"yaw",yaw}};
   if(name.find("_proxy")!=std::string::npos){auto visual=name.substr(0,name.size()-6)+"_visual";
    if(!entities.count(visual))entities[visual]=ecm.EntityByComponents(gz::sim::components::Name(visual));
    auto actor=entities[visual];if(actor==gz::sim::kNullEntity)throw std::runtime_error("Missing actor");
    gz::sim::Actor ac(actor);ac.SetTrajectoryPose(ecm,gz::math::Pose3d(x,y,1,0,0,yaw));ac.SetAnimationName(ecm,"walk");
    // No skeletal animation was recorded. Freeze the standing avatar pose rather than claim exact limb replay.
    ac.SetAnimationTime(ecm,std::chrono::steady_clock::duration::zero());
    ecm.SetChanged(actor,gz::sim::components::TrajectoryPose::typeId,gz::sim::ComponentState::OneTimeChange);
    ecm.SetChanged(actor,gz::sim::components::AnimationTime::typeId,gz::sim::ComponentState::OneTimeChange);
   }
  }
  if(lastPublish<0||st-lastPublish>=.5){lastPublish=st;gz::msgs::StringMsg msg;
   msg.set_data(json({{"recorded_time",t},{"source_row",i},{"interpolation",a},{"cycle_duration",duration+5},{"paused",info.paused},{"applied",applied},{"mode","recorded_xy_yaw_visualization"}}).dump());status.Publish(msg);}
 }
};
}
GZ_ADD_PLUGIN(wm_recorded::Replay,gz::sim::System,wm_recorded::Replay::ISystemConfigure,wm_recorded::Replay::ISystemPreUpdate)
