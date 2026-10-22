// Isolated Common cohort pilot; original adapters and robot source are unchanged.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <chrono>
#include <map>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
using json=nlohmann::json;
namespace response_experiment {
class Adapter: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 int fd{-1},population{6},step{0}; bool failed{false},haveRobot{false};
 double planningDt{.1}; double last{-1},lastPublish{-1},prevX{0},prevY{0},prevYaw{0};
 std::string socketPath; json from=json::array(),to=json::array(),people=json::array(),groups=json::array(),robotInput;
 std::map<int,double> animation; gz::transport::Node node; gz::transport::Node::Publisher publisher,stopPublisher; gz::sim::EventManager *events{nullptr};
 public:
 ~Adapter() override {if(fd>=0)close(fd);}
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>& sdf,gz::sim::EntityComponentManager&,gz::sim::EventManager& manager) override {
  events=&manager;stopPublisher=node.Advertise<gz::msgs::Twist>("/cmd_vel");
  socketPath=sdf->Get<std::string>("socket");population=sdf->Get<int>("population");planningDt=sdf->Get<double>("planning_dt");
  if(std::abs(planningDt-.1)>1e-9&&std::abs(planningDt-.25)>1e-9)throw std::runtime_error("Invalid planning dt");
  if(population<1||population>24)throw std::runtime_error("Invalid population");
  publisher=node.Advertise<gz::msgs::StringMsg>("/common/ground_truth");
 }
 json Exchange(const json &request) {
  if(fd<0){fd=socket(AF_UNIX,SOCK_STREAM,0);if(fd<0)throw std::runtime_error("socket failed");
   timeval timeout{10,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
   sockaddr_un address{};address.sun_family=AF_UNIX;if(socketPath.size()>=sizeof(address.sun_path))throw std::runtime_error("socket path too long");
   std::strcpy(address.sun_path,socketPath.c_str());if(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)throw std::runtime_error("worker unavailable");}
  std::string bytes=request.dump()+"\n";size_t sent=0;
  while(sent<bytes.size()){auto n=send(fd,bytes.data()+sent,bytes.size()-sent,MSG_NOSIGNAL);if(n<0&&errno==EINTR)continue;if(n<=0)throw std::runtime_error("worker send failed");sent+=n;}
  std::string reply;char c;while(reply.size()<131072){auto n=recv(fd,&c,1,0);if(n<0&&errno==EINTR)continue;if(n!=1)throw std::runtime_error("worker timeout/disconnected");if(c=='\n')return json::parse(reply);reply+=c;}
  throw std::runtime_error("worker response too large");
 }
 void Apply(gz::sim::EntityComponentManager &ecm,double dt){
  for(const auto &h:people){int id=h.at("id");double x=h.at("x"),y=h.at("y"),yaw=h.at("yaw"),speed=h.at("speed");
   auto model=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("common_"+std::to_string(id)+"_proxy"));
   if(model!=gz::sim::kNullEntity)gz::sim::Model(model).SetWorldPoseCmd(ecm,gz::math::Pose3d(x,y,.85,0,0,yaw));
   auto actor=ecm.EntityByComponents(gz::sim::components::Name("common_"+std::to_string(id)+"_visual"));
   if(actor!=gz::sim::kNullEntity){gz::sim::Actor a(actor);animation[id]+=speed*dt*4.1534;
    a.SetTrajectoryPose(ecm,gz::math::Pose3d(x,y,1.,0,0,yaw));a.SetAnimationName(ecm,"walk");
    a.SetAnimationTime(ecm,std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(animation[id])));
    ecm.SetChanged(actor,gz::sim::components::TrajectoryPose::typeId,gz::sim::ComponentState::OneTimeChange);
    ecm.SetChanged(actor,gz::sim::components::AnimationTime::typeId,gz::sim::ComponentState::OneTimeChange);}}
 }
 void Interlock(){gz::msgs::Twist zero;stopPublisher.Publish(zero);events->Emit<gz::sim::events::Pause>(true);}
 void Fail(const std::string &why,double t){failed=true;Interlock();std::cerr<<"[Common cohort adapter] frozen: "<<why<<std::endl;gz::msgs::StringMsg m;m.set_data(json({{"adapter_ok",false},{"time",t},{"error",why},{"fatal",true},{"restart_required",true},{"world_pause_requested",true},{"last_applied_humans",people},{"robot_input",robotInput}}).dump());publisher.Publish(m);}
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(failed){if(!info.paused){Interlock();Apply(ecm,0);}return;}if(info.paused)return;double t=std::chrono::duration<double>(info.simTime).count();
  if(last>=0&&t<last){Fail("Reset requires a fresh Common cohort session",t);return;}
  auto robot=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("mecanum_robot"));if(robot==gz::sim::kNullEntity)return;
  auto pose=gz::sim::worldPose(robot,ecm);double x=pose.Pos().X(),y=pose.Pos().Y(),yaw=pose.Rot().Yaw();
  try{
   if(last<0||t-last>=planningDt-1e-7){double dt=last<0?planningDt:t-last;
    if(std::abs(dt-planningDt)>1e-6)throw std::runtime_error("Noncontiguous planning clock");
    double vx=haveRobot?(x-prevX)/dt:0,vy=haveRobot?(y-prevY)/dt:0,omega=haveRobot?std::remainder(yaw-prevYaw,2*M_PI)/dt:0;
    robotInput={{"x",x},{"y",y},{"yaw",yaw},{"vx",vx},{"vy",vy},{"omega",omega}};
    auto reply=Exchange({{"time",t},{"dt",planningDt},{"frame","world"},{"epoch",0},{"robot",robotInput}});if(reply.contains("error"))throw std::runtime_error(reply["error"].get<std::string>());
    if(std::abs(reply.at("time").get<double>()-t)>1e-7||std::abs(reply.at("state_time").get<double>()-(t+planningDt))>1e-7||reply.at("humans").size()!=static_cast<size_t>(population))throw std::runtime_error("Invalid reply clock/population");
    from=reply["before"];to=reply["humans"];groups=reply["groups_before"];step=reply["step"].get<int>()-1;
    if(from.size()!=to.size())throw std::runtime_error("Invalid before population");
    for(int i=0;i<population;i++){if(from[i]["id"].get<int>()!=i+1||to[i]["id"].get<int>()!=i+1)throw std::runtime_error("Human identity changed");for(auto key:{"x","y","yaw","speed"})if(!std::isfinite(to[i][key].get<double>()))throw std::runtime_error("Nonfinite human state");}
    last=t;prevX=x;prevY=y;prevYaw=yaw;haveRobot=true;
   }
   const double alpha=std::max(0.,std::min(1.,(t-last)/planningDt));people=from;
   for(int i=0;i<population;i++){double dx=to[i]["x"].get<double>()-from[i]["x"].get<double>(),dy=to[i]["y"].get<double>()-from[i]["y"].get<double>();
    people[i]["x"]=from[i]["x"].get<double>()+alpha*dx;people[i]["y"]=from[i]["y"].get<double>()+alpha*dy;
    people[i]["yaw"]=from[i]["yaw"].get<double>()+alpha*std::remainder(to[i]["yaw"].get<double>()-from[i]["yaw"].get<double>(),2*M_PI);people[i]["speed"]=std::hypot(dx,dy)/planningDt;people[i]["vx"]=dx/planningDt;people[i]["vy"]=dy/planningDt;}
   Apply(ecm,std::chrono::duration<double>(info.dt).count());
   if(lastPublish<0||t-lastPublish>=.02-1e-7){auto currentGroups=groups;for(auto &g:currentGroups){double cx=0,cy=0;for(auto id:g["members"]){cx+=people[id.get<int>()-1]["x"].get<double>();cy+=people[id.get<int>()-1]["y"].get<double>();}g["center"]={cx/g["members"].size(),cy/g["members"].size()};}
    json state={{"adapter_ok",true},{"time",t},{"step",step},{"interval_start",last},{"humans",people},{"groups",currentGroups},{"robot",{{"x",x},{"y",y},{"yaw",yaw}}},{"robot_input",robotInput}};
    gz::msgs::StringMsg m;m.set_data(state.dump());publisher.Publish(m);lastPublish=t;}
  }catch(const std::exception &e){Fail(e.what(),t);}
 }
};
}
GZ_ADD_PLUGIN(response_experiment::Adapter,gz::sim::System,response_experiment::Adapter::ISystemConfigure,response_experiment::Adapter::ISystemPreUpdate)
