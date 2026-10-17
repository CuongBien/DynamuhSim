// Isolated pose/transport adapter. All human behavior executes in upstream Python.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
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
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
using json = nlohmann::json;
namespace social_experiment {
class Adapter : public gz::sim::System, public gz::sim::ISystemConfigure,
                public gz::sim::ISystemPreUpdate {
  int fd{-1};
  bool failed{false}, initialized{false}, haveRobot{false};
  double last{-1}, prevX{0}, prevY{0}, prevYaw{0};
  std::string socketPath;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  json people = json::array();
  double animation[3]{0,0,0};
 public:
  ~Adapter() override { if (fd >= 0) close(fd); }
  void Configure(const gz::sim::Entity&, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager&, gz::sim::EventManager&) override {
    socketPath = sdf->Get<std::string>("socket");
    publisher = node.Advertise<gz::msgs::StringMsg>("/socnavgym/ground_truth");
  }
  json Exchange(const json &request) {
    if (fd < 0) {
      fd = socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0) throw std::runtime_error("socket failed");
      timeval timeout{10, 0};
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
      sockaddr_un address{}; address.sun_family = AF_UNIX;
      if (socketPath.size() >= sizeof(address.sun_path)) throw std::runtime_error("socket path too long");
      std::strcpy(address.sun_path, socketPath.c_str());
      if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
        throw std::runtime_error("worker unavailable");
    }
    std::string bytes = request.dump() + "\n";
    size_t sent = 0;
    while (sent < bytes.size()) {
      auto n = send(fd, bytes.data()+sent, bytes.size()-sent, MSG_NOSIGNAL);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw std::runtime_error(std::string("worker send failed: ")+std::strerror(errno));
      sent += n;
    }
    std::string reply;
    char c;
    while (reply.size() < 65536) {
      auto n = recv(fd, &c, 1, 0);
      if (n < 0 && errno == EINTR) continue;
      if (n != 1) throw std::runtime_error(std::string("worker timeout/disconnected: ")+std::strerror(errno));
      if (c == '\n') return json::parse(reply);
      reply += c;
    }
    throw std::runtime_error("worker response too large");
  }
  void Apply(gz::sim::EntityComponentManager &ecm, double dt) {
    for (const auto &h : people) {
      int id = h.at("id");
      double x=h.at("x"), y=h.at("y"), yaw=h.at("yaw"), speed=h.at("speed");
      auto model = ecm.EntityByComponents(gz::sim::components::Model(),
          gz::sim::components::Name("socnavgym_"+std::to_string(id)+"_proxy"));
      if (model != gz::sim::kNullEntity)
        gz::sim::Model(model).SetWorldPoseCmd(ecm, gz::math::Pose3d(x,y,.85,0,0,yaw));
      auto actor = ecm.EntityByComponents(gz::sim::components::Name("socnavgym_"+std::to_string(id)+"_visual"));
      if (actor != gz::sim::kNullEntity) {
        gz::sim::Actor a(actor);
        animation[id-1] += speed * dt * 4.1534;
        a.SetTrajectoryPose(ecm, gz::math::Pose3d(x,y,1.0,0,0,yaw));
        a.SetAnimationName(ecm, "walk");
        a.SetAnimationTime(ecm, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(animation[id-1])));
        ecm.SetChanged(actor, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);
        ecm.SetChanged(actor, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
      }
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    if (failed) { Apply(ecm,0); return; }
    const double t=std::chrono::duration<double>(info.simTime).count();
    if (last >= 0 && t < last) { Fail("simulation reset requires a fresh adapter session", t); return; }
    if (last >= 0 && t-last < .1-1e-7) { Apply(ecm,0); return; }
    auto robot = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("mecanum_robot"));
    if (robot == gz::sim::kNullEntity) return;
    auto p = gz::sim::worldPose(robot, ecm);
    double x=p.Pos().X(), y=p.Pos().Y(), yaw=p.Rot().Yaw();
    double dt=last < 0 ? 0 : t-last;
    double vx=haveRobot ? (x-prevX)/dt : 0, vy=haveRobot ? (y-prevY)/dt : 0;
    double omega=haveRobot ? std::remainder(yaw-prevYaw,2*M_PI)/dt : 0;
    try {
      json request={{"time",t},{"advance",initialized},{"dt",dt},
        {"robot",{{"x",x},{"y",y},{"yaw",yaw},{"vx",vx},{"vy",vy},{"omega",omega}}}};
      auto reply=Exchange(request);
      if (reply.contains("error")) throw std::runtime_error(reply["error"].get<std::string>());
      if (std::abs(reply.at("time").get<double>()-t)>1e-7 || reply.at("humans").size()!=3)
        throw std::runtime_error("invalid worker timestamp/population");
      bool seen[4]{false,false,false,false};
      for (const auto &h : reply["humans"]) {
        int id=h.at("id");
        if (id<1 || id>3 || seen[id]) throw std::runtime_error("invalid or duplicate human id");
        seen[id]=true;
        for (auto field : {"x","y","yaw","speed"})
          if (!std::isfinite(h.at(field).get<double>())) throw std::runtime_error("nonfinite human state");
      }
      people=reply["humans"];
      Apply(ecm,dt);
      reply["robot"]=request["robot"];
      reply["adapter_ok"]=true;
      gz::msgs::StringMsg message; message.set_data(reply.dump()); publisher.Publish(message);
      last=t; prevX=x; prevY=y; prevYaw=yaw; haveRobot=true; initialized=true;
    } catch (const std::exception &e) { Fail(e.what(),t); }
  }
  void Fail(const std::string &why, double t) {
    failed=true;
    std::cerr << "[SocNavGym adapter] frozen: " << why << std::endl;
    gz::msgs::StringMsg message;
    message.set_data(json({{"adapter_ok",false},{"time",t},{"error",why}}).dump());
    publisher.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(social_experiment::Adapter, gz::sim::System,
              social_experiment::Adapter::ISystemConfigure, social_experiment::Adapter::ISystemPreUpdate)
