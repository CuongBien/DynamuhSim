// Isolated Ogre2 camera listener: temporary render visibility, no physics/ECM changes.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2DepthCamera.hh>
#include <OgreCamera.h>
#include <algorithm>
#include <iostream>
#include <set>
#include <vector>
namespace depth_proxy_experiment {
class Filter : public gz::sim::System, public gz::sim::ISystemConfigure,
               public Ogre::Camera::Listener {
  gz::common::ConnectionPtr updateConnection,teardownConnection;
  std::set<Ogre::Camera*> cameras;
  std::vector<gz::rendering::VisualPtr> proxies;
  std::vector<std::vector<unsigned int>> saved;
  unsigned int preCount=0,postCount=0;
 public:
  void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&,
      gz::sim::EntityComponentManager&,gz::sim::EventManager &events) override {
    updateConnection=events.Connect<gz::sim::events::SceneUpdate>([this]{Update();});
    teardownConnection=events.Connect<gz::sim::events::RenderTeardown>([this]{Clear();});
  }
  void Update() {
    auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
    if(proxies.empty()) {
      for(unsigned int i=0;i<scene->VisualCount();++i) {
        auto visual=scene->VisualByIndex(i);const auto name=visual->Name();
        if(name.find("common_")!=std::string::npos && name.find("_proxy")!=std::string::npos && name.find("body_visual")!=std::string::npos)
          proxies.push_back(visual);
      }
      if(!proxies.empty())std::cout<<"[DepthProxyFilter] proxies="<<proxies.size()<<std::endl;
    }
    for(unsigned int i=0;i<scene->SensorCount();++i) {
      auto sensor=scene->SensorByIndex(i);const auto name=sensor->Name();
      if(name.find("proxy_low_off")==std::string::npos && name.find("proxy_high_off")==std::string::npos)continue;
      auto depth=std::dynamic_pointer_cast<gz::rendering::Ogre2DepthCamera>(sensor);
      if(depth && depth->OgreCamera() && cameras.insert(depth->OgreCamera()).second) {
        depth->OgreCamera()->addListener(this);
        std::cout<<"[DepthProxyFilter] registered="<<name<<std::endl;
      }
    }
  }
  void cameraPreRenderScene(Ogre::Camera *) override {
    std::vector<unsigned int> flags;
    for(auto &visual:proxies){flags.push_back(visual->VisibilityFlags());visual->SetVisibilityFlags(0);}
    saved.push_back(std::move(flags));++preCount;
    if(preCount==1)std::cout<<"[DepthProxyFilter] pre_render proxies="<<proxies.size()<<std::endl;
  }
  void cameraPostRenderScene(Ogre::Camera *) override {
    Restore();++postCount;
    if(postCount==1)std::cout<<"[DepthProxyFilter] post_render restored"<<std::endl;
  }
  void cameraDestroyed(Ogre::Camera *camera) override {cameras.erase(camera);}
  void Restore() {
    if(saved.empty())return;
    const auto flags=std::move(saved.back());saved.pop_back();
    for(size_t i=0;i<flags.size();++i)proxies[i]->SetVisibilityFlags(flags[i]);
  }
  void Clear() {
    while(!saved.empty())Restore();
    for(auto *camera:cameras)camera->removeListener(this);
    cameras.clear();proxies.clear();
    std::cout<<"[DepthProxyFilter] teardown pre="<<preCount<<" post="<<postCount<<std::endl;
  }
};
}
GZ_ADD_PLUGIN(depth_proxy_experiment::Filter,gz::sim::System,gz::sim::ISystemConfigure)
