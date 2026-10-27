// Label-only per-camera render intervention; never changes physics or sensor inputs.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2DepthCamera.hh>
#include <OgreCamera.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
namespace visibility_experiment {
struct Item {gz::rendering::VisualPtr visual; int human; bool proxy;};
struct Saved {gz::rendering::VisualPtr visual; unsigned int flags;};
class Filter : public gz::sim::System, public gz::sim::ISystemConfigure, public Ogre::Camera::Listener {
 gz::common::ConnectionPtr update,teardown;
 std::map<Ogre::Camera*,std::string> cameras;
 std::vector<Item> items;
 std::vector<std::vector<Saved>> stack;
 unsigned int pre=0,post=0; bool listed=false;
 public:
 void Configure(const gz::sim::Entity&,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager&,gz::sim::EventManager &events) override {
  update=events.Connect<gz::sim::events::SceneUpdate>([this]{Update();});
  teardown=events.Connect<gz::sim::events::RenderTeardown>([this]{Clear();});
 }
 void Update(){
  auto scene=gz::rendering::sceneFromFirstRenderEngine();if(!scene)return;
  items.clear();int found=0;
  for(unsigned int i=0;i<scene->VisualCount();++i){
   auto v=scene->VisualByIndex(i);if(v->GeometryCount()==0)continue;
   std::string path=v->Name();auto parent=v->Parent();
   while(parent){path+="/"+parent->Name();parent=parent->Parent();}
   bool proxy=path.find("_proxy")!=std::string::npos;int human=0;
   for(int h=1;h<=6;++h)if(path.find("common_"+std::to_string(h)+"_visual")!=std::string::npos){human=h;break;}
   items.push_back({v,human,proxy});if(human)++found;
   if(!listed&&(human||proxy))std::cout<<"[Visibility] geometry="<<v->Name()<<" human="<<human<<" proxy="<<proxy<<std::endl;
  }
  if(found>=6&&!listed){std::cout<<"[Visibility] ready human_geometries="<<found<<std::endl;listed=true;}
  for(unsigned int i=0;i<scene->SensorCount();++i){
   auto sensor=scene->SensorByIndex(i);std::string name=sensor->Name();auto pos=name.find("visibility_");if(pos==std::string::npos)continue;
   auto depth=std::dynamic_pointer_cast<gz::rendering::Ogre2DepthCamera>(sensor);
   if(depth&&depth->OgreCamera()&&!cameras.count(depth->OgreCamera())){
    cameras[depth->OgreCamera()]=name.substr(pos+11);depth->OgreCamera()->addListener(this);
    std::cout<<"[Visibility] registered="<<cameras[depth->OgreCamera()]<<std::endl;
   }
  }
 }
 void cameraPreRenderScene(Ogre::Camera *camera) override {
  const auto mode=cameras.at(camera);std::vector<Saved> saved;
  int drop=mode.rfind("drop",0)==0?std::stoi(mode.substr(4)):0;
  int only=mode.rfind("only",0)==0?std::stoi(mode.substr(4)):0;
  // Save before any flag mutations; restoring parent/child flags uses the same order.
  for(const auto &i:items){bool hide=i.proxy||(drop&&i.human==drop)||(mode=="null"&&i.human)||(only&&i.human!=only);
   if(hide)saved.push_back({i.visual,i.visual->VisibilityFlags()});}
  for(auto &i:saved)i.visual->SetVisibilityFlags(0);
  stack.push_back(std::move(saved));++pre;
 }
 void Restore(){if(stack.empty())return;auto saved=std::move(stack.back());stack.pop_back();for(auto &i:saved)i.visual->SetVisibilityFlags(i.flags);}
 void cameraPostRenderScene(Ogre::Camera*) override {Restore();++post;}
 void cameraDestroyed(Ogre::Camera *camera) override {cameras.erase(camera);}
 void Clear(){while(!stack.empty())Restore();for(auto &c:cameras)c.first->removeListener(this);cameras.clear();items.clear();std::cout<<"[Visibility] teardown pre="<<pre<<" post="<<post<<std::endl;}
};
}
GZ_ADD_PLUGIN(visibility_experiment::Filter,gz::sim::System,gz::sim::ISystemConfigure)
