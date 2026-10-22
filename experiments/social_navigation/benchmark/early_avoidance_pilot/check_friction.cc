// Exact installed SDF API semantics used by gz-physics7 DART friction loader.
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <iostream>
int main(int argc,char **argv){
 if(argc!=2)return 2;sdf::Root root;auto errors=root.Load(argv[1]);
 if(!errors.empty()){for(auto &e:errors)std::cerr<<e.Message()<<std::endl;return 3;}
 auto model=root.Model();int total=0,recognized=0;
 for(uint64_t i=0;i<model->LinkCount();++i){auto link=model->LinkByIndex(i);
  for(uint64_t j=0;j<link->CollisionCount();++j){auto e=link->CollisionByIndex(j)->Element();
   if(!e->HasElement("surface"))continue;auto ode=e->GetElement("surface")->GetElement("friction")->GetElement("ode");
   if(!ode->HasElement("fdir1"))continue;auto f=ode->GetElement("fdir1");++total;
   if(f->HasAttribute("gz:expressed_in") && f->Get<std::string>("gz:expressed_in")=="base_link")++recognized;
  }}
 std::cout<<"{\"friction_directions\":"<<total<<",\"recognized_gz_base_link_frames\":"<<recognized<<"}"<<std::endl;
 return total==4 && recognized==4?0:1;
}
