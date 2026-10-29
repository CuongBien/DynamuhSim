// Link the installed NavFn implementation; never patch its source or cost constants.
#include <nav2_navfn_planner/navfn.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <vector>
#include <algorithm>
#include <stdexcept>
int main(int argc,char**argv){
 try {
  if(argc!=8)throw std::runtime_error("usage: navfn_probe raw.u8 width height start_x start_y goal_x goal_y");
  int w=std::stoi(argv[2]),h=std::stoi(argv[3]);int source[2]={std::stoi(argv[4]),std::stoi(argv[5])},target[2]={std::stoi(argv[6]),std::stoi(argv[7])};
  if(w<3||h<3||w>4000||h>4000)throw std::runtime_error("Invalid dimensions");
  for(auto p:{source,target})if(p[0]<1||p[0]>=w-1||p[1]<1||p[1]>=h-1)throw std::runtime_error("Endpoint outside interior");
  std::ifstream file(argv[1],std::ios::binary);if(!file)throw std::runtime_error("Missing native costmap");
  std::vector<unsigned char> data((std::istreambuf_iterator<char>(file)),{});
  if(data.size()!=static_cast<size_t>(w*h))throw std::runtime_error("Payload size mismatch");
  if(data[target[1]*w+target[0]]>=253&&data[target[1]*w+target[0]]!=255){std::cout<<"{\"found\":false,\"reason\":\"blocked target\"}\n";return 0;}
  data[source[1]*w+source[0]]=0; // Same starting-cell clearing as NavfnPlanner.
  nav2_navfn_planner::NavFn nav(w,h);nav.setCostmap(data.data(),true,true);
  // NavfnPlanner propagates from robot cell and traces back from requested goal.
  nav.setGoal(source);nav.setStart(target);nav.calcNavFnDijkstra([]{return false;},true);
  float potential=nav.potarr[target[1]*w+target[0]];
  int n=potential<POT_HIGH?nav.calcPath(4*std::max(w,h)):0;
  std::cout<<std::setprecision(10)<<"{\"found\":"<<(n>0?"true":"false")<<",\"potential\":"<<potential<<",\"cost_neutral\":"<<COST_NEUTRAL<<",\"cost_factor\":"<<COST_FACTOR<<",\"path_cells\":[";
  for(int i=n-1;i>=0;--i){if(i!=n-1)std::cout<<",";std::cout<<"["<<nav.getPathX()[i]<<","<<nav.getPathY()[i]<<"]";}
  std::cout<<"]}\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
