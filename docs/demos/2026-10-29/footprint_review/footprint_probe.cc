#include <nav2_costmap_2d/footprint.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
int main(){nlohmann::json result=nlohmann::json::array();for(double r:{0.25,0.17}){auto p=nav2_costmap_2d::makeFootprintFromRadius(r);nav2_costmap_2d::padFootprint(p,0.01);auto radii=nav2_costmap_2d::calculateMinAndMaxDistances(p);nlohmann::json points=nlohmann::json::array();double x=0,y=0;for(auto& q:p){points.push_back({q.x,q.y});x=std::max(x,std::abs(q.x));y=std::max(y,std::abs(q.y));}result.push_back({{"robot_radius",r},{"padding",0.01},{"points",points},{"inscribed_radius",radii.first},{"circumscribed_radius",radii.second},{"width_x",2*x},{"width_y",2*y}});}std::cout<<result.dump(2)<<std::endl;}
