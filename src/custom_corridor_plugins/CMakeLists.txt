cmake_minimum_required(VERSION 3.22)
project(custom_corridor_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(oscillating_obstacle_system SHARED
  src/oscillating_obstacle_system.cc
)

target_compile_features(oscillating_obstacle_system PUBLIC cxx_std_17)
target_link_libraries(oscillating_obstacle_system PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

install(TARGETS oscillating_obstacle_system
  LIBRARY DESTINATION lib
)

add_library(multi_human_scenario_system SHARED
  src/multi_human_scenario_system.cpp
)

target_compile_features(multi_human_scenario_system PRIVATE cxx_std_17)
target_link_libraries(multi_human_scenario_system PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::core
  gz-plugin2::gz-plugin2
  nlohmann_json::nlohmann_json
)

install(TARGETS multi_human_scenario_system
  LIBRARY DESTINATION lib
)

ament_environment_hooks()

ament_package()