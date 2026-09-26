cmake_minimum_required(VERSION 3.22)
project(custom_corridor_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

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

include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists_multi_human_snippet.txt)

ament_package()