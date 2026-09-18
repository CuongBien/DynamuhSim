cmake_minimum_required(VERSION 3.8)

project(custom_corridor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install corridor worlds, launch files and robot models
install(
  DIRECTORY
    worlds
    launch
    models
    rviz
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/goal_pose_bridge.py
    scripts/d435i_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
