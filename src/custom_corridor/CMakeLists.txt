cmake_minimum_required(VERSION 3.8)

project(custom_corridor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Runtime resources always retain their original install locations.
install(
  DIRECTORY
    launch
    models
    urdf
    rviz
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)


# Historical datasets/episode outputs are source data, not runtime package assets.
# Fresh prefixes use the lean install by default. Existing install trees are not
# deleted by CMake; use a new --install-base to remove an old full-install footprint.
option(DYNAMUHSIM_INSTALL_LEGACY_WORLD_PAYLOADS
  "Install historical dataset and generated episode payloads under worlds" OFF)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/config/world_payloads.txt"
  _world_payload_dirs REGEX "^[^#].+")
if(DYNAMUHSIM_INSTALL_LEGACY_WORLD_PAYLOADS)
  foreach(_payload IN LISTS _world_payload_dirs)
    string(STRIP "${_payload}" _payload)
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/worlds/${_payload}")
      message(FATAL_ERROR
        "Legacy world payload missing: ${_payload}. From the repository root run "
        "python3 tools/hydrate_history.py --source git --group world:${_payload} "
        "--report /tmp/hydrate-report.json, then configure again. "
        "See docs/HISTORICAL_ARTIFACTS.md for archive hydration.")
    endif()
  endforeach()
  install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
else()
  set(_world_payload_excludes)
  foreach(_payload IN LISTS _world_payload_dirs)
    string(STRIP "${_payload}" _payload)
    # CMake copy install excludes directories; ament symlink install matches
    # recursive file globs. Both forms are needed for the same result in each mode.
    list(APPEND _world_payload_excludes
      PATTERN "${_payload}" EXCLUDE
      PATTERN "${_payload}/*" EXCLUDE)
  endforeach()
  install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}
    ${_world_payload_excludes})
endif()

install(
  PROGRAMS
    scripts/goal_pose_bridge.py
    scripts/hospital_auto_route.py
    scripts/d435i_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
