# Preserve the literal Gazebo friction-frame attribute

ElementTree preserved the expanded XML name while rewriting gz:expressed_in to ns0:expressed_in in the previous experiment camera copier. gz-physics7 DART checks the literal name gz:expressed_in, so it stopped resolving all four wheel friction directions in base_link. The original src/ robot was not modified. The old dataset pilot used a string-only camera metadata repair and retained the correct literal prefix.

namespace_prepare.py wraps the old camera copier and registers/preserves the gz prefix before writing its runtime copy. No numerical friction, geometry, drive parameter, ROS source or old evidence is changed. The installed SDF API checker recognizes4frames in the original,0in the bad runtime and4in the repaired copy. An XML canonicalization comparison alone cannot detect this Gazebo-specific semantic difference.

Source: [Gazebo Physics7 DART SDFFeatures](https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7/dartsim/src/SDFFeatures.cc), ConstructSdfCollision friction-frame lookup.

Existing Jazzy workspace, from repository root:

```bash
source /opt/ros/jazzy/setup.bash
TASK=experiments/social_navigation/benchmark/early_avoidance_pilot
mkdir -p "$TASK/generated"
g++ -std=c++17 "$TASK/check_friction.cc" -o "$TASK/generated/check_friction" $(pkg-config --cflags --libs sdformat14)
python3 "$TASK/test_replay.py"
```

The regression constructs a bad-prefix copy from the original tracked robot, proves rejection with the installed native parser, then verifies the repaired copy. No GPU or ROS processes are started. Use the new copier explicitly for experiment runtime files; previous reports/raw files remain unchanged, with corrections recorded in the next experiment report.

The standalone duplicate friction test was removed on 27/10/2026; its identical native-parser case remains in the four-test replay suite.
