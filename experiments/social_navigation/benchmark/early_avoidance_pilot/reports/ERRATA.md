# Correction to prior dynamic depth pilot claims

The previous depth_proxy_lifecycle/motion01 used a runtime robot copy whose XML serialization changed gz:expressed_in to ns0:expressed_in on all four wheel friction directions. Gazebo Physics7 DART checks the literal gz:expressed_in name. The runtime therefore did not preserve original friction-frame interpretation, despite equal XML expanded names, collision geometry and untouched src/ files.

The previous guard failure is a real measured event and its0.311m/s feasibility calculation is correct for the captured state. It cannot be treated as a clean original-physics social-navigation test or attributed solely to the human policy. Its dynamic camera prefix remains measured sensor evidence in that altered setup, not certified native robot dynamics.

The same bug affects avoidance01, avoidance_truth01 and avoidance_truth_wide01 in this stage. Their TIMEOUT outcomes, raw files, earlier structural audits and correction labels are all preserved. These are INVALID_NATIVE_PHYSICS_SETUP, excluded from configuration success rates and training. Oracle and wider angular sampling did not fix the underlying serialization error.

A canonical XML equality check missed this Gazebo-specific literal-prefix behavior. check_friction.cc now exercises the installed native SDF API; the original robot and repaired copy resolve4/4frames, the bad runtime0/4. The new namespace_prepare.py restores the literal prefix without numeric physical changes. The public run_validated.py uses this helper and original AMCL/odometry.

The older dataset_pilot/pilot01 used string-only metadata correction and retains4/4frames; its prior raw dataset was not changed. Static camera/BEV and proxy-visibility comparisons do not drive the robot and remain scoped to static sensor evidence; their collision-definition parity was never a dynamic-contact test. Original src/, native worker/vendor code and old reports/raw evidence are retained unchanged.

Source: [gz-physics7 DART ConstructSdfCollision](https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7/dartsim/src/SDFFeatures.cc), literal frictionDirectionElem HasAttribute lookup.
