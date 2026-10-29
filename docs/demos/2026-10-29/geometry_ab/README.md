# Native geometry / inflation physical comparison

The native Gazebo robot is unchanged. Recipe v4 generates a run-local URDF from that SDF, including collision primitives and fixed transforms, and declares a 0.17 m Nav2 circle with 0.01 m padding. Existing recipe v1-v3 defaults remain unchanged.

The predeclared arms in `plan.json` compare the old recorded baseline with corrected geometry at inflation 0.80 m and 0.45 m. Both new arms use standing seed 45 and headless NVIDIA rendering. No waypoint or artificial command forces passage through people.

This is a diagnostic single-seed comparison. It is not a multi-seed navigation benchmark, social-policy evaluation, or World Model training dataset release. Goal completion, narrow-gap crossing, and descriptive geometric clearance are reported separately. Original captures are preserved.
