# Factory regression: existing treatment versus five-second MPPI candidate

Four fresh headless native GPU runs, declared before execution in plan.json: standing and walking, baseline v4 versus candidate v6, seed45, six people. Initial/backend geometry is matched within each scenario. Human reactions differ with robot motion; this is not fixed trajectory replay. MPPI noise RNG is not fixed.

Only CostCritic weight16->3.81, near_collision_cost140->253 and time_steps30->100 differ. Native camera/SDF, aligned URDF/TF, radius .17 + padding .01, inflation .45, cost scale3 and speed caps remain unchanged. The previous fixed-gap acceptance is not used as evidence of social behavior here.

Capture integrity, physical goal arrival, descriptive swept human clearance and static clearance are separate from social review. Robot-center entry into the triangle of a standing three-person group is reported, including the subset whose native speeds are all<=.05m/s. This is a narrow diagnostic, not an O-space/persona label. A human-gap drop greater than .10m versus baseline is flagged for review.

No defaults are promoted, no WM/RL model or classifier is trained/modified, and failed or uncomfortable outcomes are preserved. The last shared unit suite was250 tests OK (2 conditional skips); this stage changes no runtime code and uses the frozen capture and audit entrypoints.
