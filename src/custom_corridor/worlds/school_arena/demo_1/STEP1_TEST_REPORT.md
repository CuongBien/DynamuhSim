# Demo 1 — Step 1 acceptance report

Static geometry/map/topology validation: **PASS**

- PASS: Robot A → B (21 graph nodes)
- PASS: Human Room01 → Room12 (25 graph nodes)
- PASS: Human Stair → Room05 (10 graph nodes)
- PASS: Human Corridor → Room (5 graph nodes)

- PASS: full 0.22 m HuNav footprint on every graph edge
- PASS: Room01 wall-crossing shortcut rejected
- PASS: stair-core diagonal shortcut rejected

> PASS here proves the world/map/topology are consistent and route footprints do
> not intersect occupied map cells. Run nav2_plan_clearance_test.py and
> nav2_baseline_test.py for live Gazebo/Nav2 acceptance.
