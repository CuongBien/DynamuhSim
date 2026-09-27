#!/usr/bin/env python3
from pathlib import Path

OUT = Path(__file__).resolve().parent / "school_arena.sdf"

WALL_T = 0.12
WALL_H = 2.60
DOOR_W = 0.90
CORRIDOR_W = 1.20

parts = []

def add(s):
    parts.append(s)

def box(name, x, y, sx, sy, sz=WALL_H, z=None,
        color="0.82 0.82 0.82 1", static=True):
    if z is None:
        z = sz / 2.0
    add(f"""
    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
""")

def cylinder(name, x, y, radius, length, z=None,
             color="0.20 0.55 0.25 1", static=True):
    if z is None:
        z = length / 2.0
    add(f"""
    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
""")

def wall_h(name, x0, x1, y, h=WALL_H, color="0.88 0.88 0.90 1"):
    if x1 <= x0:
        return
    box(name, (x0+x1)/2, y, x1-x0, WALL_T, h, color=color)

def wall_v(name, x, y0, y1, h=WALL_H, color="0.88 0.88 0.90 1"):
    if y1 <= y0:
        return
    box(name, x, (y0+y1)/2, WALL_T, y1-y0, h, color=color)

def segmented_front_h(prefix, x0, x1, y):
    L = x1 - x0
    d1 = x0 + 0.30 * L
    d2 = x0 + 0.70 * L
    a = DOOR_W / 2
    intervals = [(x0, d1-a), (d1+a, d2-a), (d2+a, x1)]
    for i, (a0, a1) in enumerate(intervals):
        wall_h(f"{prefix}_front_{i}", a0, a1, y)

def segmented_front_v(prefix, y0, y1, x):
    L = y1 - y0
    d1 = y0 + 0.30 * L
    d2 = y0 + 0.70 * L
    a = DOOR_W / 2
    intervals = [(y0, d1-a), (d1+a, d2-a), (d2+a, y1)]
    for i, (a0, a1) in enumerate(intervals):
        wall_v(f"{prefix}_front_{i}", x, a0, a1)

def room(name, x0, x1, y0, y1, front):
    c = "0.92 0.90 0.82 1"
    if front == "N":
        wall_h(f"{name}_south", x0, x1, y0, color=c)
        wall_v(f"{name}_west", x0, y0, y1, color=c)
        wall_v(f"{name}_east", x1, y0, y1, color=c)
        segmented_front_h(name, x0, x1, y1)
    elif front == "S":
        wall_h(f"{name}_north", x0, x1, y1, color=c)
        wall_v(f"{name}_west", x0, y0, y1, color=c)
        wall_v(f"{name}_east", x1, y0, y1, color=c)
        segmented_front_h(name, x0, x1, y0)
    elif front == "E":
        wall_h(f"{name}_south", x0, x1, y0, color=c)
        wall_h(f"{name}_north", x0, x1, y1, color=c)
        wall_v(f"{name}_west", x0, y0, y1, color=c)
        segmented_front_v(name, y0, y1, x1)
    elif front == "W":
        wall_h(f"{name}_south", x0, x1, y0, color=c)
        wall_h(f"{name}_north", x0, x1, y1, color=c)
        wall_v(f"{name}_east", x1, y0, y1, color=c)
        segmented_front_v(name, y0, y1, x0)
    else:
        raise ValueError(front)

def moving_person(name, start_x, start_y, waypoints, color):
    wps = "\n".join(f"          <waypoint>{x:.3f} {y:.3f}</waypoint>"
                    for x, y in waypoints)
    add(f"""
    <model name="{name}">
      <static>false</static>
      <pose>{start_x:.3f} {start_y:.3f} 0.82 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>8.0</mass>
          <inertia>
            <ixx>12.0</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>12.0</iyy><iyz>0</iyz><izz>0.45</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.38 0.38 1.64</size></box></geometry>
          <surface>
            <friction>
              <bullet>
                <friction>0.8</friction>
                <friction2>0.8</friction2>
                <rolling_friction>0.0</rolling_friction>
              </bullet>
            </friction>
          </surface>
        </collision>
        <visual name="body">
          <geometry><cylinder><radius>0.19</radius><length>1.35</length></cylinder></geometry>
          <pose>0 0 -0.10 0 0 0</pose>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 0.76 0 0 0</pose>
          <geometry><sphere><radius>0.16</radius></sphere></geometry>
          <material><ambient>0.82 0.67 0.52 1</ambient><diffuse>0.82 0.67 0.52 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system"
              name="gz::sim::systems::TrajectoryFollower">
        <link_name>base_link</link_name>
        <loop>true</loop>
        <force>7.0</force>
        <torque>8.0</torque>
        <range_tolerance>0.18</range_tolerance>
        <bearing_tolerance>5</bearing_tolerance>
        <waypoints>
{wps}
        </waypoints>
      </plugin>
    </model>
""")

header = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="school_arena">
    <physics name="school_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.75 0.75 0.78 1</ambient>
      <background>0.92 0.95 1.0 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <pose>0 0 20 0 0 0</pose>
      <direction>-0.4 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material>
            <ambient>0.55 0.58 0.60 1</ambient>
            <diffuse>0.55 0.58 0.60 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
add(header)

# Centerline:
# (-15,-8) -> (15,-8) -> (15,-4) -> (7,-4)
# -> (7,0) -> (15,0) -> (15,14)
# => exactly 5 right-angle turns around the stair core.

x0, x1 = -15.0, 14.4
step = (x1 - x0) / 6.0
for i in range(6):
    a, b = x0 + i*step, x0 + (i+1)*step
    room(f"classroom_{i+1:02d}", a, b, -12.6, -8.6, "N")

x0n, x1n = -15.0, 4.6
stepn = (x1n - x0n) / 4.0
for i in range(4):
    a, b = x0n + i*stepn, x0n + (i+1)*stepn
    room(f"classroom_{i+7:02d}", a, b, -7.4, -3.4, "S")

y0e, y1e = 0.6, 14.0
stepe = (y1e - y0e) / 3.0
for i in range(3):
    a, b = y0e + i*stepe, y0e + (i+1)*stepe
    room(f"classroom_{i+11:02d}", 15.6, 19.6, a, b, "W")

y0w, y1w = 4.6, 14.0
stepw = (y1w - y0w) / 2.0
for i in range(2):
    a, b = y0w + i*stepw, y0w + (i+1)*stepw
    room(f"classroom_{i+14:02d}", 10.4, 14.4, a, b, "E")

wall_v("corridor_A_west_end", -15.0, -8.6, -7.4)
wall_h("corridor_A_north_filler", 4.6, 14.4, -7.4)
wall_v("corridor_B_outer_east", 15.6, -8.6, -3.4)
wall_v("corridor_B_inner_west_lower", 14.4, -7.4, -4.6)
wall_h("corridor_C_outer_south", 6.4, 15.6, -4.6)
wall_v("corridor_D_outer_west", 6.4, -4.6, 0.6)
wall_h("corridor_E_outer_north", 6.4, 14.4, 0.6)
wall_v("corridor_F_west_lower", 14.4, 0.6, 4.6)
wall_h("corridor_F_north_end", 14.4, 15.6, 14.0)

wall_h("stairs_south_wall", 7.6, 14.4, -3.4, color="0.58 0.58 0.62 1")
wall_v("stairs_west_wall", 7.6, -3.4, -0.6, color="0.58 0.58 0.62 1")
wall_h("stairs_north_wall", 7.6, 14.4, -0.6, color="0.58 0.58 0.62 1")
wall_v("stairs_east_wall", 14.4, -3.4, -0.6, color="0.58 0.58 0.62 1")

for i in range(10):
    sx = 0.58
    sy = 2.15
    sh = 0.08 * (i + 1)
    cx = 8.05 + i * 0.61
    box(f"stair_step_{i+1:02d}", cx, -2.0, sx, sy, sh,
        z=sh/2, color="0.45 0.45 0.48 1")

box("bench_01", -9.5, -8.43, 1.40, 0.20, 0.45, z=0.225,
    color="0.45 0.28 0.12 1")
box("bench_02", 0.0, -8.43, 1.40, 0.20, 0.45, z=0.225,
    color="0.45 0.28 0.12 1")
box("locker_01", -5.0, -7.55, 1.80, 0.18, 1.80, z=0.90,
    color="0.25 0.45 0.70 1")
box("locker_02", 2.6, -7.55, 1.40, 0.18, 1.80, z=0.90,
    color="0.25 0.45 0.70 1")
cylinder("trash_bin_01", 14.75, 3.0, 0.16, 0.55,
         color="0.20 0.25 0.28 1")
cylinder("trash_bin_02", 15.25, 10.3, 0.16, 0.55,
         color="0.20 0.25 0.28 1")
box("backpack_01", -1.2, -7.80, 0.32, 0.22, 0.28, z=0.14,
    color="0.70 0.12 0.12 1")
box("backpack_02", 14.72, 8.3, 0.28, 0.22, 0.24, z=0.12,
    color="0.10 0.20 0.65 1")
box("notice_board", 6.52, -1.7, 0.10, 1.30, 1.00, z=1.20,
    color="0.18 0.55 0.35 1")

moving_person("student_head_on_A", -12.0, -8.05,
              [(-12.0, -8.05), (10.0, -8.05)],
              "0.10 0.35 0.85 1")
moving_person("student_opposite_A", 10.0, -7.85,
              [(10.0, -7.85), (-10.0, -7.85)],
              "0.85 0.20 0.20 1")
moving_person("student_corner_B", 15.05, -7.0,
              [(15.05, -7.0), (15.05, -4.1), (10.5, -4.0)],
              "0.60 0.20 0.75 1")
moving_person("student_stair_loop", 14.85, -5.2,
              [(14.85, -5.2), (15.0, -4.0), (7.0, -4.0),
               (7.0, 0.0), (14.9, 0.0), (15.0, 5.0)],
              "0.90 0.55 0.10 1")
moving_person("student_door_exit", -3.73, -10.4,
              [(-3.73, -10.4), (-3.73, -8.05), (2.0, -8.05)],
              "0.10 0.65 0.55 1")
moving_person("student_vertical_up", 14.85, 1.4,
              [(14.85, 1.4), (14.85, 12.4)],
              "0.75 0.40 0.15 1")
moving_person("student_vertical_down", 15.15, 12.0,
              [(15.15, 12.0), (15.15, 2.0)],
              "0.35 0.20 0.80 1")
moving_person("student_short_cross", 15.15, 6.4,
              [(15.15, 6.4), (14.65, 6.4), (15.35, 6.4)],
              "0.15 0.65 0.20 1")

add("""
  </world>
</sdf>
""")

OUT.write_text("".join(parts), encoding="utf-8")
print(f"Wrote: {OUT}")
print("Expected: 15 classrooms, 30 door openings, 1.2 m corridor, 5 turns, 8 moving people.")
