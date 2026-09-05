from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "worlds"

LENGTH = 30.0
X_MIN = -15.0
X_MAX = 15.0

WALL_HEIGHT = 2.0
WALL_THICKNESS = 0.10

RECESS_DEPTH = 0.60
RECESS_WIDTH = 1.20
RECESS_X = 2.0


def box(name, sx, sy, sz, x, y, z):
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{sx} {sy} {sz}</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{sx} {sy} {sz}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>

      <pose>{x} {y} {z} 0 0 0</pose>
    </model>
    """


def generate(width):
    half = width / 2.0
    t = WALL_THICKNESS
    h = WALL_HEIGHT

    left_y = -half - t / 2.0
    right_y = half + t / 2.0

    # Recess opening
    half_open = RECESS_WIDTH / 2.0

    left_segment_length = (RECESS_X - half_open) - X_MIN
    right_segment_start = RECESS_X + half_open
    right_segment_length = X_MAX - right_segment_start

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="corridor_{width:.2f}">

    <gravity>0 0 -9.81</gravity>

    <!-- Physics -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Ground -->
    {box(
        "ground",
        34.0, 10.0, 0.10,
        0.0, 0.0, -0.05
    )}

    <!-- Left continuous wall -->
    {box(
        "left_wall",
        LENGTH, t, h,
        0.0, left_y, h / 2.0
    )}

    <!-- Right wall: segment before recess -->
    {box(
        "right_wall_front",
        left_segment_length, t, h,
        X_MIN + left_segment_length / 2.0,
        right_y,
        h / 2.0
    )}

    <!-- Right wall: segment after recess -->
    {box(
        "right_wall_back",
        right_segment_length, t, h,
        right_segment_start + right_segment_length / 2.0,
        right_y,
        h / 2.0
    )}

    <!-- Recess side walls -->
    {box(
        "recess_side_left",
        t, RECESS_DEPTH, h,
        RECESS_X - half_open - t / 2.0,
        half + RECESS_DEPTH / 2.0,
        h / 2.0
    )}

    {box(
        "recess_side_right",
        t, RECESS_DEPTH, h,
        RECESS_X + half_open + t / 2.0,
        half + RECESS_DEPTH / 2.0,
        h / 2.0
    )}

    <!-- Recess back wall -->
    {box(
        "recess_back",
        RECESS_WIDTH + 2.0 * t,
        t,
        h,
        RECESS_X,
        half + RECESS_DEPTH + t / 2.0,
        h / 2.0
    )}

    <!-- Sun -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <direction>-0.5 -0.5 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

  </world>
</sdf>
"""

    filename = OUT / ("corridor_" + f"{width:.2f}".replace(".", "") + ".sdf")
    filename.write_text(sdf)

    print(f"Generated: {filename}")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)

    for width in (0.70, 0.90, 1.20):
        generate(width)
