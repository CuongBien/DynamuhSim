#!/usr/bin/env python3

from pathlib import Path

OUTPUT = Path(__file__).parent / "school_arena_1.sdf"

# =========================================================
# CONFIG
# =========================================================

CORRIDOR_WIDTH = 1.20

ROOM_WIDTH = 4.0
ROOM_DEPTH = 4.0

NUM_TOP_ROOMS = 8
NUM_BOTTOM_ROOMS = 7

DOOR_WIDTH = 0.90

WALL_THICKNESS = 0.12
WALL_HEIGHT = 2.7

ROOM_COLOR = "0.90 0.88 0.78 1"
CORRIDOR_COLOR = "0.55 0.58 0.60 1"

models = []


# =========================================================
# BASIC MODEL
# =========================================================

def add_box(
    name,
    x,
    y,
    sx,
    sy,
    sz,
    color="0.8 0.8 0.8 1",
    z=None
):

    if z is None:
        z = sz / 2.0

    models.append(f"""
    <model name="{name}">
        <static>true</static>

        <pose>
            {x} {y} {z} 0 0 0
        </pose>

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
                    <ambient>{color}</ambient>
                    <diffuse>{color}</diffuse>
                </material>

            </visual>

        </link>

    </model>
    """)


# =========================================================
# WALL FUNCTIONS
# =========================================================

def horizontal_wall(name, x1, x2, y):

    length = x2 - x1

    if length <= 0:
        return

    add_box(
        name,
        (x1 + x2) / 2,
        y,
        length,
        WALL_THICKNESS,
        WALL_HEIGHT,
        ROOM_COLOR
    )


def vertical_wall(name, x, y1, y2):

    length = y2 - y1

    if length <= 0:
        return

    add_box(
        name,
        x,
        (y1 + y2) / 2,
        WALL_THICKNESS,
        length,
        WALL_HEIGHT,
        ROOM_COLOR
    )


# =========================================================
# ROOM WITH TWO DOORS
# =========================================================

def corridor_wall_with_two_doors(
    name,
    x_left,
    x_right,
    y
):

    room_w = x_right - x_left

    door1 = x_left + room_w * 0.30
    door2 = x_left + room_w * 0.70

    half_door = DOOR_WIDTH / 2

    # wall section 1
    horizontal_wall(
        f"{name}_wall_1",
        x_left,
        door1 - half_door,
        y
    )

    # wall section 2
    horizontal_wall(
        f"{name}_wall_2",
        door1 + half_door,
        door2 - half_door,
        y
    )

    # wall section 3
    horizontal_wall(
        f"{name}_wall_3",
        door2 + half_door,
        x_right,
        y
    )


def create_top_room(index, x_left):

    x_right = x_left + ROOM_WIDTH

    corridor_top = CORRIDOR_WIDTH / 2

    y_bottom = corridor_top
    y_top = corridor_top + ROOM_DEPTH

    # back wall
    horizontal_wall(
        f"room_{index}_back",
        x_left,
        x_right,
        y_top
    )

    # left wall
    vertical_wall(
        f"room_{index}_left",
        x_left,
        y_bottom,
        y_top
    )

    # right wall
    vertical_wall(
        f"room_{index}_right",
        x_right,
        y_bottom,
        y_top
    )

    # corridor-facing wall
    corridor_wall_with_two_doors(
        f"room_{index}",
        x_left,
        x_right,
        y_bottom
    )


def create_bottom_room(index, x_left):

    x_right = x_left + ROOM_WIDTH

    corridor_bottom = -CORRIDOR_WIDTH / 2

    y_top = corridor_bottom
    y_bottom = corridor_bottom - ROOM_DEPTH

    # back wall
    horizontal_wall(
        f"room_{index}_back",
        x_left,
        x_right,
        y_bottom
    )

    # left wall
    vertical_wall(
        f"room_{index}_left",
        x_left,
        y_bottom,
        y_top
    )

    # right wall
    vertical_wall(
        f"room_{index}_right",
        x_right,
        y_bottom,
        y_top
    )

    # corridor-facing wall
    corridor_wall_with_two_doors(
        f"room_{index}",
        x_left,
        x_right,
        y_top
    )


# =========================================================
# WORLD HEADER
# =========================================================

header = """<?xml version="1.0"?>

<sdf version="1.9">

<world name="school_arena">

    <physics name="physics" type="ignored">

        <max_step_size>0.001</max_step_size>

        <real_time_factor>1.0</real_time_factor>

    </physics>


    <plugin
        filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>

    <plugin
        filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands"/>

    <plugin
        filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>


    <light type="directional" name="sun">

        <pose>0 0 20 0 0 0</pose>

        <direction>-0.5 0.2 -1</direction>

        <diffuse>0.9 0.9 0.9 1</diffuse>

    </light>


    <model name="ground">

        <static>true</static>

        <link name="link">

            <collision name="collision">

                <geometry>

                    <plane>
                        <normal>0 0 1</normal>
                        <size>80 40</size>
                    </plane>

                </geometry>

            </collision>


            <visual name="visual">

                <geometry>

                    <plane>
                        <normal>0 0 1</normal>
                        <size>80 40</size>
                    </plane>

                </geometry>

                <material>

                    <ambient>
                        0.45 0.47 0.50 1
                    </ambient>

                    <diffuse>
                        0.45 0.47 0.50 1
                    </diffuse>

                </material>

            </visual>

        </link>

    </model>
"""

models.append(header)


# =========================================================
# GENERATE SCHOOL
# =========================================================

school_start_x = -16.0


# ---------------------------------------------------------
# TOP: ROOM 1 -> ROOM 8
# ---------------------------------------------------------

for i in range(NUM_TOP_ROOMS):

    x = school_start_x + i * ROOM_WIDTH

    create_top_room(
        index=i + 1,
        x_left=x
    )


# ---------------------------------------------------------
# BOTTOM: ROOM 9 -> ROOM 15
# ---------------------------------------------------------

for i in range(NUM_BOTTOM_ROOMS):

    x = school_start_x + i * ROOM_WIDTH

    create_bottom_room(
        index=NUM_TOP_ROOMS + i + 1,
        x_left=x
    )


# =========================================================
# CORRIDOR END WALLS
# =========================================================

corridor_left = school_start_x

corridor_right = (
    school_start_x
    + NUM_TOP_ROOMS * ROOM_WIDTH
)

vertical_wall(
    "corridor_left_end",
    corridor_left,
    -CORRIDOR_WIDTH / 2,
    CORRIDOR_WIDTH / 2
)

vertical_wall(
    "corridor_right_end",
    corridor_right,
    -CORRIDOR_WIDTH / 2,
    CORRIDOR_WIDTH / 2
)


# =========================================================
# SCHOOL OBJECTS
# =========================================================

# Bench
add_box(
    "bench_01",
    -10,
    0.42,
    1.3,
    0.20,
    0.45,
    color="0.45 0.25 0.10 1"
)

add_box(
    "bench_02",
    0,
    -0.42,
    1.3,
    0.20,
    0.45,
    color="0.45 0.25 0.10 1"
)

add_box(
    "bench_03",
    9,
    0.42,
    1.3,
    0.20,
    0.45,
    color="0.45 0.25 0.10 1"
)


# Lockers
add_box(
    "locker_01",
    -5,
    0.48,
    2.0,
    0.18,
    1.8,
    color="0.20 0.40 0.75 1",
    z=0.9
)

add_box(
    "locker_02",
    5,
    0.48,
    2.0,
    0.18,
    1.8,
    color="0.20 0.40 0.75 1",
    z=0.9
)


# Backpack / obstacle
add_box(
    "backpack_01",
    -2,
    -0.25,
    0.30,
    0.25,
    0.30,
    color="0.75 0.10 0.10 1",
    z=0.15
)

add_box(
    "backpack_02",
    7,
    0.20,
    0.30,
    0.25,
    0.30,
    color="0.10 0.20 0.75 1",
    z=0.15
)


# =========================================================
# END WORLD
# =========================================================

models.append("""
</world>

</sdf>
""")


OUTPUT.write_text(
    "".join(models),
    encoding="utf-8"
)

print("=" * 60)

print("School world generated")

print(f"Output: {OUTPUT}")

print(f"Rooms: {NUM_TOP_ROOMS + NUM_BOTTOM_ROOMS}")

print(f"Corridor width: {CORRIDOR_WIDTH} m")

print(f"Doors per room: 2")

print("=" * 60)
