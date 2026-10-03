#!/usr/bin/env python3

"""
Generate an open-top hospital environment for Gazebo Sim.

Hospital Easy layout
--------------------

                NORTH SIDE
    +---------+---------+---------+---------+
    | Room 01 | Room 02 | Room 03 |  ...    |
    |         |         |         |          |
    |    []   |    []   |    []   |          |
    +----door-+----door-+----door-+----------+
    |                                            |
    |                CORRIDOR                    |
    |                                            |
    +----door-+----door-+----door-+----------+
    |    []   |    []   |    []   |          |
    | Room 11 | Room 12 | Room 13 |  ...     |
    +---------+---------+---------+---------+

                SOUTH SIDE

- 10 rooms on the north side
- 10 rooms on the south side
- One long corridor in the middle
- One doorway per room
- One bench in front of each room
- No ceiling
- No end walls
- Low walls for easy visualization in Gazebo
"""

import argparse
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

OUTPUT_DIRECTORY = Path(__file__).resolve().parent


# Overall hospital dimensions
HOSPITAL_LENGTH = 60.0
HOSPITAL_WIDTH = 17.0


# Corridor
CORRIDOR_WIDTH = 5.0


# Rooms
ROOM_COUNT_PER_SIDE = 10
ROOM_DEPTH = (HOSPITAL_WIDTH - CORRIDOR_WIDTH) / 2.0
ROOM_WIDTH = 5.6


# Walls
WALL_THICKNESS = 0.20

# Low wall so that Gazebo camera can observe the robot.
WALL_HEIGHT = 2.5


# Doors
DOOR_WIDTH = 1.2
DOOR_HEIGHT = 1.2


# Benches
BENCH_LENGTH = 1.6
BENCH_WIDTH = 0.45
BENCH_HEIGHT = 0.45


# Floor
FLOOR_THICKNESS = 0.10


# ============================================================
# SDF HELPERS
# ============================================================

def box_model(
    name,
    x,
    y,
    z,
    sx,
    sy,
    sz,
    color="0.75 0.75 0.75 1.0",
):
    """
    Create one static box model.

    x, y, z:
        Center position.

    sx, sy, sz:
        Box dimensions.
    """

    return f"""
    <model name="{name}">
      <static>true</static>

      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
            </box>
          </geometry>

          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>

      </link>
    </model>
"""


# ============================================================
# FLOOR
# ============================================================

def generate_floor():
    """
    Generate the hospital floor.
    """

    return box_model(
        name="hospital_floor",
        x=0.0,
        y=0.0,
        z=-FLOOR_THICKNESS / 2.0,
        sx=HOSPITAL_LENGTH,
        sy=HOSPITAL_WIDTH,
        sz=FLOOR_THICKNESS,
        color="0.82 0.82 0.82 1.0",
    )


# ============================================================
# ROOM GEOMETRY
# ============================================================

def room_centers():
    """
    Calculate the X center of every room.

    The 10 rooms are distributed along the hospital length.
    """

    total_rooms_length = ROOM_COUNT_PER_SIDE * ROOM_WIDTH

    start_x = -total_rooms_length / 2.0 + ROOM_WIDTH / 2.0

    centers = []

    for i in range(ROOM_COUNT_PER_SIDE):
        x = start_x + i * ROOM_WIDTH
        centers.append(x)

    return centers


# ============================================================
# OUTER WALLS
# ============================================================

def generate_outer_walls():
    """
    Generate the outer walls of both room rows.

    There are intentionally NO end walls.
    This makes the hospital easier to observe from Gazebo.
    """

    models = ""

    north_outer_y = HOSPITAL_WIDTH / 2.0
    south_outer_y = -HOSPITAL_WIDTH / 2.0

    # --------------------------------------------------------
    # North outer wall
    # --------------------------------------------------------

    models += box_model(
        name="north_outer_wall",
        x=0.0,
        y=north_outer_y,
        z=WALL_HEIGHT / 2.0,
        sx=HOSPITAL_LENGTH,
        sy=WALL_THICKNESS,
        sz=WALL_HEIGHT,
        color="0.78 0.78 0.78 1.0",
    )

    # --------------------------------------------------------
    # South outer wall
    # --------------------------------------------------------

    models += box_model(
        name="south_outer_wall",
        x=0.0,
        y=south_outer_y,
        z=WALL_HEIGHT / 2.0,
        sx=HOSPITAL_LENGTH,
        sy=WALL_THICKNESS,
        sz=WALL_HEIGHT,
        color="0.78 0.78 0.78 1.0",
    )

    return models


# ============================================================
# ROOM PARTITION WALLS
# ============================================================

def generate_room_partition_walls():
    """
    Generate vertical partition walls between adjacent rooms.

    These walls separate the 10 rooms on each side.
    """

    models = ""

    centers = room_centers()

    # --------------------------------------------------------
    # North rooms
    # --------------------------------------------------------

    north_inner_y = CORRIDOR_WIDTH / 2.0
    north_outer_y = HOSPITAL_WIDTH / 2.0

    north_room_center_y = (
        north_inner_y + north_outer_y
    ) / 2.0

    north_room_depth = (
        north_outer_y - north_inner_y
    )

    for i in range(ROOM_COUNT_PER_SIDE - 1):

        x = (
            centers[i] + centers[i + 1]
        ) / 2.0

        models += box_model(
            name=f"north_partition_{i}",
            x=x,
            y=north_room_center_y,
            z=WALL_HEIGHT / 2.0,
            sx=WALL_THICKNESS,
            sy=north_room_depth,
            sz=WALL_HEIGHT,
            color="0.78 0.78 0.78 1.0",
        )

    # --------------------------------------------------------
    # South rooms
    # --------------------------------------------------------

    south_inner_y = -CORRIDOR_WIDTH / 2.0
    south_outer_y = -HOSPITAL_WIDTH / 2.0

    south_room_center_y = (
        south_inner_y + south_outer_y
    ) / 2.0

    south_room_depth = (
        south_inner_y - south_outer_y
    )

    for i in range(ROOM_COUNT_PER_SIDE - 1):

        x = (
            centers[i] + centers[i + 1]
        ) / 2.0

        models += box_model(
            name=f"south_partition_{i}",
            x=x,
            y=south_room_center_y,
            z=WALL_HEIGHT / 2.0,
            sx=WALL_THICKNESS,
            sy=south_room_depth,
            sz=WALL_HEIGHT,
            color="0.78 0.78 0.78 1.0",
        )

    return models


# ============================================================
# CORRIDOR-SIDE WALLS WITH DOOR OPENINGS
# ============================================================

def generate_corridor_walls():
    """
    Generate the walls separating the rooms from the corridor.

    Each room has a doorway.

    Instead of creating one continuous wall, the wall is split
    into:

        left wall segment
        door opening
        right wall segment

    This creates a real passage from the corridor into each room.
    """

    models = ""

    centers = room_centers()

    corridor_half = CORRIDOR_WIDTH / 2.0

    # --------------------------------------------------------
    # North corridor-side wall
    # --------------------------------------------------------

    north_wall_y = corridor_half

    for i, room_x in enumerate(centers):

        room_left = room_x - ROOM_WIDTH / 2.0
        room_right = room_x + ROOM_WIDTH / 2.0

        door_left = room_x - DOOR_WIDTH / 2.0
        door_right = room_x + DOOR_WIDTH / 2.0

        # Left wall segment
        left_length = door_left - room_left

        if left_length > 0.01:

            left_center_x = (
                room_left + door_left
            ) / 2.0

            models += box_model(
                name=f"north_room_{i + 1:02d}_wall_left",
                x=left_center_x,
                y=north_wall_y,
                z=WALL_HEIGHT / 2.0,
                sx=left_length,
                sy=WALL_THICKNESS,
                sz=WALL_HEIGHT,
                color="0.78 0.78 0.78 1.0",
            )

        # Right wall segment
        right_length = room_right - door_right

        if right_length > 0.01:

            right_center_x = (
                door_right + room_right
            ) / 2.0

            models += box_model(
                name=f"north_room_{i + 1:02d}_wall_right",
                x=right_center_x,
                y=north_wall_y,
                z=WALL_HEIGHT / 2.0,
                sx=right_length,
                sy=WALL_THICKNESS,
                sz=WALL_HEIGHT,
                color="0.78 0.78 0.78 1.0",
            )

    # --------------------------------------------------------
    # South corridor-side wall
    # --------------------------------------------------------

    south_wall_y = -corridor_half

    for i, room_x in enumerate(centers):

        room_left = room_x - ROOM_WIDTH / 2.0
        room_right = room_x + ROOM_WIDTH / 2.0

        door_left = room_x - DOOR_WIDTH / 2.0
        door_right = room_x + DOOR_WIDTH / 2.0

        # Left wall segment
        left_length = door_left - room_left

        if left_length > 0.01:

            left_center_x = (
                room_left + door_left
            ) / 2.0

            models += box_model(
                name=f"south_room_{i + 1:02d}_wall_left",
                x=left_center_x,
                y=south_wall_y,
                z=WALL_HEIGHT / 2.0,
                sx=left_length,
                sy=WALL_THICKNESS,
                sz=WALL_HEIGHT,
                color="0.78 0.78 0.78 1.0",
            )

        # Right wall segment
        right_length = room_right - door_right

        if right_length > 0.01:

            right_center_x = (
                door_right + room_right
            ) / 2.0

            models += box_model(
                name=f"south_room_{i + 1:02d}_wall_right",
                x=right_center_x,
                y=south_wall_y,
                z=WALL_HEIGHT / 2.0,
                sx=right_length,
                sy=WALL_THICKNESS,
                sz=WALL_HEIGHT,
                color="0.78 0.78 0.78 1.0",
            )

    return models


# ============================================================
# DOOR FRAME / VISUAL
# ============================================================

def generate_doors():
    """
    Generate simple door frames.

    The center of the doorway remains open so the robot can
    actually pass through it.

    Door frames are visual/static objects only.
    """

    models = ""

    centers = room_centers()

    door_post_width = 0.10
    door_top_height = 0.10

    # Door frame vertical height.
    # Keep it equal to the wall height.
    frame_height = WALL_HEIGHT

    # --------------------------------------------------------
    # North doors
    # --------------------------------------------------------

    north_y = CORRIDOR_WIDTH / 2.0 - 0.01

    for i, x in enumerate(centers):

        door_left_x = x - DOOR_WIDTH / 2.0
        door_right_x = x + DOOR_WIDTH / 2.0

        # Left door post
        models += box_model(
            name=f"north_door_{i + 1:02d}_left",
            x=door_left_x,
            y=north_y,
            z=frame_height / 2.0,
            sx=door_post_width,
            sy=0.12,
            sz=frame_height,
            color="0.55 0.38 0.22 1.0",
        )

        # Right door post
        models += box_model(
            name=f"north_door_{i + 1:02d}_right",
            x=door_right_x,
            y=north_y,
            z=frame_height / 2.0,
            sx=door_post_width,
            sy=0.12,
            sz=frame_height,
            color="0.55 0.38 0.22 1.0",
        )

        # Top frame
        models += box_model(
            name=f"north_door_{i + 1:02d}_top",
            x=x,
            y=north_y,
            z=frame_height - door_top_height / 2.0,
            sx=DOOR_WIDTH,
            sy=0.12,
            sz=door_top_height,
            color="0.55 0.38 0.22 1.0",
        )

    # --------------------------------------------------------
    # South doors
    # --------------------------------------------------------

    south_y = -CORRIDOR_WIDTH / 2.0 + 0.01

    for i, x in enumerate(centers):

        door_left_x = x - DOOR_WIDTH / 2.0
        door_right_x = x + DOOR_WIDTH / 2.0

        # Left door post
        models += box_model(
            name=f"south_door_{i + 1:02d}_left",
            x=door_left_x,
            y=south_y,
            z=frame_height / 2.0,
            sx=door_post_width,
            sy=0.12,
            sz=frame_height,
            color="0.55 0.38 0.22 1.0",
        )

        # Right door post
        models += box_model(
            name=f"south_door_{i + 1:02d}_right",
            x=door_right_x,
            y=south_y,
            z=frame_height / 2.0,
            sx=door_post_width,
            sy=0.12,
            sz=frame_height,
            color="0.55 0.38 0.22 1.0",
        )

        # Top frame
        models += box_model(
            name=f"south_door_{i + 1:02d}_top",
            x=x,
            y=south_y,
            z=frame_height - door_top_height / 2.0,
            sx=DOOR_WIDTH,
            sy=0.12,
            sz=door_top_height,
            color="0.55 0.38 0.22 1.0",
        )

    return models


# ============================================================
# BENCHES
# ============================================================

def generate_benches(solid_base=False):
    """
    Generate waiting benches beside each room entrance.

    Layout:

        [ BENCH ]   [ DOOR ]   [ BENCH ]

    The doorway is always kept clear so that:
    - people can enter/exit the room
    - the robot can pass through the doorway
    - benches do not block the navigation path

    If there is not enough horizontal space for two benches,
    only one bench is placed.
    """

    models = ""

    centers = room_centers()

    # --------------------------------------------------------
    # Bench dimensions
    # --------------------------------------------------------

    seat_thickness = 0.18
    leg_height = BENCH_HEIGHT - seat_thickness

    # --------------------------------------------------------
    # Position relative to corridor
    # --------------------------------------------------------
    #
    # Corridor:
    #
    #       NORTH ROOM
    #          ↓
    #     ────────────
    #        🪑 🚪 🪑
    #     ────────────
    #          ↑
    #       SOUTH ROOM
    #
    # Put benches close to the corridor-side walls.
    #

    north_bench_y = CORRIDOR_WIDTH / 2.0 - 0.40
    south_bench_y = -CORRIDOR_WIDTH / 2.0 + 0.40

    # --------------------------------------------------------
    # Distance from door center
    # --------------------------------------------------------
    #
    # Door width = 1.2 m
    #
    # Put the bench center outside the door area.
    #

    bench_gap = 0.30

    bench_x_offset = (
        DOOR_WIDTH / 2.0
        + bench_gap
        + BENCH_LENGTH / 2.0
    )

    # --------------------------------------------------------
    # Helper: create one bench
    # --------------------------------------------------------

    def create_bench(name_prefix, x, y):

        result = ""

        # Seat
        result += box_model(
            name=f"{name_prefix}_seat",
            x=x,
            y=y,
            z=BENCH_HEIGHT,
            sx=BENCH_LENGTH,
            sy=BENCH_WIDTH,
            sz=seat_thickness,
            color="0.10 0.25 0.55 1.0",
        )

        if solid_base:
            # From z=0 to the underside of the seat (0.36 m). The robot's
            # horizontal LiDAR at z~=0.17 m now sees a continuous obstacle.
            base_height = BENCH_HEIGHT - seat_thickness / 2.0
            result += box_model(
                name=f"{name_prefix}_base",
                x=x,
                y=y,
                z=base_height / 2.0,
                sx=BENCH_LENGTH,
                sy=BENCH_WIDTH,
                sz=base_height,
                color="0.18 0.23 0.33 1.0",
            )
        else:
            # Keep the original two-legged bench in Hospital Easy.
            result += box_model(
                name=f"{name_prefix}_leg_left",
                x=x - BENCH_LENGTH * 0.35,
                y=y,
                z=leg_height / 2.0,
                sx=0.10,
                sy=0.10,
                sz=leg_height,
                color="0.35 0.35 0.35 1.0",
            )

            result += box_model(
                name=f"{name_prefix}_leg_right",
                x=x + BENCH_LENGTH * 0.35,
                y=y,
                z=leg_height / 2.0,
                sx=0.10,
                sy=0.10,
                sz=leg_height,
                color="0.35 0.35 0.35 1.0",
            )

        return result

    # ========================================================
    # NORTH SIDE ROOMS
    # ========================================================

    for i, room_x in enumerate(centers):

        room_left = room_x - ROOM_WIDTH / 2.0
        room_right = room_x + ROOM_WIDTH / 2.0

        door_left = room_x - DOOR_WIDTH / 2.0
        door_right = room_x + DOOR_WIDTH / 2.0

        # ----------------------------------------------------
        # Candidate bench on LEFT side of door
        # ----------------------------------------------------

        left_bench_x = (
            door_left
            - bench_gap
            - BENCH_LENGTH / 2.0
        )

        left_bench_left = (
            left_bench_x - BENCH_LENGTH / 2.0
        )

        # ----------------------------------------------------
        # Candidate bench on RIGHT side of door
        # ----------------------------------------------------

        right_bench_x = (
            door_right
            + bench_gap
            + BENCH_LENGTH / 2.0
        )

        right_bench_right = (
            right_bench_x + BENCH_LENGTH / 2.0
        )

        # ----------------------------------------------------
        # Check whether both benches fit
        # ----------------------------------------------------

        left_fits = (
            left_bench_left >= room_left + 0.15
        )

        right_fits = (
            right_bench_right <= room_right - 0.15
        )

        # ----------------------------------------------------
        # Two benches
        # ----------------------------------------------------

        if left_fits and right_fits:

            models += create_bench(
                f"north_room_{i + 1:02d}_bench_left",
                left_bench_x,
                north_bench_y,
            )

            models += create_bench(
                f"north_room_{i + 1:02d}_bench_right",
                right_bench_x,
                north_bench_y,
            )

        # ----------------------------------------------------
        # Only left bench
        # ----------------------------------------------------

        elif left_fits:

            models += create_bench(
                f"north_room_{i + 1:02d}_bench_left",
                left_bench_x,
                north_bench_y,
            )

        # ----------------------------------------------------
        # Only right bench
        # ----------------------------------------------------

        elif right_fits:

            models += create_bench(
                f"north_room_{i + 1:02d}_bench_right",
                right_bench_x,
                north_bench_y,
            )

    # ========================================================
    # SOUTH SIDE ROOMS
    # ========================================================

    for i, room_x in enumerate(centers):

        room_left = room_x - ROOM_WIDTH / 2.0
        room_right = room_x + ROOM_WIDTH / 2.0

        door_left = room_x - DOOR_WIDTH / 2.0
        door_right = room_x + DOOR_WIDTH / 2.0

        # ----------------------------------------------------
        # Candidate bench on LEFT side of door
        # ----------------------------------------------------

        left_bench_x = (
            door_left
            - bench_gap
            - BENCH_LENGTH / 2.0
        )

        left_bench_left = (
            left_bench_x - BENCH_LENGTH / 2.0
        )

        # ----------------------------------------------------
        # Candidate bench on RIGHT side of door
        # ----------------------------------------------------

        right_bench_x = (
            door_right
            + bench_gap
            + BENCH_LENGTH / 2.0
        )

        right_bench_right = (
            right_bench_x + BENCH_LENGTH / 2.0
        )

        # ----------------------------------------------------
        # Check whether both benches fit
        # ----------------------------------------------------

        left_fits = (
            left_bench_left >= room_left + 0.15
        )

        right_fits = (
            right_bench_right <= room_right - 0.15
        )

        # ----------------------------------------------------
        # Two benches
        # ----------------------------------------------------

        if left_fits and right_fits:

            models += create_bench(
                f"south_room_{i + 1:02d}_bench_left",
                left_bench_x,
                south_bench_y,
            )

            models += create_bench(
                f"south_room_{i + 1:02d}_bench_right",
                right_bench_x,
                south_bench_y,
            )

        # ----------------------------------------------------
        # Only left bench
        # ----------------------------------------------------

        elif left_fits:

            models += create_bench(
                f"south_room_{i + 1:02d}_bench_left",
                left_bench_x,
                south_bench_y,
            )

        # ----------------------------------------------------
        # Only right bench
        # ----------------------------------------------------

        elif right_fits:

            models += create_bench(
                f"south_room_{i + 1:02d}_bench_right",
                right_bench_x,
                south_bench_y,
            )

    return models


# ============================================================
# ROOM LABELS
# ============================================================

def static_human_model(name, x, y, z, yaw, vertical_scale=1.0, collision_height=1.70, has_collision=True,):
        """Tạo người đứng yên; collision có chiều cao phù hợp từng tư thế."""

        collision_xml = ""
        if has_collision:
            # Model đặt ở z; cylinder phải bắt đầu từ mặt sàn z=0.
            collision_center_z = collision_height / 2.0 - z
            collision_xml = f"""
                    <collision name="collision">
                        <pose>0 0 {collision_center_z:.3f} 0 0 0</pose>
                        <geometry>
                            <cylinder>
                                <radius>0.28</radius>
                                <length>{collision_height:.3f}</length>
                            </cylinder>
                        </geometry>
                    </collision>"""

        return f"""
            <model name="{name}">
                <static>true</static>
                <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}</pose>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <mesh>
                                <uri>model://human/meshes/walk.dae</uri>
                                <scale>1.0 1.0 {vertical_scale:.3f}</scale>
                            </mesh>
                        </geometry>
                    </visual>
                    {collision_xml}
                </link>
            </model>
    """


def generate_static_humans():
        """Place two seated people and one standing person at room 5."""

        room_5_bench_x = room_centers()[4] - DOOR_WIDTH / 2.0 - 0.30 - BENCH_LENGTH / 2.0
        room_5_bench_y = CORRIDOR_WIDTH / 2.0 - 0.40

        models = ""
        models += static_human_model(
                "hospital_room_05_seated_1",
                room_5_bench_x - 0.40,
                room_5_bench_y,
                0.65,
                -1.5708,
                vertical_scale=0.65,
        )
        models += static_human_model(
                "hospital_room_05_seated_2",
                room_5_bench_x + 0.40,
                room_5_bench_y,
                0.65,
                -1.5708,
                vertical_scale=0.65,
        )
        models += static_human_model(
                "hospital_room_05_standing",
                room_5_bench_x,
                room_5_bench_y - 1.0,
                1.0,
                1.5708,
                collision_height=1.70,
        )

        return models

def human_proxy_model(name, x, y, z=0.85, yaw=0.0, radius=0.28, length=1.70):
    """Physical collision proxy for LiDAR and physics, synchronized with RVO2."""
    return f"""
    <model name="{name}_proxy">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{length:.2f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="proxy_visual">
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{length:.2f}</length>
            </cylinder>
          </geometry>
          <transparency>1.0</transparency>
        </visual>
      </link>
    </model>
"""


def rvo2_actor_model(name, mesh="walk.dae", scale=1.0, anim_name="walk"):
    """Visual-only animated skeletal actor zero-anchored for Gazebo Sim 8."""
    return f"""
    <actor name="{name}">
      <pose>0 0 0 0 0 0</pose>
      <skin>
        <filename>model://human/meshes/{mesh}</filename>
        <scale>{scale:.2f}</scale>
      </skin>
      <animation name="{anim_name}">
        <filename>model://human/meshes/{mesh}</filename>
        <interpolate_x>false</interpolate_x>
      </animation>
      <script>
        <loop>false</loop>
        <delay_start>0</delay_start>
        <auto_start>false</auto_start>
        <trajectory id="0" type="{anim_name}" tension="0">
          <waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
          <waypoint><time>1</time><pose>0 0 0 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""


def generate_walking_human():
    """Easy Actor 1: Oncoming corridor walker."""
    return (
        rvo2_actor_model("hospital_walking_human", mesh="walk_orange.dae", scale=1.00)
        + human_proxy_model("hospital_walking_human", -14.0, 0.0, yaw=0.0)
    )


def generate_second_walking_human():
    """Easy Actor 2: South room 6 walker."""
    room_6_x = room_centers()[5]  # 2.8
    return (
        rvo2_actor_model("hospital_walking_human_2", mesh="walk_blue.dae", scale=0.95)
        + human_proxy_model("hospital_walking_human_2", room_6_x, -4.0, yaw=1.5708)
    )


def generate_third_walking_human():
    """Easy Actor 3: Waiting follower at south room 2 bench."""
    bench_x = room_centers()[1] - DOOR_WIDTH / 2.0 - 0.30 - BENCH_LENGTH / 2.0
    bench_y = -CORRIDOR_WIDTH / 2.0 + 0.40
    return (
        rvo2_actor_model("hospital_walking_human_3", mesh="walk_red.dae", scale=1.02)
        + human_proxy_model("hospital_walking_human_3", bench_x, bench_y, yaw=0.0)
    )


# ============================================================
# MEDIUM & HARD WALKERS
# ============================================================

def generate_medium_single_walker():
    """Medium Actor 1: Westbound pedestrian."""
    return (
        rvo2_actor_model("medium_walker_01", mesh="walk_orange.dae", scale=1.00)
        + human_proxy_model("medium_walker_01", -10.0, 0.0, yaw=3.14159)
    )


def generate_medium_head_on_walker():
    """Medium Actor 2: Westward corridor walker."""
    return (
        rvo2_actor_model("medium_walker_02_head_on", mesh="walk_red.dae", scale=0.95)
        + human_proxy_model("medium_walker_02_head_on", 8.0, 0.0, yaw=3.14159)
    )


def generate_medium_room_crossing_walker():
    """Medium Actor 3: Cross corridor through south/north room 5 doors."""
    room_x = room_centers()[4]
    return (
        rvo2_actor_model("medium_walker_03_room_crossing", mesh="walk_blue.dae", scale=1.06)
        + human_proxy_model("medium_walker_03_room_crossing", room_x, -4.0, yaw=1.5708)
    )


def generate_medium_same_direction_walker():
    """Medium Actor 4: Eastbound corridor segment pedestrian."""
    return (
        rvo2_actor_model("medium_walker_04_same_direction", mesh="walk.dae", scale=1.00)
        + human_proxy_model("medium_walker_04_same_direction", 11.0, 0.0, yaw=0.0)
    )


def generate_medium_wait_then_cross_walker():
    """Medium Actor 5: Cross room 9 corridor doors."""
    room_x = room_centers()[8]
    return (
        rvo2_actor_model("medium_walker_05_wait_then_cross", mesh="walk_orange.dae", scale=0.98)
        + human_proxy_model("medium_walker_05_wait_then_cross", room_x, -4.0, yaw=1.5708)
    )


def generate_medium_standing_people():
    """Three Medium-only static people at the right of selected entrances."""
    centers = room_centers()
    placements = [
        ("medium_standing_room_04_south", centers[3] - 1.0, -1.0, 0.0),
        ("medium_standing_room_07_north", centers[6] + 1.0, 1.0, 3.14159),
        ("medium_standing_room_08_south", centers[7] - 1.0, -1.0, 0.0),
    ]
    models = ""
    for name, x, y, yaw in placements:
        models += f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose>
      <link name="body">
        <visual name="standing_person">
          <geometry><mesh><uri>model://human/meshes/stand.dae</uri></mesh></geometry>
        </visual>
        <collision name="body_collision">
          <pose>0 0 -0.15 0 0 0</pose>
          <geometry><cylinder><radius>0.28</radius><length>1.70</length></cylinder></geometry>
        </collision>
      </link>
    </model>
"""
    return models


def generate_medium_trolley_pusher():
    """Medium Actors 6 & 7: Pusher and three-shelf trolley."""
    pusher = (
        rvo2_actor_model("medium_walker_06_trolley_pusher", mesh="push_trolley.dae", scale=1.00, anim_name="push")
        + human_proxy_model("medium_walker_06_trolley_pusher", -24.0, 0.6, yaw=0.0)
    )
    trolley = (
        rvo2_actor_model("medium_trolley_06", mesh="trolley.dae", scale=1.00, anim_name="push")
        + human_proxy_model("medium_trolley_06", -23.3, 0.6, yaw=0.0, radius=0.35)
    )
    return pusher + trolley


def generate_medium_yielding_walker():
    """Medium Actor 8: Social yielding pedestrian driven by RVO2."""
    return (
        rvo2_actor_model("medium_walker_07_yielding", mesh="walk_red.dae", scale=1.02)
        + human_proxy_model("medium_walker_07_yielding", 0.0, -1.35, yaw=1.5708)
    )


def generate_hard_door_exit_walker():
    """Hard Actor 9 (H8): Exit north room 5, walk west."""
    room_x = room_centers()[4]
    return (
        rvo2_actor_model("hard_walker_08_door_exit", mesh="walk_blue.dae", scale=1.00)
        + human_proxy_model("hard_walker_08_door_exit", room_x, 4.0, yaw=-1.5708)
    )


def generate_hard_door_entry_walker():
    """Hard Actor 10 (H9): Walk east, enter south room 5."""
    room_x = room_centers()[4]
    return (
        rvo2_actor_model("hard_walker_09_door_entry", mesh="walk_orange.dae", scale=0.96)
        + human_proxy_model("hard_walker_09_door_entry", room_x - 4.2, -1.35, yaw=0.0)
    )


def generate_hard_multi_crossing_walkers():
    """Hard Actors 11 & 12 (H10/H11): Cross through adjacent room doors."""
    centers = room_centers()
    h10 = (
        rvo2_actor_model("hard_walker_10_cross_south_to_north", mesh="walk_red.dae", scale=1.02)
        + human_proxy_model("hard_walker_10_cross_south_to_north", centers[5], -4.0, yaw=1.5708)
    )
    h11 = (
        rvo2_actor_model("hard_walker_11_cross_north_to_south", mesh="walk.dae", scale=0.98)
        + human_proxy_model("hard_walker_11_cross_north_to_south", centers[6], 4.0, yaw=-1.5708)
    )
    return h10 + h11


def generate_hard_group_walkers():
    """Hard Actors 13 & 14 (H12/H13): Walk side by side."""
    h12 = (
        rvo2_actor_model("hard_walker_12_group", mesh="walk_blue.dae", scale=1.04)
        + human_proxy_model("hard_walker_12_group", 11.5, -0.4, yaw=0.0)
    )
    h13 = (
        rvo2_actor_model("hard_walker_13_group", mesh="walk_red.dae", scale=0.95)
        + human_proxy_model("hard_walker_13_group", 11.5, 0.4, yaw=0.0)
    )
    return h12 + h13


def generate_hard_second_trolley_pusher():
    """Hard Actors 15 & 16 (H14 & cart): Pusher and delivery cart."""
    pusher = (
        rvo2_actor_model("hard_walker_14_trolley_pusher", mesh="push_trolley.dae", scale=1.00, anim_name="push")
        + human_proxy_model("hard_walker_14_trolley_pusher", 22.0, -1.2, yaw=3.14159)
    )
    trolley = (
        rvo2_actor_model("hard_trolley_14", mesh="trolley.dae", scale=1.00, anim_name="push")
        + human_proxy_model("hard_trolley_14", 21.3, -1.2, yaw=3.14159, radius=0.35)
    )
    return pusher + trolley


def generate_world(difficulty="easy"):
    if difficulty not in ("easy", "medium", "hard"):
        raise ValueError(f"Unsupported hospital difficulty: {difficulty}")

    models = ""

    # --------------------------------------------------------
    # Static hospital environment
    # --------------------------------------------------------

    models += generate_floor()

    models += generate_outer_walls()

    models += generate_room_partition_walls()

    models += generate_corridor_walls()

    models += generate_doors()

    models += generate_benches(solid_base=difficulty in ("medium", "hard"))


    if difficulty == "easy":
        models += generate_static_humans()
        models += generate_walking_human()
        models += generate_second_walking_human()
        models += generate_third_walking_human()
    else:
        models += generate_medium_single_walker()
        models += generate_medium_head_on_walker()
        models += generate_medium_room_crossing_walker()
        models += generate_medium_same_direction_walker()
        models += generate_medium_wait_then_cross_walker()
        models += generate_medium_standing_people()
        models += generate_medium_trolley_pusher()
        models += generate_medium_yielding_walker()
        if difficulty == "hard":
            models += generate_hard_door_exit_walker()
            models += generate_hard_door_entry_walker()
            models += generate_hard_multi_crossing_walkers()
            models += generate_hard_group_walkers()
            models += generate_hard_second_trolley_pusher()
    human_count = {"easy": 3, "medium": 8, "hard": 16}[difficulty]
    return f"""<?xml version="1.0" ?>

<sdf version="1.9">

  <world name="hospital_{difficulty}">

    <!-- ================================================== -->
    <!-- WORLD SETTINGS -->
    <!-- ================================================== -->

    <gravity>0 0 -9.81</gravity>

        <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
        <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
        <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>

    <!-- ============================================================== -->
    <!-- RVO2 Social Navigation System Plugin                           -->
    <!-- ============================================================== -->
    <plugin
      filename="librvo2_human_system.so"
      name="custom_corridor::RVO2HumanSystem">
      <human_count>{human_count}</human_count>
      <robot_name>burger</robot_name>
      <robot_radius>0.28</robot_radius>
      <human_radius>0.28</human_radius>
      <corridor_mode>false</corridor_mode>
      <politeness_balance_point>0.60</politeness_balance_point>
      <randomize_politeness>true</randomize_politeness>
      <default_scenario_file>hospital_{difficulty}</default_scenario_file>
    </plugin>

    <physics name="default_physics" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>


    <!-- ================================================== -->
    <!-- LIGHTING -->
    <!-- ================================================== -->

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.75 0.75 1</background>
      <shadows>false</shadows>
    </scene>

    <light name="hospital_sun"
           type="directional">

      <pose>0 0 20 0 0 0</pose>

      <direction>-0.3 0.2 -1.0</direction>

      <diffuse>1 1 1 1</diffuse>

      <specular>0.3 0.3 0.3 1</specular>

      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>

      <cast_shadows>false</cast_shadows>

    </light>


    <!-- ================================================== -->
    <!-- HOSPITAL MODELS -->
    <!-- ================================================== -->

{models}

  </world>

</sdf>
"""


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generate separate Easy, Medium and Hard hospital worlds."
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--difficulty", choices=("easy", "medium", "hard"),
        help="World to generate (default: easy).",
    )
    selection.add_argument(
        "--all", action="store_true", help="Generate all three separate SDF files."
    )
    args = parser.parse_args()
    difficulties = ("easy", "medium", "hard") if args.all else (args.difficulty or "easy",)

    import xml.etree.ElementTree as ET

    for difficulty in difficulties:
        sdf_content = generate_world(difficulty)
        world = ET.fromstring(sdf_content).find("world")
        if world is None or world.get("name") != f"hospital_{difficulty}":
            raise RuntimeError(f"Generated world does not match difficulty: {difficulty}")
        actors = world.findall("actor")
        names = [node.get("name") for node in world if node.get("name")]
        if len(names) != len(set(names)):
            raise RuntimeError(f"Duplicate world entity names in {difficulty}")
        hard_actors = [actor for actor in actors if actor.get("name", "").startswith("hard_")]
        if difficulty != "hard" and hard_actors:
            raise RuntimeError(f"Hard actors leaked into {difficulty}")

        output_file = OUTPUT_DIRECTORY / f"hospital_{difficulty}.sdf"
        output_file.write_text(sdf_content, encoding="utf-8")
        print(f"[OK] Difficulty : {difficulty}")
        print(f"     World      : {world.get('name')}")
        print(f"     Output     : {output_file}")
        print(f"     Actor tags : {len(actors)} (includes carts)")
        print(f"     Launch     : ros2 launch custom_corridor hospital_arena.launch.py difficulty:={difficulty}")


if __name__ == "__main__":
    main()
