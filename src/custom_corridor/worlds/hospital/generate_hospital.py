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

OUTPUT_FILE = Path(__file__).resolve().parent / "hospital_easy.sdf"


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

def generate_room_markers():
    """
    Add small visual markers near the rooms.

    These are simple blocks used only to make the room
    arrangement easier to identify in Gazebo.
    """

    models = ""

    centers = room_centers()

    # Small wall-mounted marker near each room.
    marker_width = 0.35
    marker_height = 0.25

    for i, x in enumerate(centers):

        # North marker
        models += box_model(
            name=f"north_room_marker_{i + 1:02d}",
            x=x,
            y=CORRIDOR_WIDTH / 2.0 + 0.03,
            z=0.75,
            sx=marker_width,
            sy=0.04,
            sz=marker_height,
            color="0.95 0.95 0.95 1.0",
        )

        # South marker
        models += box_model(
            name=f"south_room_marker_{i + 1:02d}",
            x=x,
            y=-CORRIDOR_WIDTH / 2.0 - 0.03,
            z=0.75,
            sx=marker_width,
            sy=0.04,
            sz=marker_height,
            color="0.95 0.95 0.95 1.0",
        )

    return models


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

def generate_walking_human():
    """Người đi từ phòng 3 đến phòng 5 phía nam rồi quay về, lặp liên tục."""

    waypoints = [
        # time (s), x, y, yaw (rad)
        (0.0,  -14.0,  0.0,  0.0),     # Trước phòng 3
        (14.0,  -2.8,  0.0,  0.0),     # Dọc hành lang, tốc độ ~0.8 m/s
        (14.4,  -2.8,  0.0, -1.5708), # Quay về cửa phòng phía nam
        (17.5,  -2.8, -2.5, -1.5708), # Qua cửa
        (19.4,  -2.8, -4.0, -1.5708), # Trong phòng 5
        (21.4,  -2.8, -4.0,  1.5708), # Dừng rồi quay ra
        (23.3,  -2.8, -2.5,  1.5708),
        (26.4,  -2.8,  0.0,  1.5708),
        (26.8,  -2.8,  0.0,  3.14159),
        (40.8, -14.0,  0.0,  3.14159), # Trở về điểm đầu
        (41.2, -14.0,  0.0,  0.0),     # Quay đầu, bắt đầu vòng tiếp
    ]

    waypoint_xml = "\n".join(
        f"""
          <waypoint>
            <time>{t:.2f}</time>
            <pose>{x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose>
          </waypoint>"""
        for t, x, y, yaw in waypoints
    )

    return f"""
    <actor name="hospital_walking_human">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          {waypoint_xml}
        </trajectory>
      </script>
    </actor>
"""

def generate_second_walking_human():
    """Room 6 south -> room 2 -> wait -> room 3 north."""

    centers = room_centers()
    room_6_x = centers[5]  #  2.8
    room_2_x = centers[1]  # -19.6
    room_3_x = centers[2]  # -14.0

    # Ghế phía nam có tâm y = -2.1; người đứng cũ cách ghế 1 m.
    south_bench_y = -CORRIDOR_WIDTH / 2.0 + 0.40
    walking_y = south_bench_y + 1.0  # -1.1, phía trong hành lang

    # Thời gian tính theo giây; quãng đi chính xấp xỉ 0.8 m/s.
    waypoints = [
    # time (s), x, y, yaw (rad)
    (0.0,  room_6_x, -4.0,      1.5708),   # Trong phòng 6 nam
    (3.6,  room_6_x, walking_y, 1.5708),   # Ra hành lang
    (3.9,  room_6_x, walking_y, 3.14159),  # Quay sang trái
    (31.9, room_2_x, walking_y, 3.14159),  # Đến phòng 2
    (41.9, room_2_x, walking_y, 3.14159),  # Đứng chờ 10 giây
    (42.3, room_2_x, walking_y, 0.0),      # Quay lại
    (49.3, room_3_x, walking_y, 0.0),      # Đến cửa phòng 3
    (49.7, room_3_x, walking_y, 1.5708),   # Quay về phía bắc
    (56.7, room_3_x, 4.0,       1.5708),   # Vào phòng 3 bắc
]

    waypoint_xml = "\n".join(
        f"""
          <waypoint>
            <time>{t:.2f}</time>
            <pose>{x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose>
          </waypoint>"""
        for t, x, y, yaw in waypoints
    )

    return f"""
    <actor name="hospital_walking_human_2">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          {waypoint_xml}
        </trajectory>
      </script>
    </actor>
"""

def generate_third_walking_human():
    """Wait by the bench at south room 2, then follow actor 2 into north room 3."""

    centers = room_centers()
    room_2_x = centers[1]
    room_3_x = centers[2]
    bench_x = room_2_x - DOOR_WIDTH / 2.0 - 0.30 - BENCH_LENGTH / 2.0
    bench_y = -CORRIDOR_WIDTH / 2.0 + 0.40
    walking_y = bench_y + 1.0

    # Actor 2 leaves room 2 at t=42.3. The follower starts beside the bench
    # after that point and reaches each turn roughly 2-3 seconds later.
    waypoints = [
        (0.0, bench_x, bench_y, 0.0),
        (42.3, bench_x, bench_y, 0.0),
        (43.6, bench_x, walking_y, 1.5708),
        (44.0, bench_x, walking_y, 0.0),
        (52.0, room_3_x, walking_y, 0.0),
        (52.4, room_3_x, walking_y, 1.5708),
        (59.4, room_3_x, 4.0, 1.5708),
    ]

    waypoint_xml = "\n".join(
        f"""<waypoint><time>{t:.2f}</time>
        <pose>{x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose></waypoint>"""
        for t, x, y, yaw in waypoints
    )

    return f"""
    <actor name="hospital_walking_human_3">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          {waypoint_xml}
        </trajectory>
      </script>
    </actor>
"""

# ============================================================
# WORLD GENERATION
# ============================================================

def generate_medium_single_walker():
    """Walker 1 takes the opposite lane through the trolley route.

    Planned avoidance: transition before entering the cart's swept region.
    This is not perception-driven social avoidance.
    """
    points = [(0, -10, 0, 3.14159), (8, -16, 0, 3.14159),
              (10, -17, -1.3, 3.14159), (22, -26, -1.3, 3.14159),
              (24, -26, -1.3, 0), (36, -17, -1.3, 0),
              (38, -16, 0, 0), (46, -10, 0, 0),
              (48, -10, 0, 3.14159)]
    waypoints = "".join(f"<waypoint><time>{time}</time><pose>{x} {y} 1 0 0 {yaw}</pose></waypoint>"
                        for time, x, y, yaw in points)
    return f"""
    <actor name="medium_walker_01">
      <skin><filename>model://human/meshes/walk.dae</filename><scale>1</scale></skin>
      <animation name="walk"><filename>model://human/meshes/walk.dae</filename><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0">{waypoints}</trajectory>
      </script>
    </actor>
"""


def generate_medium_head_on_walker():
    """Medium actor 2: walk westward on the corridor and return."""
    return """
    <actor name="medium_walker_02_head_on">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          <waypoint><time>0.0</time><pose>8.0 0.0 1.0 0 0 3.14159</pose></waypoint>
          <waypoint><time>8.0</time><pose>2.0 0.0 1.0 0 0 3.14159</pose></waypoint>
          <waypoint><time>10.0</time><pose>2.0 0.0 1.0 0 0 0</pose></waypoint>
          <waypoint><time>18.0</time><pose>8.0 0.0 1.0 0 0 0</pose></waypoint>
          <waypoint><time>20.0</time><pose>8.0 0.0 1.0 0 0 3.14159</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""


def generate_medium_room_crossing_walker():
    """Medium actor 3: cross the corridor through south/north room 5 doors."""
    room_x = room_centers()[4]
    waypoints = [
        (0.0, -4.0, 1.57080),
        (2.0, -4.0, 1.57080),
        (10.0, 2.0, 1.57080),
        (12.0, 4.0, 1.57080),
        (14.0, 4.0, -1.57080),
        (24.0, -4.0, -1.57080),
        (26.0, -4.0, 1.57080),
    ]
    waypoint_xml = "\n".join(
        f"<waypoint><time>{t:.1f}</time>"
        f"<pose>{room_x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose></waypoint>"
        for t, y, yaw in waypoints
    )
    return f"""
    <actor name="medium_walker_03_room_crossing">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          {waypoint_xml}
        </trajectory>
      </script>
    </actor>
"""


def generate_medium_same_direction_walker():
    """Medium actor 4: an eastbound pedestrian in a separate corridor segment."""
    return """
    <actor name="medium_walker_04_same_direction">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          <waypoint><time>0.0</time><pose>11.0 0.0 1.0 0 0 0</pose></waypoint>
          <waypoint><time>8.0</time><pose>17.0 0.0 1.0 0 0 0</pose></waypoint>
          <waypoint><time>10.0</time><pose>17.0 0.0 1.0 0 0 3.14159</pose></waypoint>
          <waypoint><time>18.0</time><pose>11.0 0.0 1.0 0 0 3.14159</pose></waypoint>
          <waypoint><time>20.0</time><pose>11.0 0.0 1.0 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""


def generate_medium_wait_then_cross_walker():
    """Medium actor 5: wait in south room 9, then cross to north room 9."""
    room_x = room_centers()[8]
    waypoints = [
        (0.0, -4.0, 1.57080),
        (8.0, -4.0, 1.57080),
        (12.0, -1.0, 1.57080),
        (16.0, 2.0, 1.57080),
        (18.7, 4.0, 1.57080),
        (20.7, 4.0, -1.57080),
        (31.4, -4.0, -1.57080),
        (33.4, -4.0, 1.57080),
    ]
    waypoint_xml = "\n".join(
        f"<waypoint><time>{t:.1f}</time>"
        f"<pose>{room_x:.3f} {y:.3f} 1.0 0 0 {yaw:.5f}</pose></waypoint>"
        for t, y, yaw in waypoints
    )
    return f"""
    <actor name="medium_walker_05_wait_then_cross">
      <skin>
        <filename>model://human/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>model://human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.0">
          {waypoint_xml}
        </trajectory>
      </script>
    </actor>
"""


def generate_medium_trolley_pusher():
    """New human and rigid three-shelf cart with identical timed trajectories.

    Visual scripted motion only, as with the other Medium walking actors.
    Cart geometry is offset ahead of its actor origin; the root stays level.
    """
    waypoints = [(0, -24, 0), (12, -18, 0), (16, -18, 3.14159),
                 (28, -24, 3.14159), (32, -24, 0)]
    result = ""
    for name, mesh in [("medium_walker_06_trolley_pusher", "push_trolley.dae"),
                       ("medium_trolley_06", "trolley.dae")]:
        points = "".join(f"<waypoint><time>{t}</time><pose>{x} 0.6 1 0 0 {yaw}</pose></waypoint>"
                         for t, x, yaw in waypoints)
        result += f"""
    <actor name="{name}">
      <skin><filename>model://human/meshes/{mesh}</filename><scale>1</scale></skin>
      <animation name="push"><filename>model://human/meshes/{mesh}</filename><interpolate_x>false</interpolate_x></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="push" tension="0">{points}</trajectory>
      </script>
    </actor>
"""
    return result


def generate_medium_yielding_walker():
    """Actor 7 is driven by a Medium-only reactive robot-yielding plugin."""
    return """
    <actor name="medium_walker_07_yielding">
      <pose>0 0 0 0 0 0</pose>
      <skin><filename>model://human/meshes/walk.dae</filename><scale>1</scale></skin>
      <animation name="walk"><filename>model://human/meshes/walk.dae</filename><interpolate_x>false</interpolate_x></animation>
      <script><loop>false</loop><delay_start>0</delay_start><auto_start>false</auto_start>
        <trajectory id="0" type="walk" tension="0">
          <waypoint><time>0</time><pose>0 -1.35 1 0 0 1.570796</pose></waypoint>
          <waypoint><time>1</time><pose>0 -1.35 1 0 0 1.570796</pose></waypoint>
        </trajectory>
      </script>
    </actor>
    <plugin filename="libhospital_yielding_system.so" name="custom_corridor::HospitalYieldingSystem">
      <robot_name>burger</robot_name>
    </plugin>
"""


def generate_medium_standing_people():
    """Three new Medium-only people at the right of selected entrances.

    Right means the observer faces the room from the corridor. All positions
    are in the corridor, with clearance to the doorway, benches and wall.
    """
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


def generate_world(difficulty="easy"):
    if difficulty not in ("easy", "medium"):
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

    models += generate_benches(solid_base=difficulty == "medium")

    # The markers are solid boxes centered in the doorways. Keep the original
    # Easy world unchanged, but leave every Medium doorway unobstructed.
    if difficulty == "easy":
        models += generate_room_markers()

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
    parser = argparse.ArgumentParser(description="Generate a hospital Gazebo world")
    parser.add_argument("--difficulty", choices=("easy", "medium"), default="easy")
    args = parser.parse_args()

    sdf_content = generate_world(args.difficulty)
    output_file = OUTPUT_FILE.with_name(f"hospital_{args.difficulty}.sdf")

    output_file.write_text(
        sdf_content,
        encoding="utf-8",
    )

    print()
    print("=" * 60)
    print(f"Hospital {args.difficulty.capitalize()} generated successfully")
    print("=" * 60)
    print(f"Output : {output_file}")
    print()
    print("Layout:")
    print(f"  Rooms per side : {ROOM_COUNT_PER_SIDE}")
    print(f"  Total rooms    : {ROOM_COUNT_PER_SIDE * 2}")
    print(f"  Hospital size  : {HOSPITAL_LENGTH} x {HOSPITAL_WIDTH} m")
    print(f"  Corridor width : {CORRIDOR_WIDTH} m")
    print(f"  Room width     : {ROOM_WIDTH} m")
    print(f"  Room depth     : {ROOM_DEPTH} m")
    print(f"  Wall height    : {WALL_HEIGHT} m")
    print(f"  Door width     : {DOOR_WIDTH} m")
    print()
    print("Features:")
    print("  [OK] Open-top environment")
    print("  [OK] No ceiling")
    print("  [OK] No end walls")
    print("  [OK] 20 rooms")
    print("  [OK] Door opening for every room")
    print("  [OK] Bench in front of every room")
    print("  [OK] Static collision")
    print("=" * 60)
    print()


if __name__ == "__main__":
    main()
