"""Automated scientific plotting for single-trial trajectory and metrics."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from evaluation.spatial_alignment import get_alcove_bounds


def generate_trial_plots(
    output_dir: Path,
    robot_trajectory: Sequence[Tuple[float, float, float]],
    obstacle_trajectory: Sequence[Tuple[float, float, float]],
    proximity_samples: Sequence[Tuple[float, float, float]],
    cmd_samples: Sequence[Tuple[float, float, float]],
    encounters: Sequence[Dict[str, Any]],
    goal_xy: Optional[Tuple[float, float]] = None,
    spawn_x: float = -13.0,
    corridor_width: float = 0.90,
) -> Dict[str, str]:
    """Generate scientific visual figures: 2D Trajectory, Clearance vs. Time, and Velocity Profiles."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return {}

    output_dir.mkdir(parents=True, exist_ok=True)
    generated_plots = {}

    half_w = corridor_width / 2.0
    alcove_x_min, alcove_x_max, alcove_y_min, alcove_y_max = get_alcove_bounds(frame="odom", spawn_x=spawn_x)

    # -------------------------------------------------------------
    # 1. 2D Trajectory Plot (X-Y Plane with Corridor & Alcove)
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 4), dpi=150)

    # Draw lower wall (left wall in Gazebo, y = -half_w)
    x_span = [min(r[1] for r in robot_trajectory) - 1.0 if robot_trajectory else 0.0,
              max(r[1] for r in robot_trajectory) + 2.0 if robot_trajectory else 20.0]
    if goal_xy:
        x_span[1] = max(x_span[1], goal_xy[0] + 1.0)

    ax.plot([x_span[0], x_span[1]], [-half_w, -half_w], "k-", linewidth=3, label="Corridor Walls")

    # Draw upper wall with Alcove cutout
    ax.plot([x_span[0], alcove_x_min], [half_w, half_w], "k-", linewidth=3)
    ax.plot([alcove_x_min, alcove_x_min], [half_w, alcove_y_max], "k-", linewidth=2.5)
    ax.plot([alcove_x_min, alcove_x_max], [alcove_y_max, alcove_y_max], "k-", linewidth=2.5)
    ax.plot([alcove_x_max, alcove_x_max], [alcove_y_max, half_w], "k-", linewidth=2.5)
    ax.plot([alcove_x_max, x_span[1]], [half_w, half_w], "k-", linewidth=3)

    # Fill alcove zone lightly
    ax.fill_between([alcove_x_min, alcove_x_max], half_w, alcove_y_max, color="limegreen", alpha=0.15, label="Alcove Recess")

    # Draw Robot trajectory
    rx = [r[1] for r in robot_trajectory]
    ry = [r[2] for r in robot_trajectory]
    ax.plot(rx, ry, color="royalblue", linewidth=2.0, label="Robot Trajectory")

    # Mark Start & End
    if rx:
        ax.scatter([rx[0]], [ry[0]], color="blue", marker="o", s=70, zorder=5, label="Robot Start")
        ax.scatter([rx[-1]], [ry[-1]], color="navy", marker="X", s=90, zorder=5, label="Robot End")

    # Draw Goal
    if goal_xy:
        ax.scatter([goal_xy[0]], [goal_xy[1]], color="red", marker="*", s=140, zorder=5, label="Goal Target")

    # Draw Obstacle trajectory
    ox = [o[1] for o in obstacle_trajectory]
    oy = [o[2] for o in obstacle_trajectory]
    if ox:
        ax.plot(ox, oy, color="darkorange", linestyle="--", linewidth=1.8, alpha=0.75, label="Obstacle Range")

    # Mark Closest Approach Point
    if proximity_samples:
        min_prox = min(proximity_samples, key=lambda r: r[2])
        t_min = min_prox[0]
        # find robot pose at t_min
        r_at_min = next(((x, y) for t, x, y in robot_trajectory if abs(t - t_min) < 0.1), None)
        if r_at_min:
            ax.scatter([r_at_min[0]], [r_at_min[1]], color="crimson", marker="D", s=80, zorder=6,
                       label=f"Min Clearance: {min_prox[2]:.2f}m")

    ax.set_title("2D Trajectory & Corridor Yielding Geometry", fontsize=12, fontweight="bold")
    ax.set_xlabel("X [m] (Corridor Length)", fontsize=10)
    ax.set_ylabel("Y [m] (Lateral)", fontsize=10)
    ax.set_ylim(-half_w - 0.3, alcove_y_max + 0.3)
    ax.set_xlim(x_span[0] - 0.5, x_span[1] + 0.5)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), borderaxespad=0.0, fontsize=8)
    fig.tight_layout()

    traj_path = output_dir / "trajectory_2d.png"
    fig.savefig(traj_path, bbox_inches="tight")
    plt.close(fig)
    generated_plots["trajectory_2d"] = str(traj_path)

    # -------------------------------------------------------------
    # 2. Clearance vs. Simulation Time Plot
    # -------------------------------------------------------------
    if proximity_samples:
        fig, ax = plt.subplots(figsize=(10, 4), dpi=150)
        times = [r[0] for r in proximity_samples]
        clearances = [r[2] for r in proximity_samples]
        centers = [r[1] for r in proximity_samples]

        ax.plot(times, clearances, color="purple", linewidth=2.0, label="Estimated Clearance")
        ax.plot(times, centers, color="silver", linestyle=":", linewidth=1.5, label="Center Distance")

        # Thresholds
        ax.axhline(0.0, color="crimson", linestyle="--", linewidth=1.5, label="Collision Threshold (0.0m)")
        ax.axhline(0.45, color="darkorange", linestyle="-.", linewidth=1.2, label="Intimate Space (0.45m)")
        ax.axhline(1.20, color="gold", linestyle=":", linewidth=1.2, label="Personal Space (1.20m)")

        # Shade encounter regions
        for enc in encounters:
            ax.axvspan(enc.get("encounter_time_s", 0), enc.get("encounter_end_s", 0),
                       color="red", alpha=0.1, label="Active Encounter" if enc.get("id") == 1 else "")

        ax.set_title("Robot-Obstacle Clearance over Simulation Time", fontsize=12, fontweight="bold")
        ax.set_xlabel("Simulation Time [s]", fontsize=10)
        ax.set_ylabel("Clearance [m]", fontsize=10)
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()

        clearance_path = output_dir / "clearance_time.png"
        fig.savefig(clearance_path, bbox_inches="tight")
        plt.close(fig)
        generated_plots["clearance_time"] = str(clearance_path)

    # -------------------------------------------------------------
    # 3. Velocity & Jerk Profiles Plot
    # -------------------------------------------------------------
    if cmd_samples:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True, dpi=150)
        t_cmd = [c[0] for c in cmd_samples]
        vx = [c[1] for c in cmd_samples]
        wz = [c[2] for c in cmd_samples]

        # Calculate Jerk
        jerks = [0.0]
        for i in range(1, len(cmd_samples)):
            dt = cmd_samples[i][0] - cmd_samples[i-1][0]
            if 1e-4 < dt <= 1.0:
                accel = (cmd_samples[i][1] - cmd_samples[i-1][1]) / dt
                prev_accel = 0.0
                if i > 1:
                    prev_dt = cmd_samples[i-1][0] - cmd_samples[i-2][0]
                    if 1e-4 < prev_dt <= 1.0:
                        prev_accel = (cmd_samples[i-1][1] - cmd_samples[i-2][1]) / prev_dt
                jerks.append(abs((accel - prev_accel) / dt))
            else:
                jerks.append(0.0)

        # Plot vx
        ax1.plot(t_cmd, vx, color="royalblue", linewidth=1.5)
        ax1.set_ylabel("Linear v [m/s]", fontsize=9)
        ax1.grid(True, linestyle=":", alpha=0.6)
        ax1.set_title("Command Velocity & Motion Jerk Profiles", fontsize=11, fontweight="bold")

        # Plot wz
        ax2.plot(t_cmd, wz, color="forestgreen", linewidth=1.5)
        ax2.set_ylabel("Angular w [rad/s]", fontsize=9)
        ax2.grid(True, linestyle=":", alpha=0.6)

        # Plot Jerk
        ax3.plot(t_cmd, jerks, color="darkred", linewidth=1.2)
        ax3.set_ylabel("Linear Jerk [m/s³]", fontsize=9)
        ax3.set_xlabel("Simulation Time [s]", fontsize=10)
        ax3.grid(True, linestyle=":", alpha=0.6)

        fig.tight_layout()
        vel_path = output_dir / "velocity_profile.png"
        fig.savefig(vel_path, bbox_inches="tight")
        plt.close(fig)
        generated_plots["velocity_profile"] = str(vel_path)

    return generated_plots
