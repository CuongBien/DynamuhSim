"""
Robot Policies Implementations (P1, P2, P3a, P3b, P3c, P3d, P4)
for Data Generation and Interaction Benchmarking.
"""

import math
from typing import Tuple, Optional, Any
import numpy as np

from .geometry import apply_footprint_wall_safety_filter


class BasePolicy:
    def __init__(self, policy_name: str, sub_behavior: str, seed: int):
        self.policy_name = policy_name
        self.sub_behavior = sub_behavior
        self.seed = seed
        self.rng = np.random.RandomState(seed)
        self.params = {}
        self.first_action_sim_time = None

    def reset(self, sim_time: float):
        self.first_action_sim_time = None

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        """
        Returns a dict:
        {
            "cmd_applied": (vx, vy, wz),
            "noise": (vx_n, vy_n, wz_n),
            "cmd_intended": (vx_i, vy_i, wz_i)
        }
        """
        raise NotImplementedError


class P1Policy(BasePolicy):
    """P1: Pure MPPI (35%) - Step 3b unmodified"""
    def __init__(self, seed: int):
        super().__init__("P1", "pure_mppi", seed)

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        if self.first_action_sim_time is None:
            if abs(nav_cmd[1]) > 0.05 or abs(ry) > 0.05 or (nav_cmd[0] < 0.20 and d_human < 3.5):
                self.first_action_sim_time = sim_time
        return {
            "cmd_applied": nav_cmd,
            "noise": (0.0, 0.0, 0.0),
            "cmd_intended": nav_cmd
        }


class P2Policy(BasePolicy):
    """
    P2: MPPI + Ornstein-Uhlenbeck noise (20%)
    Strict specs (halved sigma per user instruction):
    - tau = 1.0s, sigma(vx) = 0.05, sigma(vy) = 0.05, sigma(wz) = 0.15
    - vx_max ~ U[0.30, 0.60] m/s per episode
    - Forward progress: maintains forward drive so mean vx >= 0.20 m/s and H1 goal reach >= 60%
    """
    def __init__(self, seed: int, halve_sigma: bool = True):
        super().__init__("P2", "mppi_ou_noise", seed)
        self.vx_max = float(self.rng.uniform(0.30, 0.60))
        self.tau = 1.0
        scale = 0.5 if halve_sigma else 1.0
        self.sigma_vx = 0.10 * scale
        self.sigma_vy = 0.10 * scale
        self.sigma_wz = 0.30 * scale
        self.halve_sigma = halve_sigma
        self.params = {
            "vx_max": round(self.vx_max, 4),
            "tau": self.tau,
            "sigma_vx": round(self.sigma_vx, 4),
            "sigma_vy": round(self.sigma_vy, 4),
            "sigma_wz": round(self.sigma_wz, 4),
            "halved_sigma": halve_sigma
        }
        self.eta_vx = 0.0
        self.eta_vy = 0.0
        self.eta_wz = 0.0
        self.last_sim_time = None

    def reset(self, sim_time: float):
        super().reset(sim_time)
        self.eta_vx = 0.0
        self.eta_vy = 0.0
        self.eta_wz = 0.0
        self.last_sim_time = sim_time

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        dt = 0.05
        if self.last_sim_time is not None:
            dt = max(0.001, min(0.1, sim_time - self.last_sim_time))
        self.last_sim_time = sim_time

        decay = math.exp(-dt / self.tau)
        scale = math.sqrt(max(0.0, 1.0 - math.exp(-2.0 * dt / self.tau)))

        self.eta_vx = self.eta_vx * decay + self.sigma_vx * scale * float(self.rng.normal(0.0, 1.0))
        self.eta_vy = self.eta_vy * decay + self.sigma_vy * scale * float(self.rng.normal(0.0, 1.0))
        self.eta_wz = self.eta_wz * decay + self.sigma_wz * scale * float(self.rng.normal(0.0, 1.0))

        # Intended MPPI command
        vx_nav, vy_nav, wz_nav = nav_cmd

        # Apply noise
        vx_noisy = vx_nav + self.eta_vx
        vy_noisy = vy_nav + self.eta_vy
        wz_noisy = wz_nav + self.eta_wz

        # Ensure forward progress: if MPPI intends forward motion, keep positive drive >= 0.20 m/s
        if vx_nav > 0.15 and d_human > 1.8:
            vx_noisy = max(0.20, vx_noisy)

        vx_cand = float(np.clip(vx_noisy, -0.10, self.vx_max))
        vy_cand = float(np.clip(vy_noisy, -0.45, 0.45))
        wz_cand = float(np.clip(wz_noisy, -1.20, 1.20))

        # Footprint wall safety filter protects P2 from entering wall costmap trap
        vx_out, vy_out, wz_out = apply_footprint_wall_safety_filter(
            rx, ry, ryaw, vx_cand, vy_cand, wz_cand
        )

        if self.first_action_sim_time is None:
            if abs(vy_out) > 0.08 or abs(ry) > 0.05:
                self.first_action_sim_time = sim_time

        return {
            "cmd_applied": (vx_out, vy_out, wz_out),
            "noise": (self.eta_vx, self.eta_vy, self.eta_wz),
            "cmd_intended": nav_cmd
        }


class P3aPolicy(BasePolicy):
    """P3a: Stop & Wait"""
    def __init__(self, seed: int):
        super().__init__("P3", "P3a_stop_and_wait", seed)
        self.d_stop = float(self.rng.uniform(1.5, 3.0))
        self.T_wait = float(self.rng.uniform(1.0, 4.0))
        self.params = {
            "d_stop": round(self.d_stop, 3),
            "T_wait": round(self.T_wait, 3)
        }
        self.state = "WAITING"
        self.stop_start_sim_time = None

    def reset(self, sim_time: float):
        super().reset(sim_time)
        self.state = "WAITING"
        self.stop_start_sim_time = None

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        if self.state == "WAITING":
            if d_human <= self.d_stop:
                self.state = "STOPPING"
                self.stop_start_sim_time = sim_time
                if self.first_action_sim_time is None:
                    self.first_action_sim_time = sim_time
                return {"cmd_applied": (0.0, 0.0, 0.0), "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
            return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
        elif self.state == "STOPPING":
            if sim_time - self.stop_start_sim_time < self.T_wait:
                return {"cmd_applied": (0.0, 0.0, 0.0), "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
            else:
                self.state = "RESUMED"
                return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
        else:  # RESUMED
            return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}


class P3bPolicy(BasePolicy):
    """P3b: Lateral Nudge"""
    def __init__(self, seed: int):
        super().__init__("P3", "P3b_lateral_nudge", seed)
        self.d_nudge = float(self.rng.uniform(1.5, 4.0))
        side = float(self.rng.choice([-1.0, 1.0]))
        mag = float(self.rng.uniform(0.20, 0.35))
        self.vy_cmd = side * mag
        self.dur_nudge = float(self.rng.uniform(1.0, 2.0))
        self.params = {
            "d_nudge": round(self.d_nudge, 3),
            "vy_cmd": round(self.vy_cmd, 3),
            "dur_nudge": round(self.dur_nudge, 3)
        }
        self.state = "WAITING"
        self.nudge_start_sim_time = None
        self.held_y = 0.0

    def reset(self, sim_time: float):
        super().reset(sim_time)
        self.state = "WAITING"
        self.nudge_start_sim_time = None
        self.held_y = 0.0

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        vx_nav, vy_nav, wz_nav = nav_cmd

        if self.state == "WAITING":
            if d_human <= self.d_nudge:
                self.state = "NUDGING"
                self.nudge_start_sim_time = sim_time
                if self.first_action_sim_time is None:
                    self.first_action_sim_time = sim_time
                vx_out = max(0.15, vx_nav) if vx_nav > 0.05 else 0.20
                cmd = (vx_out, self.vy_cmd, 0.0)
                cmd_filtered = apply_footprint_wall_safety_filter(rx, ry, ryaw, *cmd)
                return {"cmd_applied": cmd_filtered, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
            return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}

        elif self.state == "NUDGING":
            vx_out = max(0.15, vx_nav) if vx_nav > 0.05 else 0.20
            if sim_time - self.nudge_start_sim_time >= self.dur_nudge:
                self.state = "HOLDING"
                self.held_y = ry
                cmd = (vx_out, 0.0, 0.0)
            else:
                cmd = (vx_out, self.vy_cmd, 0.0)
            cmd_filtered = apply_footprint_wall_safety_filter(rx, ry, ryaw, *cmd)
            return {"cmd_applied": cmd_filtered, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}

        elif self.state == "HOLDING":
            human_passed = (hx is not None and rx >= hx - 0.20) or (d_human > self.d_nudge + 0.5 and rx > -10.0)
            if human_passed:
                self.state = "RETURNING"
                cmd = (vx_nav, -1.5 * ry, wz_nav)
            else:
                vy_hold = float(np.clip(-2.0 * (ry - self.held_y), -0.20, 0.20))
                vx_out = max(0.15, vx_nav) if vx_nav > 0.05 else 0.20
                cmd = (vx_out, vy_hold, 0.0)
            cmd_filtered = apply_footprint_wall_safety_filter(rx, ry, ryaw, *cmd)
            return {"cmd_applied": cmd_filtered, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}

        elif self.state == "RETURNING":
            if abs(ry) <= 0.05:
                self.state = "COMPLETED"
                return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
            vy_return = float(np.clip(-1.5 * ry, -0.25, 0.25))
            cmd = (vx_nav, vy_return, wz_nav)
            cmd_filtered = apply_footprint_wall_safety_filter(rx, ry, ryaw, *cmd)
            return {"cmd_applied": cmd_filtered, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}

        else:  # COMPLETED
            return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}


class P3cPolicy(BasePolicy):
    """P3c: Slow Down"""
    def __init__(self, seed: int):
        super().__init__("P3", "P3c_slow_down", seed)
        self.k_slow = float(self.rng.uniform(0.3, 0.6))
        self.params = {"k_slow": round(self.k_slow, 3)}

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        vx_nav, vy_nav, wz_nav = nav_cmd
        if d_human < 3.0:
            if self.first_action_sim_time is None:
                self.first_action_sim_time = sim_time
            vx_out = vx_nav * self.k_slow if vx_nav > 0.0 else vx_nav
            return {"cmd_applied": (vx_out, vy_nav, wz_nav), "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}
        return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}


class P3dPolicy(BasePolicy):
    """P3d: Keep Unchanged (Internal Control)"""
    def __init__(self, seed: int):
        super().__init__("P3", "P3d_keep_unchanged", seed)

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        if self.first_action_sim_time is None:
            if abs(nav_cmd[1]) > 0.05 or abs(ry) > 0.05 or (nav_cmd[0] < 0.20 and d_human < 3.5):
                self.first_action_sim_time = sim_time
        return {"cmd_applied": nav_cmd, "noise": (0.0, 0.0, 0.0), "cmd_intended": nav_cmd}


class P4Policy(BasePolicy):
    """
    P4: Constrained Random with Footprint-Aware Wall Safety Filter (15%).
    Resamples every 0.5-1.5s in vx in [-0.1, 0.5], vy in [-0.3, 0.3], wz in [-1, 1].
    Strict yaw limit in +/- 45 deg, vx clamping when facing walls, and active anti-freeze drive.
    NO FILTER WITH HUMAN.
    """
    def __init__(self, seed: int):
        super().__init__("P4", "constrained_random", seed)
        self.next_resample_time = 0.0
        self.current_cmd = (0.25, 0.0, 0.0)
        self.params = {
            "yaw_limit_deg": 45.0,
            "resample_interval": "[0.5, 1.5]",
            "vx_range": "[-0.10, 0.50]",
            "vy_range": "[-0.30, 0.30]",
            "wz_range": "[-1.00, 1.00]"
        }

    def reset(self, sim_time: float):
        super().reset(sim_time)
        self.next_resample_time = sim_time + float(self.rng.uniform(0.5, 1.5))
        self.current_cmd = (float(self.rng.uniform(0.20, 0.40)), 0.0, 0.0)

    def compute_cmd(
        self, sim_time: float, rx: float, ry: float, ryaw: float,
        hx: Optional[float], hy: Optional[float], d_human: float,
        laser_scan: Optional[Any], nav_cmd: Tuple[float, float, float]
    ) -> dict:
        if sim_time >= self.next_resample_time:
            dt_step = float(self.rng.uniform(0.5, 1.5))
            self.next_resample_time = sim_time + dt_step
            vx = float(self.rng.uniform(-0.10, 0.50))
            vy = float(self.rng.uniform(-0.30, 0.30))
            wz = float(self.rng.uniform(-1.00, 1.00))

            # Bias wz to keep yaw strictly in [-45 deg, +45 deg]
            if ryaw > 0.45:
                wz = float(self.rng.uniform(-1.0, -0.2))
            elif ryaw < -0.45:
                wz = float(self.rng.uniform(0.2, 1.0))

            self.current_cmd = (vx, vy, wz)
            if self.first_action_sim_time is None and (abs(vy) > 0.08 or abs(wz) > 0.2):
                self.first_action_sim_time = sim_time

        vx_intended, vy_intended, wz_intended = self.current_cmd

        # Footprint-aware wall safety filter (NO filter with human)
        vx_out, vy_out, wz_out = apply_footprint_wall_safety_filter(
            rx, ry, ryaw, vx_intended, vy_intended, wz_intended
        )

        return {
            "cmd_applied": (vx_out, vy_out, wz_out),
            "noise": (0.0, 0.0, 0.0),
            "cmd_intended": (vx_intended, vy_intended, wz_intended)
        }


def create_policy_instance(policy_code: str, sub_behavior: str, seed: int, halve_sigma: bool = True) -> BasePolicy:
    if policy_code == "P1":
        return P1Policy(seed)
    elif policy_code == "P2":
        return P2Policy(seed, halve_sigma=halve_sigma)
    elif policy_code in ["P3", "P3a", "P3b", "P3c", "P3d"]:
        if sub_behavior in ["P3a", "P3a_stop_and_wait"]:
            return P3aPolicy(seed)
        elif sub_behavior in ["P3b", "P3b_lateral_nudge"]:
            return P3bPolicy(seed)
        elif sub_behavior in ["P3c", "P3c_slow_down"]:
            return P3cPolicy(seed)
        elif sub_behavior in ["P3d", "P3d_keep_unchanged"]:
            return P3dPolicy(seed)
        else:
            raise ValueError(f"Unknown P3 sub-behavior: {sub_behavior}")
    elif policy_code == "P4":
        return P4Policy(seed)
    else:
        raise ValueError(f"Unknown policy code: {policy_code}")
