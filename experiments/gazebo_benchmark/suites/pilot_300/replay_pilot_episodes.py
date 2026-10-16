#!/usr/bin/env python3
"""
Replay 5 Pilot Episodes from recorded cmd_vel:
- Selects 5 episodes meeting Section C specification:
  * Ep 30: H1 interactive (corridor_090, N=2, Nav2=True)
  * Ep 45: H3 interactive (corridor_120, N=10, Nav2=True)
  * Ep 58: Non-Nav2 open-loop (corridor_120, N=10, Nav2=False)
  * Ep 5:  H2 interactive (corridor_090, N=5, Nav2=True)
  * Ep 18: Non-interactive (corridor_090, N=5, Nav2=True)
- Replays recorded cmd_vel aligned to sim-time with Gazebo synchronization
- Records ground truth of both robot and humans in original and replay
- Reports Mean and Max trajectory deviation (Target: Mean < 1 cm, Max < 2 cm)
- Outputs raw JSON: experiments/gazebo_benchmark/results/pilot_300/pilot_5_replay_results.json
"""

import os
import sys
import time
import json
import signal
import subprocess
import numpy as np
from typing import Dict, List, Any

ROOT_DIR = "/home/cuongbien/DynamuhSim"
sys.path.insert(0, os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/core"))
OUTPUT_JSON = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_5_replay_results.json")

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class PrecisionPilotReplayer(Node):
    def __init__(self, recorded_cmds: List[Dict], t_sim_start: float, duration_s: float = 20.0):
        super().__init__('precision_pilot_replayer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.t_sim_start = t_sim_start
        self.duration_s = duration_s

        self.cmds = [(float(c.get('t_rel', 0.0)), float(c.get('vx', 0.0)), float(c.get('vy', 0.0)), float(c.get('w', 0.0))) for c in recorded_cmds]
        self.cmds.sort(key=lambda x: x[0])

        self.sim_time = 0.0
        self.cmd_idx = 0
        self.last_pub_sim_time = -1.0
        self.replayed_gt = []
        self.active = False
        self.completed = False

        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, 100)
        self.gt_sub = self.create_subscription(String, '/rvo2_human/ground_truth', self._gt_cb, 100)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 50)

    def _gt_cb(self, msg: String):
        if not self.active or self.completed:
            return
        t_rel = self.sim_time - self.t_sim_start
        try:
            d = json.loads(msg.data)
            d["sim_time_rel"] = round(t_rel, 3)
            self.replayed_gt.append(d)
        except Exception:
            pass

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if not self.active:
            if self.sim_time >= self.t_sim_start:
                self.active = True
                self.last_pub_sim_time = self.sim_time
            else:
                return

        t_rel = self.sim_time - self.t_sim_start
        if t_rel >= self.duration_s:
            self.completed = True
            return

        # Advance scheduled command
        prev_idx = self.cmd_idx
        while self.cmd_idx < len(self.cmds) - 1 and self.cmds[self.cmd_idx + 1][0] <= t_rel:
            self.cmd_idx += 1

        # Dispatch at 50 Hz sim-time or on index advance
        if (self.sim_time - self.last_pub_sim_time >= 0.02) or (self.cmd_idx != prev_idx):
            self.last_pub_sim_time = self.sim_time
            sched_t, vx, vy, w = self.cmds[self.cmd_idx]
            tw = TwistStamped()
            tw.header.stamp = msg.clock
            tw.header.frame_id = 'base_footprint'
            tw.twist.linear.x = float(vx)
            tw.twist.linear.y = float(vy)
            tw.twist.angular.z = float(w)
            self.cmd_pub.publish(tw)


def run_replay_for_episode(ep_id: int, episodes_dir: str, domain_id: int = 91) -> Dict[str, Any]:
    ep_dir = os.path.join(episodes_dir, f"ep_{ep_id:04d}")
    with open(os.path.join(ep_dir, "manifest.json")) as f:
        m = json.load(f)
    with open(os.path.join(ep_dir, "cmd_vel.json")) as f:
        recorded_cmds = json.load(f)
    with open(os.path.join(ep_dir, "ground_truth.json")) as f:
        orig_gt = json.load(f)

    corr = m.get("corridor", "corridor_090")
    corr_type = "090" if corr == "corridor_090" else "120"
    world_name = "corridor_0.90" if corr_type == "090" else "corridor_1.20"
    world_file = os.path.join(ROOT_DIR, f"experiments/gazebo_benchmark/configs/generated/world_ep_{ep_id:04d}.sdf")
    robot_file = os.path.join(ep_dir, "robot.sdf")

    # Calibrate for 50ms controller cycle delay
    time_offset_s = 0.05
    t_sim_start = orig_gt[0].get("timestamp", 0.0) - orig_gt[0].get("sim_time_rel", 0.0) + time_offset_s
    duration_s = min(orig_gt[-1].get("sim_time_rel", 20.0), 20.0)
    if m.get("time_to_goal") is not None:
        duration_s = min(duration_s, float(m["time_to_goal"]))

    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = f"gz_replay_{domain_id}_ep_{ep_id:04d}"
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    corridor_models = os.path.join(ROOT_DIR, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(ROOT_DIR, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.join(os.path.dirname(os.path.dirname(plugin_share)), 'lib')
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"

    procs = []
    try:
        sim_cmd = [
            "ros2", "launch", "custom_corridor", "corridor_tb3.launch.py",
            f"width:={'0.90' if corr_type == '090' else '1.20'}",
            f"custom_world:={world_file}",
            f"custom_robot_model:={robot_file}",
            "obstacle:=human",
            "gui:=false", "rviz:=false",
            "robot_model:=mecanum",
            "x:=-13.0", "y:=0.0", "yaw:=0.0",
            "paused:=true"
        ]
        p_sim = subprocess.Popen(sim_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        procs.append(p_sim)
        time.sleep(2.5)

        rclpy.init()
        replayer = PrecisionPilotReplayer(recorded_cmds, t_sim_start, duration_s=duration_s)

        t_spin0 = time.time()
        while time.time() - t_spin0 < 0.8:
            rclpy.spin_once(replayer, timeout_sec=0.05)

        unpause_cmd = [
            "gz", "service", "-s", f"/world/{world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", "pause: false"
        ]
        subprocess.run(unpause_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        wall_start = time.time()
        while rclpy.ok() and not replayer.completed and (time.time() - wall_start) < 35.0:
            rclpy.spin_once(replayer, timeout_sec=0.01)

        rep_gt = replayer.replayed_gt
        replayer.destroy_node()
        rclpy.shutdown()

        t_eval = np.arange(0.1, duration_s - 0.1, 0.1)
        orig_t = np.array([g.get('sim_time_rel', 0.0) for g in orig_gt])
        orig_rx = np.array([g.get('robot', {}).get('x', -13.0) for g in orig_gt])
        orig_ry = np.array([g.get('robot', {}).get('y', 0.0) for g in orig_gt])

        rep_t = np.array([g.get('sim_time_rel', 0.0) for g in rep_gt])
        rep_rx = np.array([g.get('robot', {}).get('x', -13.0) for g in rep_gt])
        rep_ry = np.array([g.get('robot', {}).get('y', 0.0) for g in rep_gt])

        ox = np.interp(t_eval, orig_t, orig_rx)
        oy = np.interp(t_eval, orig_t, orig_ry)
        rx = np.interp(t_eval, rep_t, rep_rx)
        ry = np.interp(t_eval, rep_t, rep_ry)

        r_errs = np.hypot((rx - ox) * 1000.0, (ry - oy) * 1000.0)
        r_mean = float(np.mean(r_errs))
        r_max = float(np.max(r_errs))

        # Human 1
        orig_hx = np.array([g.get('humans', [{}])[0].get('x', 0.0) for g in orig_gt])
        orig_hy = np.array([g.get('humans', [{}])[0].get('y', 0.0) for g in orig_gt])
        rep_hx = np.array([g.get('humans', [{}])[0].get('x', 0.0) for g in rep_gt])
        rep_hy = np.array([g.get('humans', [{}])[0].get('y', 0.0) for g in rep_gt])

        ohx = np.interp(t_eval, orig_t, orig_hx)
        ohy = np.interp(t_eval, orig_t, orig_hy)
        rhx = np.interp(t_eval, rep_t, rep_hx)
        rhy = np.interp(t_eval, rep_t, rep_hy)

        h_errs = np.hypot((rhx - ohx) * 1000.0, (rhy - ohy) * 1000.0)
        h_mean = float(np.mean(h_errs))
        h_max = float(np.max(h_errs))

        passed = bool(r_mean < 10.0 and r_max < 20.0 and h_mean < 10.0 and h_max < 20.0)
        result = {
            "episode_id": ep_id,
            "persona": m.get("persona"),
            "corridor": corr,
            "num_humans": m.get("num_humans"),
            "use_nav2": m.get("use_nav2"),
            "duration_s": round(duration_s, 2),
            "robot_mean_deviation_mm": round(r_mean, 2),
            "robot_max_deviation_mm": round(r_max, 2),
            "human_mean_deviation_mm": round(h_mean, 2),
            "human_max_deviation_mm": round(h_max, 2),
            "passed": passed
        }
        return result

    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                time.sleep(0.1)
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def main():
    episodes_dir = os.path.join(ROOT_DIR, "experiments/gazebo_benchmark/results/pilot_300/pilot_300_dataset/episodes")

    # The 5 selected representative episodes according to Section C:
    # 1. H1 interactive: Ep 30
    # 2. H3 interactive: Ep 45
    # 3. Non-Nav2 open-loop: Ep 58
    # 4. H2 interactive: Ep 5
    # 5. Non-interactive: Ep 18
    selected_eps = [30, 45, 58, 5, 18]

    print("=" * 80)
    print("REPLAY BENCHMARK: 5 PILOT EPISODES")
    print(f"Target Criteria: Mean Deviation < 10.0 mm (1 cm), Max Deviation < 20.0 mm (2 cm)")
    print("=" * 80)

    results = []
    base_domain = 91

    for i, ep_id in enumerate(selected_eps):
        dom = base_domain + i
        # Run each episode in an isolated process via subprocess to guarantee clean FastDDS state
        script_snippet = f"""
import sys, json
sys.path.insert(0, '{os.path.join(ROOT_DIR, "experiments/gazebo_benchmark")}')
from replay_pilot_episodes import run_replay_for_episode
res = run_replay_for_episode({ep_id}, '{episodes_dir}', {dom})
print('JSON_RESULT:' + json.dumps(res))
"""
        cmd = ["python3", "-c", script_snippet]
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(dom)
        p = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        res = None
        for line in p.stdout.splitlines():
            if line.startswith("JSON_RESULT:"):
                res = json.loads(line[len("JSON_RESULT:"):])
                break

        if res is None:
            print(f"Failed to replay ep {ep_id}: {p.stderr}")
            continue

        results.append(res)
        print(f"[{i+1}/5] Ep #{res['episode_id']:04d} ({res['persona']}, corr={res['corridor']}, Nav2={res['use_nav2']}): "
              f"Robot Mean={res['robot_mean_deviation_mm']:.2f}mm, Max={res['robot_max_deviation_mm']:.2f}mm | "
              f"Human Mean={res['human_mean_deviation_mm']:.2f}mm, Max={res['human_max_deviation_mm']:.2f}mm | "
              f"{'PASS' if res['passed'] else 'FAIL'}")

    all_passed = all(r.get('passed', False) for r in results) and len(results) == 5

    output_data = {
        "target_criteria": {
            "mean_max_mm": 10.0,
            "max_max_mm": 20.0
        },
        "episodes_tested": len(results),
        "all_passed": all_passed,
        "results": results
    }

    with open(OUTPUT_JSON, "w") as fp:
        json.dump(output_data, fp, indent=2)

    print("=" * 80)
    print(f"REPLAY BENCHMARK COMPLETE: {'ALL PASSED' if all_passed else 'SOME FAILED'}")
    print(f"Exported raw JSON to: {OUTPUT_JSON}")
    print("=" * 80)


if __name__ == '__main__':
    main()
