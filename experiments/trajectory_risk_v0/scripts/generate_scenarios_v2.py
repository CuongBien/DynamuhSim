#!/usr/bin/env python3
"""
generate_scenarios_v2.py
Sinh ra 13 scenario JSON (scenario_v2_001 -> scenario_v2_013) vào thư mục scenarios_v1/.

Thiết kế:
  - Mỗi scenario: 10-12 người.
  - 8 người background: đi ở các lane |y| >= 3.2 (KHÔNG chắn đường robot ở y≈0).
  - 2-4 người interaction: behavior đặc trưng của từng family.
  - Vị trí start ngẫu nhiên theo seed -> đảm bảo randomness.

Cách chạy:
  cd ~/nav_ws/experiments/trajectory_risk_v0
  python3 scripts/generate_scenarios_v2.py
"""

import json, os, math, random

PI = math.pi

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR   = os.path.join(SCRIPT_DIR, "..", "scenarios_v1")


# ────────────────────────────────────────────────────────────────────────────
# Helper: 8 background humans dùng chung (lane cố định, start ngẫu nhiên theo seed)
# ────────────────────────────────────────────────────────────────────────────
def make_bg(start_id: int, seed: int):
    """8 background humans ở 8 lane xa trục robot (|y| >= 3.2)."""
    rng = random.Random(seed)
    # (y, yaw_base, speed_base)
    lanes = [
        ( 3.2,  0.0,   0.42),  # bắc, đi sang phải
        ( 3.8,  PI,    0.50),  # bắc, đi sang trái
        ( 4.3,  0.0,   0.57),  # bắc, đi sang phải
        ( 4.0,  PI,    0.44),  # bắc, đi sang trái
        (-3.2,  PI,    0.46),  # nam, đi sang trái
        (-3.8,  0.0,   0.54),  # nam, đi sang phải
        (-4.3,  PI,    0.48),  # nam, đi sang trái
        (-4.0,  0.0,   0.60),  # nam, đi sang phải
    ]
    humans = []
    for i, (y, yaw, spd) in enumerate(lanes):
        start_x = round(rng.uniform(-12.0, 2.0), 2)
        speed   = round(max(0.30, min(0.70, spd + rng.uniform(-0.07, 0.07))), 2)
        wps     = [[-13.5, y], [4.0, y]] if yaw == 0 else [[4.0, y], [-13.5, y]]
        humans.append({
            "id": start_id + i,
            "mode": "autonomous",
            "behavior": "background_flow",
            "speed": speed,
            "waypoints": wps,
            "loop": True,
            "waypoint_tolerance": 0.12,
            "start": {"x": start_x, "y": y, "yaw": round(yaw, 5)}
        })
    return humans


def build(ep_id: int, family: str, desc: str, interaction: list):
    """Ghép interaction + 8 background thành dict scenario hoàn chỉnh."""
    bg = make_bg(len(interaction) + 1, seed=ep_id)
    humans = interaction + bg
    return {
        "episode_id": ep_id,
        "scenario_family": family,
        "description": desc,
        "robot_start": {"x": -13.0, "y": 0.0, "yaw": 0.0},
        "robot_goal":  {"x":  3.5,  "y": 0.0, "yaw": 0.0},
        "human_count": len(humans),
        "interaction_human_count": len(interaction),
        "background_human_count": len(bg),
        "humans": humans,
    }


# ════════════════════════════════════════════════════════════════════════════
# 101  empty_safe  — 0 interaction, 12 background (lane xa)
# ════════════════════════════════════════════════════════════════════════════
def _s101():
    lanes12 = [
        # (y,    yaw,  speed, start_x)
        ( 3.2,  0.0,  0.42, -11.0),
        ( 3.8,  PI,   0.50,   1.0),
        ( 4.3,  0.0,  0.57,  -5.0),
        ( 4.0,  PI,   0.44,  -8.0),
        ( 3.5,  0.0,  0.38,  -3.0),
        ( 4.6,  PI,   0.60, -10.0),
        (-3.2,  PI,   0.46,  -6.0),
        (-3.8,  0.0,  0.54,  -9.0),
        (-4.3,  PI,   0.48,  -2.0),
        (-4.0,  0.0,  0.60, -12.0),
        (-3.5,  PI,   0.45,  -4.0),
        (-4.6,  0.0,  0.62,  -7.0),
    ]
    humans = []
    for i, (y, yaw, spd, sx) in enumerate(lanes12, 1):
        wps = [[-13.5, y], [4.0, y]] if yaw == 0 else [[4.0, y], [-13.5, y]]
        humans.append({
            "id": i, "mode": "autonomous", "behavior": "background_flow",
            "speed": spd, "waypoints": wps, "loop": True,
            "waypoint_tolerance": 0.12,
            "start": {"x": sx, "y": y, "yaw": round(yaw, 5)}
        })
    return {
        "episode_id": 101,
        "scenario_family": "empty_safe",
        "description": "12 background humans ở side-lane |y|>=3.2. Trục robot y=0 hoàn toàn trống.",
        "robot_start": {"x": -13.0, "y": 0.0, "yaw": 0.0},
        "robot_goal":  {"x":  3.5,  "y": 0.0, "yaw": 0.0},
        "human_count": 12,
        "interaction_human_count": 0,
        "background_human_count": 12,
        "humans": humans,
    }


# ════════════════════════════════════════════════════════════════════════════
# 102  static_people  — 4 người đứng gần trục + 8 background = 12
# ════════════════════════════════════════════════════════════════════════════
def _s102():
    interaction = [
        {"id":1,"mode":"stationary","behavior":"static_person",
         "start":{"x":-10.5,"y": 0.80,"yaw": 0.0}},
        {"id":2,"mode":"stationary","behavior":"static_person",
         "start":{"x": -6.0,"y":-0.85,"yaw": PI}},
        {"id":3,"mode":"stationary","behavior":"static_person",
         "start":{"x": -2.0,"y": 0.55,"yaw": 0.0}},
        {"id":4,"mode":"stationary","behavior":"static_person",
         "start":{"x": -8.5,"y":-1.20,"yaw": PI}},
    ]
    return build(102, "static_people",
        "4 người đứng im gần trục robot ở các x khác nhau + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 103  single_crossing  — 1 người cắt ngang + 9 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s103():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"crossing","speed":0.65,
         "waypoints":[[-5.5,-4.0],[-5.5,4.0]],"loop":True,"waypoint_tolerance":0.15},
    ]
    s = build(103, "single_crossing",
        "1 người cắt ngang trục robot + 8 background lane xa.",
        interaction)
    # Thêm 1 background gần hơn (y=2.6) để đủ 10
    rng = random.Random(1030)
    s["humans"].append({
        "id": len(s["humans"]) + 1,
        "mode": "autonomous", "behavior": "background_flow",
        "speed": round(rng.uniform(0.38, 0.55), 2),
        "waypoints": [[-13.5, 2.6], [4.0, 2.6]],
        "loop": True, "waypoint_tolerance": 0.12,
        "start": {"x": round(rng.uniform(-12,2), 2), "y": 2.6, "yaw": 0.0}
    })
    s["human_count"] = len(s["humans"])
    s["background_human_count"] += 1
    return s


# ════════════════════════════════════════════════════════════════════════════
# 104  multi_crossing  — 3 người cắt + 8 background = 11
# ════════════════════════════════════════════════════════════════════════════
def _s104():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"crossing","speed":0.70,
         "waypoints":[[-11.0,-3.8],[-11.0, 3.8]],"loop":True,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"crossing","speed":0.60,
         "waypoints":[[-6.0, 3.8],[-6.0, -3.8]],"loop":True,"waypoint_tolerance":0.15},
        {"id":3,"mode":"autonomous","behavior":"crossing","speed":0.55,
         "waypoints":[[-0.5,-3.8],[-0.5,  3.8]],"loop":True,"waypoint_tolerance":0.15},
    ]
    return build(104, "multi_crossing",
        "3 người cắt ngang ở x=-11, x=-6, x=-0.5 theo thứ tự robot gặp + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 105  head_on  — 2 người đi ngược + 8 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s105():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"head_on","speed":0.60,
         "waypoints":[[ 3.0, 0.40],[-12.5, 0.40]],"loop":True,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"head_on","speed":0.52,
         "waypoints":[[ 2.5,-0.40],[-12.5,-0.40]],"loop":True,"waypoint_tolerance":0.15},
    ]
    return build(105, "head_on",
        "2 người đi ngược chiều robot trên trục corridor + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 106  same_direction  — 2 người cùng chiều + 8 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s106():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"same_direction","speed":0.36,
         "waypoints":[[-12.0, 0.60],[3.0, 0.60]],"loop":False,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"same_direction","speed":0.44,
         "waypoints":[[-9.0, -0.60],[3.0,-0.60]],"loop":False,"waypoint_tolerance":0.15},
    ]
    return build(106, "same_direction",
        "2 người đi cùng chiều robot nhưng chậm hơn (v=0.36, 0.44 m/s) + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 107  mixed_flow  — 2 cùng chiều + 2 ngược chiều + 8 background = 12
# ════════════════════════════════════════════════════════════════════════════
def _s107():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"same_direction","speed":0.40,
         "waypoints":[[-11.0, 0.70],[3.0, 0.70]],"loop":False,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"same_direction","speed":0.48,
         "waypoints":[[-7.0, -0.70],[3.0,-0.70]],"loop":False,"waypoint_tolerance":0.15},
        {"id":3,"mode":"autonomous","behavior":"head_on","speed":0.58,
         "waypoints":[[ 2.8, 0.30],[-12.0, 0.30]],"loop":True,"waypoint_tolerance":0.15},
        {"id":4,"mode":"autonomous","behavior":"head_on","speed":0.50,
         "waypoints":[[ 2.0,-0.30],[-12.0,-0.30]],"loop":True,"waypoint_tolerance":0.15},
    ]
    return build(107, "mixed_flow",
        "Hai chiều: 2 same_direction + 2 head_on dải lân cận trục + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 108  group_blocking  — 4 người chắn cụm + 8 background = 12
# ════════════════════════════════════════════════════════════════════════════
def _s108():
    interaction = [
        {"id":1,"mode":"stationary","behavior":"group_blocking",
         "start":{"x":-5.5,"y":-0.90,"yaw": 0.0}},
        {"id":2,"mode":"stationary","behavior":"group_blocking",
         "start":{"x":-5.5,"y":-0.30,"yaw": PI}},
        {"id":3,"mode":"stationary","behavior":"group_blocking",
         "start":{"x":-5.5,"y": 0.30,"yaw": 0.0}},
        {"id":4,"mode":"stationary","behavior":"group_blocking",
         "start":{"x":-5.5,"y": 0.90,"yaw": PI}},
    ]
    return build(108, "group_blocking",
        "4 người đứng thành cụm chắn giữa corridor tại x=-5.5 + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 109  wait_then_cross  — 2 người đứng rồi bất ngờ qua + 8 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s109():
    interaction = [
        {"id":1,"mode":"scripted","behavior":"wait_then_cross",
         "start":{"x":-7.5,"y":-2.5,"yaw": PI/2},
         "segments":[
             {"duration": 4.0,"speed":0.0, "heading": PI/2},
             {"duration": 5.6,"speed":0.80,"heading": PI/2},
         ]},
        {"id":2,"mode":"scripted","behavior":"wait_then_cross",
         "start":{"x":-2.0,"y": 2.5,"yaw":-PI/2},
         "segments":[
             {"duration": 9.0,"speed":0.0, "heading":-PI/2},
             {"duration": 5.6,"speed":0.75,"heading":-PI/2},
         ]},
    ]
    return build(109, "wait_then_cross",
        "2 người: đứng rồi bất ngờ cắt qua trục (timing lệch nhau) + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 110  door_entry  — 2 người đi vào cửa (về phía bắc) + 8 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s110():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"door_entry","speed":0.52,
         "waypoints":[[-8.5, 0.8],[-8.5, 2.5],[-8.5, 4.5]],
         "loop":False,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"door_entry","speed":0.48,
         "waypoints":[[-3.0,-0.7],[-3.0, 2.0],[-3.0, 4.5]],
         "loop":False,"waypoint_tolerance":0.15},
    ]
    return build(110, "door_entry",
        "2 người rẽ từ corridor vào vùng cửa (di chuyển về phía bắc) + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 111  door_exit  — 2 người đi ra từ cửa vào corridor + 8 background = 10
# ════════════════════════════════════════════════════════════════════════════
def _s111():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"door_exit","speed":0.52,
         "waypoints":[[-5.5, 4.5],[-5.5, 2.0],[-5.5,-0.5]],
         "loop":False,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"door_exit","speed":0.48,
         "waypoints":[[-9.5, 4.5],[-9.5, 2.0],[-9.5,-0.8]],
         "loop":False,"waypoint_tolerance":0.15},
    ]
    return build(111, "door_exit",
        "2 người xuất hiện từ vùng cửa vào corridor (di chuyển về phía nam) + 8 background.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 112  yielding  — 1 human_yields (scripted) + 1 robot_should_yield + 8 BG = 10
# ════════════════════════════════════════════════════════════════════════════
def _s112():
    interaction = [
        {"id":1,"mode":"scripted","behavior":"human_yields",
         "start":{"x":-4.8,"y":0.65,"yaw": PI},
         "segments":[
             {"duration":2.2,"speed":0.55,"heading": PI},
             {"duration":3.2,"speed":0.0, "heading": PI},
             {"duration":4.0,"speed":0.55,"heading": PI},
         ]},
        {"id":2,"mode":"autonomous","behavior":"robot_should_yield","speed":0.72,
         "waypoints":[[-10.0,-2.5],[-10.0,2.5]],"loop":False,"waypoint_tolerance":0.10},
    ]
    return build(112, "yielding",
        "1 human_yields (dừng nhường) + 1 robot_should_yield (người cắt ngang nhanh) + 8 BG.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# 113  dense_social  — 4 behavior hỗn hợp + 8 background = 12
# ════════════════════════════════════════════════════════════════════════════
def _s113():
    interaction = [
        {"id":1,"mode":"autonomous","behavior":"crossing","speed":0.65,
         "waypoints":[[-10.0,-3.5],[-10.0,3.5]],"loop":True,"waypoint_tolerance":0.15},
        {"id":2,"mode":"autonomous","behavior":"head_on","speed":0.55,
         "waypoints":[[ 3.0, 0.30],[-12.0,0.30]],"loop":True,"waypoint_tolerance":0.15},
        {"id":3,"mode":"autonomous","behavior":"same_direction","speed":0.42,
         "waypoints":[[-9.0,-0.60],[3.0,-0.60]],"loop":False,"waypoint_tolerance":0.15},
        {"id":4,"mode":"scripted","behavior":"wait_then_cross",
         "start":{"x":-3.5,"y":-2.5,"yaw": PI/2},
         "segments":[
             {"duration":7.0,"speed":0.0, "heading": PI/2},
             {"duration":5.6,"speed":0.75,"heading": PI/2},
         ]},
    ]
    return build(113, "dense_social",
        "4 behavior hỗn hợp (crossing+head_on+same_dir+wait_cross) + 8 background = 12.",
        interaction)


# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════
SCENARIOS = [
    (1,  _s101()),  # empty_safe
    (2,  _s102()),  # static_people
    (3,  _s103()),  # single_crossing
    (4,  _s104()),  # multi_crossing
    (5,  _s105()),  # head_on
    (6,  _s106()),  # same_direction
    (7,  _s107()),  # mixed_flow
    (8,  _s108()),  # group_blocking
    (9,  _s109()),  # wait_then_cross
    (10, _s110()),  # door_entry
    (11, _s111()),  # door_exit
    (12, _s112()),  # yielding
    (13, _s113()),  # dense_social
]

if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output dir: {os.path.abspath(OUTPUT_DIR)}\n")
    for idx, s in SCENARIOS:
        fname = f"scenario_v2_{idx:03d}.json"
        fpath = os.path.join(OUTPUT_DIR, fname)
        with open(fpath, "w") as f:
            json.dump(s, f, indent=2)
        n_total = s["human_count"]
        n_int   = s["interaction_human_count"]
        n_bg    = s["background_human_count"]
        print(f"[{idx:02d}] {fname}  family={s['scenario_family']:20s}  "
              f"total={n_total:2d}  (interaction={n_int}, bg={n_bg})")
    print(f"\n✓ {len(SCENARIOS)} files generated.")