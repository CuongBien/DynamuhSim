from pathlib import Path
import json
import os
import csv
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(os.environ.get(
    "WM_DATASET",
    "~/nav_ws/wm_restore/wm_episode_batch/generated/dataset01"
)).expanduser()

OUT = Path.home() / "nav_ws/wm_analysis"
OUT.mkdir(parents=True, exist_ok=True)

if not (ROOT / "dataset.json").is_file():
    raise SystemExit(f"Khong tim thay dataset: {ROOT}")

dataset = json.loads((ROOT / "dataset.json").read_text())
episodes = dataset["episodes"]

rows = []
actions = [[], [], []]
nearest_ranges = []
depth_values = []
human_motion = {"standing": [], "walking": []}

# [horizon, class] class = unknown, free, occupied
bev_counts = np.zeros((3, 3), dtype=np.int64)

for meta in episodes:
    name = meta["episode_id"]
    split = meta["split"]
    ep = ROOT / "episodes" / name

    if not ep.is_dir():
        continue

    rows.append({
        "episode": name,
        "split": split,
        "scenario": meta.get("scenario", "unknown"),
        "windows": meta.get("windows", 0),
        "wm_valid": meta.get(
            "wm_observed_bev_eligible_windows", 0
        ),
        "navigation_status": meta.get(
            "navigation_status", "unknown"
        )
    })

    # Khong xem nhan test truoc khi chot mo hinh.
    if split == "test":
        continue

    scenario = "standing" if "standing" in name else "walking"

    # 1. Robot action, LiDAR
    with np.load(
        ep / "inputs/samples_SENSOR_ONLY.npz",
        allow_pickle=False
    ) as samples:

        # Lay moi 10 cua so (~1 giay) de giam trung lap
        a = samples["action"][::10, -1, :]

        for j in range(3):
            actions[j].extend(a[:, j].tolist())

        scan = samples["scan_ranges_m"][::10, -1, :]
        valid = samples["scan_ray_valid"][::10, -1, :]

        nearest = np.where(valid, scan, np.inf).min(axis=1)
        nearest_ranges.extend(
            nearest[np.isfinite(nearest)].tolist()
        )

        n = samples["depth_index"].shape[0]
        stride = max(1, n // 20)
        depth_ids = np.unique(
            samples["depth_index"][::stride, -1]
        )

    # 2. Depth: doc mmap, khong nap toan bo vao RAM
    depth = np.load(
        ep / "inputs/depth_SENSOR_ONLY.npy",
        mmap_mode="r"
    )

    for idx in depth_ids:
        frame = depth[int(idx), ::8, ::8]
        good = (
            np.isfinite(frame)
            & (frame >= 0.28)
            & (frame <= 5.0)
        )
        depth_values.extend(frame[good].tolist())

    del depth

    # 3. Future BEV labels + masks
    with np.load(
        ep / "labels/task_masks_LABEL_ONLY.npz",
        allow_pickle=False
    ) as masks, np.load(
        ep / "labels/future_observed_bev_LABEL_ONLY.npz",
        allow_pickle=False
    ) as bev:

        eligible = masks["wm_observed_bev_train_valid"]
        frame_ok = bev["frame_valid"]
        occ = bev["occupancy"]

        for horizon in range(3):
            keep = eligible & frame_ok[:, horizon]
            selected = occ[keep, horizon].ravel()

            # -1 -> 0, 0 -> 1, 1 -> 2
            counts = np.bincount(
                selected.astype(np.int16) + 1,
                minlength=3
            )
            bev_counts[horizon] += counts[:3]

    # 4. Dich chuyen cua 6 nguoi trong 3 giay.
    # Day la ground truth LABEL, khong phai sensor input.
    with np.load(
        ep / "labels/future_trajectories_actions_LABEL_ONLY.npz",
        allow_pickle=False
    ) as labels:

        start = labels["human_xy_at_anchor_m"][::10]
        future = labels["human_xy_future_m"][::10, -1]

        displacement = np.linalg.norm(
            future - start, axis=-1
        )
        human_motion[scenario].extend(
            displacement.ravel().tolist()
        )

# Luu CSV thong ke
with (OUT / "episode_summary.csv").open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "episode", "split", "scenario",
        "windows", "wm_valid", "navigation_status"
    ])
    writer.writeheader()
    writer.writerows(rows)

def save_hist(values, title, xlabel, filename, bins=40):
    if not values:
        return
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(values, bins=bins, edgecolor="black", linewidth=.3)
    ax.set(title=title, xlabel=xlabel, ylabel="Frequency")
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=160)
    plt.close(fig)

# Bieu do 1: so windows tung episode
fig, ax = plt.subplots(figsize=(11, 4))
ax.bar(
    [r["episode"] for r in rows],
    [r["windows"] for r in rows]
)
ax.set(ylabel="Windows", title="Windows per episode")
ax.tick_params(axis="x", rotation=40)
fig.tight_layout()
fig.savefig(OUT / "01_episode_distribution.png", dpi=160)
plt.close(fig)

# Bieu do 2-4: robot actions
for j, name in enumerate(["vx", "vy", "wz"]):
    unit = "rad/s" if name == "wz" else "m/s"
    save_hist(
        actions[j],
        f"Robot action distribution: {name}",
        f"{name} ({unit})",
        f"0{j+2}_action_{name}.png"
    )

# Bieu do 5-6: sensors
save_hist(
    nearest_ranges,
    "Nearest valid LiDAR range",
    "Distance (m)",
    "05_lidar_range.png"
)

save_hist(
    depth_values,
    "Valid Depth pixel distribution",
    "Distance (m)",
    "06_depth_distribution.png"
)

# Bieu do 7: BEV class balance theo 3 horizons
totals = bev_counts.sum(axis=1, keepdims=True)
ratios = bev_counts / np.maximum(totals, 1)

fig, ax = plt.subplots(figsize=(8, 4.5))
horizons = ["+0.5s", "+1s", "+3s"]
bottom = np.zeros(3)

for j, label in enumerate(["Unknown", "Free", "Occupied"]):
    values = ratios[:, j]
    ax.bar(horizons, values, bottom=bottom, label=label)
    bottom += values

ax.set(
    title="Future BEV class distribution",
    ylabel="Fraction of grid cells",
    ylim=(0, 1)
)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "07_bev_distribution.png", dpi=160)
plt.close(fig)

# Bieu do 8: human displacement
fig, ax = plt.subplots(figsize=(8, 4.5))

for name, values in human_motion.items():
    if values:
        ax.hist(
            values,
            bins=40,
            alpha=0.5,
            density=True,
            label=name
        )

ax.set(
    title="Human displacement in 3 seconds",
    xlabel="Displacement (m)",
    ylabel="Density"
)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "08_human_motion.png", dpi=160)
plt.close(fig)

print("\n=== DATASET SUMMARY ===")
for row in rows:
    print(row)

print("\n=== FUTURE BEV FRACTIONS ===")
for h, v in zip(horizons, ratios):
    print(h, {
        "unknown": round(float(v[0]), 4),
        "free": round(float(v[1]), 4),
        "occupied": round(float(v[2]), 4)
    })

print("\nHuman displacement (median, m):")
for name, values in human_motion.items():
    if values:
        print(name, round(float(np.median(values)), 3))

print("\nSaved results to:", OUT)
