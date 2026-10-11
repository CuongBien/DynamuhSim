import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    with open('experiments/pedestrian_persona_benchmark/ground_truth_dataset.json') as f:
        data = json.load(f)

    headon = [ep for ep in data if ep['scenario'] == 'head_on']

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))

    # --- Plot 1: Lateral Deviation Distribution for Head-on Encounters ---
    h1_devs = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H1']
    h2_low = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H2' and ep['alpha'] < 0.20]
    h2_mid = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H2' and 0.20 <= ep['alpha'] < 0.30]
    h2_high = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H2' and ep['alpha'] >= 0.30]
    h3_devs = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H3']
    h5_devs = [ep['max_lateral_dev'] for ep in headon if ep['persona'] == 'H5']

    plot_data = [h1_devs, h2_high, h2_mid, h2_low, h3_devs, h5_devs]
    labels = [
        "H1\n(alpha=0.50)",
        "H2 (Upper)\n(0.30 <= alpha <= 0.35)",
        "H2 (Mid)\n(0.20 <= alpha < 0.30)",
        "H2 (Low)\n(0.10 <= alpha < 0.20)",
        "H3\n(Blind)",
        "H5\n(SFM)"
    ]
    colors = ['#2ca02c', '#1f77b4', '#4a90e2', '#87ceeb', '#ff7f0e', '#d62728']

    bp = axes[0].boxplot(plot_data, labels=labels, patch_artist=True, showmeans=True,
                         meanprops={"marker":"o", "markerfacecolor":"red", "markeredgecolor":"red"})
    for patch, c in zip(bp['boxes'], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.7)

    axes[0].set_title("Lateral Deviation |Y| in Head-on Encounters", fontsize=12, fontweight='bold')
    axes[0].set_ylabel("Lateral Deviation (m)", fontsize=11)
    axes[0].grid(True, linestyle='--', alpha=0.5)

    # --- Plot 2: Reaction Timing / Distance at Deflection ---
    # Shows why H5 separates from H2 even if lateral deviation overlaps!
    h1_dist_def = [ep['trajectory'][np.argmax([p['lat_dev'] for p in ep['trajectory']])]['d_robot'] for ep in headon if ep['persona'] == 'H1']
    h2_dist_def = [ep['trajectory'][np.argmax([p['lat_dev'] for p in ep['trajectory']])]['d_robot'] for ep in headon if ep['persona'] == 'H2']
    h5_dist_def = [ep['trajectory'][np.argmax([p['lat_dev'] for p in ep['trajectory']])]['d_robot'] for ep in headon if ep['persona'] == 'H5']

    axes[1].hist([h1_dist_def, h2_dist_def, h5_dist_def], bins=15, label=["H1 (ORCA)", "H2 (ORCA)", "H5 (SFM)"],
                 color=['#2ca02c', '#1f77b4', '#d62728'], alpha=0.75, density=True)
    axes[1].set_title("Interaction Distance at Maximum Deflection", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("Distance to Robot at Peak Deflection (m)", fontsize=11)
    axes[1].set_ylabel("Probability Density", fontsize=11)
    axes[1].legend(loc="upper right")
    axes[1].grid(True, linestyle='--', alpha=0.5)

    # --- Plot 3: Sample 2D Trajectories of H1, H2, H3, H5 ---
    # Find representative episodes of head-on encounter
    for ep in headon:
        p = ep['persona']
        traj = ep['trajectory']
        xs = [pt['hx'] for pt in traj]
        ys = [pt['hy'] for pt in traj]
        if p == 'H1' and ep['id'] == 1:
            axes[2].plot(xs, ys, label="H1 ($\\alpha=0.50$)", color='#2ca02c', linewidth=2.5)
        elif p == 'H2' and ep['alpha'] >= 0.30 and ep['id'] == 501:
            axes[2].plot(xs, ys, label=f"H2 ($\\alpha={ep['alpha']:.2f}$)", color='#1f77b4', linewidth=2.5)
        elif p == 'H3' and ep['id'] == 1001:
            axes[2].plot(xs, ys, label="H3 (Blind - 0m dev)", color='#ff7f0e', linewidth=2.5, linestyle='--')
        elif p == 'H5' and ep['id'] == 2001:
            axes[2].plot(xs, ys, label="H5 (Smooth SFM)", color='#d62728', linewidth=2.5, linestyle=':')

    # Robot trajectory
    rep_robot_x = [pt['rx'] for pt in headon[0]['trajectory']]
    rep_robot_y = [pt['ry'] for pt in headon[0]['trajectory']]
    axes[2].plot(rep_robot_x, rep_robot_y, label="Robot (+X)", color='black', linewidth=2.0, linestyle='-.')
    axes[2].set_title("Representative 2D Trajectories (Head-on Encounter)", fontsize=12, fontweight='bold')
    axes[2].set_xlabel("X Position (m)", fontsize=11)
    axes[2].set_ylabel("Y Position (m)", fontsize=11)
    axes[2].legend(loc="upper left")
    axes[2].grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.savefig('experiments/pedestrian_persona_benchmark/h1_h5_detailed_distributions.png', dpi=300)
    plt.close()
    print("Saved detailed distributions to experiments/pedestrian_persona_benchmark/h1_h5_detailed_distributions.png")

if __name__ == '__main__':
    main()
