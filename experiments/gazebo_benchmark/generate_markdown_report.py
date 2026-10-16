import json
import os

with open("experiments/gazebo_benchmark/a3_3_4policies_results.json") as f:
    res = json.load(f)

for persona in ["H1", "H3"]:
    print(f"\n### 1. Phân bố hành động Robot ({persona})\n")
    print("| Chính sách | Kênh lệnh | $v_x$ (m/s) [Mean ± Std] | $v_x$ [p5, p95] | $v_y$ (m/s) [Mean ± Std] | $v_y$ [p5, p95] | $\\omega$ (rad/s) [Mean ± Std] | $\\omega$ [p5, p95] |")
    print("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |")
    for pol in ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]:
        d = res["summary_by_persona"][persona][pol]
        for tag, key in [("Dự định (Intended)", "actions_intended"), ("Nhiễu $\\eta$ (Noise)", "actions_noise"), ("Áp dụng (Applied)", "actions_applied")]:
            ai = d[key]
            vx_m, vx_s = ai["vx"]["mean"], ai["vx"]["std"]
            vx_p5, vx_p95 = ai["vx"]["p5"], ai["vx"]["p95"]
            vy_m, vy_s = ai["vy"]["mean"], ai["vy"]["std"]
            vy_p5, vy_p95 = ai["vy"]["p5"], ai["vy"]["p95"]
            wz_m, wz_s = ai["wz"]["mean"], ai["wz"]["std"]
            wz_p5, wz_p95 = ai["wz"]["p5"], ai["wz"]["p95"]
            pol_col = f"**{pol}**" if tag.startswith("Dự định") else ""
            print(f"| {pol_col} | {tag} | {vx_m:.3f} ± {vx_s:.3f} | [{vx_p5:.3f}, {vx_p95:.3f}] | {vy_m:.3f} ± {vy_s:.3f} | [{vy_p5:.3f}, {vy_p95:.3f}] | {wz_m:.3f} ± {wz_s:.3f} | [{wz_p5:.3f}, {wz_p95:.3f}] |")

    print(f"\n### 2. Hành vi Robot & An toàn va chạm ({persona})\n")
    print("| Chính sách | Tới đích (%) | Va chạm người (%) | Sượt (<2cm) (%) | Chạm tường (%) | $d_{clear}$ min (m, med) | Lệch ngang max (m) [med / max] | Thời gian $|v_y|>0.1$ (%) | Dừng hẳn $\\ge 1$s (%) | Đóng băng $\\ge 3$s (%) |")
    print("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
    for pol in ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]:
        d = res["summary_by_persona"][persona][pol]
        dmin = f"{d['dmin_clear_m_med']:.4f}" if d['dmin_clear_m_med'] is not None else "N/A"
        print(f"| **{pol}** | {d['goal_reach_rate_pct']}% | {d['collision_human_rate_pct']}% | {d['near_miss_rate_pct']}% | {d['wall_contact_rate_pct']}% | {dmin} | {d['max_lateral_dev_m_med']:.3f} / {d['max_lateral_dev_m_max']:.3f} | {d['ratio_time_vy_gt_01_pct']}% | {d['stopped_fully_1s_rate_pct']}% | {d['freezing_or_stuck_3s_rate_pct']}% |")

    print(f"\n### 3. Phản ứng của Người trong cửa sổ tương tác ($d < 4\\text{{ m}}$) ({persona})\n")
    print("| Chính sách | Độ lệch ngang max người (m) | Tỉ lệ bước bị kẹp (%) | Mức giảm tốc người (m/s) | $\\Delta t$ phản ứng người |")
    print("| :--- | :---: | :---: | :---: | :---: |")
    for pol in ["P1", "P2", "P3a", "P3b", "P3c", "P3d", "P4"]:
        d = res["summary_by_persona"][persona][pol]
        delta_str = "Không áp dụng"
        print(f"| **{pol}** | {d['human_max_lat_dev_m_mean']:.4f} m | {d['human_clamped_pct_mean']:.1f}% | {d['human_decel_mps_mean']:.3f} m/s | {delta_str} |")
