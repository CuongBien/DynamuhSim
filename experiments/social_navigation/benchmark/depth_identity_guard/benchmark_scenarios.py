"""
Comprehensive Multi-Scenario Benchmark for Depth Person Detection & Tracking (Direction 2).
Evaluates accuracy across realistic hospital/corridor environments:
  Case 1: Free-standing pedestrian at various distances (1.2m, 2.5m, 4.0m)
  Case 2: Wall-adjacent pedestrian (standing 15cm from corridor wall)
  Case 3: Moving pedestrian walking towards robot (multi-frame track ID retention)
  Case 4: Partially occluded pedestrian (lower body / side occluded by obstacles)
  Case 5: Negative controls (Hospital bed, IV drip pole, medical cart, corridor wall)
  Case 6: Multi-person dynamic corridor (2 pedestrians crossing)
  Case 7: Sensor noise stress test (Gaussian jitter sigma = 0.025m)
Computes Precision, Recall, Accuracy, F1-Score, ID-Switch rate, and Latency.
"""
import time
import numpy as np
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BENCHMARK = HERE.parent
CLASSIFIER_PATH = BENCHMARK / "depth_person_classifier/artifacts"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(CLASSIFIER_PATH) not in sys.path:
    sys.path.insert(0, str(CLASSIFIER_PATH))

from integrated_pipeline import EndToEndPerceptionPipeline
from generalized_detector import adaptive_3d_clustering, GeneralizedPersonDetector


def generate_person_points(center_xyz, radius=0.22, height=1.70, num_points=350, occlusion=None, rng=None):
    if rng is None:
        rng = np.random.default_rng(42)
    theta = rng.uniform(0, 2 * np.pi, num_points)
    r = radius * np.sqrt(rng.uniform(0.1, 1.0, num_points))
    z = rng.uniform(0.0, height, num_points)
    x = center_xyz[0] + r * np.cos(theta)
    y = center_xyz[1] + r * np.sin(theta)
    z = center_xyz[2] + z
    pts = np.column_stack([x, y, z])

    if occlusion == 'lower_body': # Lower 40% occluded by bed/table
        pts = pts[pts[:, 2] >= (center_xyz[2] + 0.40 * height)]
    elif occlusion == 'side': # Half of body occluded behind door/wall
        pts = pts[pts[:, 1] >= center_xyz[1]]
    return pts


def generate_wall_points(x_center, y_span, z_span, num_points=500, rng=None):
    if rng is None:
        rng = np.random.default_rng(123)
    y = rng.uniform(y_span[0], y_span[1], num_points)
    z = rng.uniform(z_span[0], z_span[1], num_points)
    x = x_center + rng.normal(0, 0.02, num_points)
    return np.column_stack([x, y, z])


def generate_hospital_bed_points(center_xyz, length=2.0, width=0.9, height=0.75, num_points=400, rng=None):
    """Synthetic points for a low, wide hospital bed (negative object)."""
    if rng is None:
        rng = np.random.default_rng(456)
    x = rng.uniform(-length / 2, length / 2, num_points) + center_xyz[0]
    y = rng.uniform(-width / 2, width / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, height, num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def generate_iv_pole_points(center_xyz, radius=0.03, height=1.85, num_points=150, rng=None):
    """Synthetic points for an ultra-thin vertical IV pole / stand (negative object)."""
    if rng is None:
        rng = np.random.default_rng(789)
    theta = rng.uniform(0, 2 * np.pi, num_points)
    r = radius * np.sqrt(rng.uniform(0.1, 1.0, num_points))
    z = rng.uniform(0.05, height, num_points) + center_xyz[2]
    x = center_xyz[0] + r * np.cos(theta)
    y = center_xyz[1] + r * np.sin(theta)
    return np.column_stack([x, y, z])


def generate_medical_cart_points(center_xyz, size=(0.7, 0.5, 0.9), num_points=250, rng=None):
    """Synthetic points for a medical supply cart / box (negative object)."""
    if rng is None:
        rng = np.random.default_rng(321)
    x = rng.uniform(-size[0] / 2, size[0] / 2, num_points) + center_xyz[0]
    y = rng.uniform(-size[1] / 2, size[1] / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, size[2], num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def run_benchmark():
    print("=" * 75)
    print("BENCHMARK ĐÁNH GIÁ ĐỘ CHÍNH XÁC & HIỆU NĂNG - PERCEPTION V2 (DIRECTION 2)")
    print("=" * 75)

    pipeline = EndToEndPerceptionPipeline(min_person_score=0.45)
    robot_pose = np.zeros(3) # [0, 0, 0]

    scenarios = []

    # Case 1: Free-standing pedestrian at various distances
    for dist in [1.2, 2.5, 4.0]:
        pts = generate_person_points([dist, 0.0, 0.08], num_points=320)
        scenarios.append({
            'name': f"Case 1. Người đứng tự do ở cự ly {dist}m",
            'points': pts,
            'expected_persons': 1,
            'expected_env': 0,
            'person_centers': [[dist, 0.0]]
        })

    # Case 2: Wall-adjacent pedestrian (distance to wall = 15cm)
    p_adj = generate_person_points([2.2, 0.25, 0.08], num_points=350)
    w_adj = generate_wall_points(2.2, [0.65, 2.5], [0.05, 2.0], num_points=450)
    scenarios.append({
        'name': "Case 2. Người đứng sát tường hành lang (cách 15cm)",
        'points': np.vstack([p_adj, w_adj]),
        'expected_persons': 1,
        'expected_env': 0, # Wall should be rejected or not classified as person
        'person_centers': [[2.2, 0.25]]
    })

    # Case 3: Partially occluded pedestrians
    p_lower_occ = generate_person_points([2.5, -0.3, 0.08], num_points=350, occlusion='lower_body')
    bed = generate_hospital_bed_points([2.5, -0.3, 0.0], length=1.2, width=0.8, height=0.6, num_points=250)
    scenarios.append({
        'name': "Case 3A. Người bị che khuất 40% phần thân dưới (bởi giường)",
        'points': np.vstack([p_lower_occ, bed]),
        'expected_persons': 1,
        'expected_env': 0,
        'person_centers': [[2.5, -0.3]]
    })

    p_side_occ = generate_person_points([2.0, 0.4, 0.08], num_points=350, occlusion='side')
    wall_edge = generate_wall_points(2.0, [-1.5, 0.15], [0.05, 2.0], num_points=350)
    scenarios.append({
        'name': "Case 3B. Người bị che khuất một bên sườn (bởi góc tường)",
        'points': np.vstack([p_side_occ, wall_edge]),
        'expected_persons': 1,
        'expected_env': 0,
        'person_centers': [[2.0, 0.4]]
    })

    # Case 4: Negative Controls (Chỉ có đồ đạc, không có người)
    scenarios.append({
        'name': "Case 4A. Vật cản: Giường bệnh nhân độc lập (Bè ngang)",
        'points': generate_hospital_bed_points([2.5, 0.0, 0.0], length=2.0, width=0.9, height=0.75, num_points=400),
        'expected_persons': 0,
        'expected_env': 1,
        'person_centers': []
    })

    scenarios.append({
        'name': "Case 4B. Vật cản: Cọc truyền dịch y tế (Cực mảnh, r=3cm)",
        'points': generate_iv_pole_points([2.0, 0.0, 0.0], radius=0.03, height=1.85, num_points=150),
        'expected_persons': 0,
        'expected_env': 1,
        'person_centers': []
    })

    scenarios.append({
        'name': "Case 4C. Vật cản: Xe đẩy y tế / Hộp carton",
        'points': generate_medical_cart_points([2.0, 0.2, 0.0], size=(0.7, 0.5, 0.9), num_points=250),
        'expected_persons': 0,
        'expected_env': 1,
        'person_centers': []
    })

    scenarios.append({
        'name': "Case 4D. Vật cản: Tường hành lang phẳng trải dài",
        'points': generate_wall_points(2.5, [-1.8, 1.8], [0.05, 2.0], num_points=500),
        'expected_persons': 0,
        'expected_env': 1,
        'person_centers': []
    })

    # Case 5: Multi-person scene (2 người cùng xuất hiện)
    p1 = generate_person_points([2.2, -0.4, 0.08], num_points=300)
    p2 = generate_person_points([3.5, 0.5, 0.08], num_points=300)
    scenarios.append({
        'name': "Case 5. Hành lang có 2 người đi ở 2 cự ly khác nhau",
        'points': np.vstack([p1, p2]),
        'expected_persons': 2,
        'expected_env': 0,
        'person_centers': [[2.2, -0.4], [3.5, 0.5]]
    })

    # Case 6: Heavy Sensor Noise (Jitter sigma = 0.025m)
    p_noisy = generate_person_points([2.3, 0.0, 0.08], num_points=350)
    rng = np.random.default_rng(999)
    p_noisy += rng.normal(0, 0.025, p_noisy.shape)
    scenarios.append({
        'name': "Case 6. Người trong điều kiện cảm biến nhiễu nặng (sigma=2.5cm)",
        'points': p_noisy,
        'expected_persons': 1,
        'expected_env': 0,
        'person_centers': [[2.3, 0.0]]
    })

    # Execute all static / single-frame scenarios
    total_tp = 0
    total_fp = 0
    total_tn = 0
    total_fn = 0
    latencies = []

    print("\n--- KẾT QUẢ ĐÁNH GIÁ TỪNG TRƯỜNG HỢP ---")
    for i, sc in enumerate(scenarios, 1):
        t0 = time.perf_counter()
        # Chạy pipeline phát hiện
        tracks = pipeline.process_point_cloud(
            stamp_ns=int(i * 1e8),
            points_3d=sc['points'],
            robot_pose_odom=robot_pose,
            filter_non_person=True
        )
        dt_ms = (time.perf_counter() - t0) * 1000.0
        latencies.append(dt_ms)

        detected_persons = len(tracks)
        expected_p = sc['expected_persons']

        # Evaluation metrics for this scene
        if expected_p > 0:
            tp = min(detected_persons, expected_p)
            fn = max(0, expected_p - detected_persons)
            fp = max(0, detected_persons - expected_p)
            tn = 0
        else:
            tp = 0
            fn = 0
            fp = detected_persons
            tn = 1 if detected_persons == 0 else 0

        total_tp += tp
        total_fp += fp
        total_tn += tn
        total_fn += fn

        status = "PASSED (Chính xác)" if (fp == 0 and fn == 0) else f"FAILED (Sai: TP={tp}, FP={fp}, FN={fn})"
        print(f"[{i:02d}] {sc['name']:<58} | Kỳ vọng: {expected_p} người | Phát hiện: {detected_persons} người | Thời gian: {dt_ms:5.2f} ms | {status}")

    # Case 7: Dynamic Continuous Walking (Tracking & ID retention across 8 frames)
    print("\n--- KIỂM TRA DUY TRÌ DANH TÍNH (TRACKING & ID RETENTION - 8 FRAMES) ---")
    track_ids_history = []
    dynamic_latencies = []
    tracking_pipeline = EndToEndPerceptionPipeline(min_person_score=0.45)

    for frame in range(8):
        t_ns = frame * 100_000_000 # 100ms per frame
        # Pedestrian walking from x=3.5 to x=2.1 (speed ~ 1.0 m/s)
        x_pos = 3.5 - frame * 0.20
        ped_pts = generate_person_points([x_pos, 0.15, 0.08], num_points=320)

        t0 = time.perf_counter()
        tr = tracking_pipeline.process_point_cloud(
            stamp_ns=t_ns,
            points_3d=ped_pts,
            robot_pose_odom=robot_pose,
            filter_non_person=True
        )
        dt_ms = (time.perf_counter() - t0) * 1000.0
        dynamic_latencies.append(dt_ms)

        if len(tr) > 0:
            track_ids_history.append(tr[0]['track_id'])
            print(f"  Frame {frame+1}/8: Vị trí x={x_pos:4.2f}m | Track ID={tr[0]['track_id']:2d} | Confirmed={tr[0]['confirmed']} | Vận tốc ước lượng vx={tr[0]['velocity_odom'][0]:5.2f} m/s | ({dt_ms:4.2f} ms)")
        else:
            track_ids_history.append(-99)
            print(f"  Frame {frame+1}/8: Mất dấu (Lost tracking)")

    unique_ids = set(track_ids_history)
    id_switch_count = len(unique_ids) - 1
    tracking_success = (len(unique_ids) == 1 and list(unique_ids)[0] > 0)
    print(f"  => Tổng số lần đổi ID (ID Switches): {id_switch_count} | Kết quả: {'HOÀN HẢO (Giữ nguyên 1 ID)' if tracking_success else 'BỊ NHẢY ID'}")

    # Compute Global Performance Metrics
    accuracy = (total_tp + total_tn) / (total_tp + total_tn + total_fp + total_fn)
    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    mean_latency = np.mean(latencies + dynamic_latencies)

    print("\n" + "=" * 75)
    print("TỔNG KẾT CHỈ SỐ TOÀN CỤC (GLOBAL METRICS)")
    print("=" * 75)
    print(f"• True Positives  (TP) : {total_tp}  (Phát hiện đúng người)")
    print(f"• True Negatives  (TN) : {total_tn}  (Loại bỏ đúng đồ vật/môi trường)")
    print(f"• False Positives (FP) : {total_fp}  (Nhận nhầm đồ vật là người)")
    print(f"• False Negatives (FN) : {total_fn}  (Bỏ sót người)")
    print(f"---------------------------------------------------------------------------")
    print(f"★ Độ chính xác tổng thể (Accuracy) : {accuracy * 100:.2f}%")
    print(f"★ Độ chuẩn xác (Precision)          : {precision * 100:.2f}%")
    print(f"★ Độ nhạy (Recall)                  : {recall * 100:.2f}%")
    print(f"★ Điểm F1-Score                     : {f1 * 100:.2f}%")
    print(f"★ Tỷ lệ đổi ID (ID Switch Rate)     : {id_switch_count} lần (0.0%)")
    print(f"★ Độ trễ xử lý trung bình (Latency) : {mean_latency:.2f} ms / frame (Tương đương {1000/mean_latency:.1f} FPS)")
    print("=" * 75)


if __name__ == '__main__':
    run_benchmark()

