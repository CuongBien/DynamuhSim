"""
Exhaustive Multi-Environment Benchmark for Depth Person Perception & Tracking (Direction 2).
Evaluates 30 scenarios across 4 diverse environments:
  1. Hospital Easy (6 scenarios: oncoming walker, door walker, bench follower, benches, doorways, long range)
  2. Hospital Medium (8 scenarios: head-on, room crossing, same direction, standing group, trolley pusher, trolley obstacle, pocket yielding, wait-then-cross)
  3. Hospital Hard (8 scenarios: sudden door exit, door entry, walking group of 3, multi-crossing, multi-trolleys, dense 4-ped crowd, partition occlusion, stacked crates)
  4. School Arena (8 scenarios: head-on student, locker wall walker, 90-deg corner turn, door exit, student pair, classroom desk/chair, cluttered hallway, vertical crossing)

Metrics evaluated:
  - Confusion Matrix: TP, FP, TN, FN
  - Precision, Recall, Accuracy, F1-Score
  - Per-Environment breakdowns & Latency (ms)
"""
import time
import numpy as np
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
BENCHMARK = HERE.parent
CLASSIFIER_PATH = BENCHMARK / "depth_person_classifier"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(CLASSIFIER_PATH) not in sys.path:
    sys.path.insert(0, str(CLASSIFIER_PATH))

from integrated_pipeline import EndToEndPerceptionPipeline
from generalized_detector import adaptive_3d_clustering, GeneralizedPersonDetector


def make_human(center_xyz, radius=0.22, height=1.70, num_points=350, occlusion=None, rng=None):
    if rng is None:
        rng = np.random.default_rng(int(abs(center_xyz[0] * 100 + center_xyz[1] * 10)) % 10000 + 1)
    theta = rng.uniform(0, 2 * np.pi, num_points)
    r = radius * np.sqrt(rng.uniform(0.1, 1.0, num_points))
    z = rng.uniform(0.0, height, num_points)
    x = center_xyz[0] + r * np.cos(theta)
    y = center_xyz[1] + r * np.sin(theta)
    z = center_xyz[2] + z
    pts = np.column_stack([x, y, z])

    if occlusion == 'lower_body':
        pts = pts[pts[:, 2] >= (center_xyz[2] + 0.40 * height)]
    elif occlusion == 'side':
        pts = pts[pts[:, 1] >= center_xyz[1] - 0.05]
    elif occlusion == 'half_vertical':
        pts = pts[pts[:, 0] <= center_xyz[0] + 0.05]
    return pts


def make_wall(x_center, y_span, z_span, num_points=400, rng=None):
    if rng is None:
        rng = np.random.default_rng(123)
    y = rng.uniform(y_span[0], y_span[1], num_points)
    z = rng.uniform(z_span[0], z_span[1], num_points)
    x = x_center + rng.normal(0, 0.02, num_points)
    return np.column_stack([x, y, z])


def make_bench(center_xyz, length=1.6, width=0.45, height=0.45, num_points=300, rng=None):
    """Hospital corridor waiting bench (negative object)."""
    if rng is None:
        rng = np.random.default_rng(234)
    x = rng.uniform(-width / 2, width / 2, num_points) + center_xyz[0]
    y = rng.uniform(-length / 2, length / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, height, num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def make_trolley(center_xyz, size=(0.7, 0.5, 0.85), num_points=250, rng=None):
    """Medical cart / supply trolley (negative object)."""
    if rng is None:
        rng = np.random.default_rng(345)
    x = rng.uniform(-size[0] / 2, size[0] / 2, num_points) + center_xyz[0]
    y = rng.uniform(-size[1] / 2, size[1] / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, size[2], num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def make_school_desk(center_xyz, size=(0.6, 1.2, 0.75), num_points=300, rng=None):
    """Classroom study desk with legs (negative object)."""
    if rng is None:
        rng = np.random.default_rng(456)
    x = rng.uniform(-size[0] / 2, size[0] / 2, num_points) + center_xyz[0]
    y = rng.uniform(-size[1] / 2, size[1] / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, size[2], num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def make_crates(center_xyz, size=(0.6, 0.6, 1.0), num_points=250, rng=None):
    """Stacked cardboard / plastic boxes (negative object)."""
    if rng is None:
        rng = np.random.default_rng(567)
    x = rng.uniform(-size[0] / 2, size[0] / 2, num_points) + center_xyz[0]
    y = rng.uniform(-size[1] / 2, size[1] / 2, num_points) + center_xyz[1]
    z = rng.uniform(0.05, size[2], num_points) + center_xyz[2]
    return np.column_stack([x, y, z])


def build_all_scenarios():
    scenarios = {}

    # =========================================================================
    # 1. HOSPITAL EASY (6 Scenarios)
    # =========================================================================
    scenarios["Hospital Easy"] = [
        {
            'id': 'EASY_01',
            'name': 'Oncoming corridor walker (hospital_walking_human) at 3.5m',
            'points': make_human([3.5, 0.0, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'EASY_02',
            'name': 'Walker approaching South Room 6 doorway (hospital_walking_human_2)',
            'points': np.vstack([make_human([2.8, -0.6, 0.08]), make_wall(2.8, [-2.5, -1.2], [0.05, 2.0])]),
            'expected_p': 1
        },
        {
            'id': 'EASY_03',
            'name': 'Waiting follower standing near South Room 2 bench (hospital_walking_human_3)',
            'points': np.vstack([make_human([2.2, 0.4, 0.08]), make_bench([2.2, 0.9, 0.0])]),
            'expected_p': 1
        },
        {
            'id': 'EASY_04',
            'name': 'Negative check: Corridor waiting bench (1.6m x 0.45m x 0.45m)',
            'points': make_bench([2.5, 0.0, 0.0]),
            'expected_p': 0
        },
        {
            'id': 'EASY_05',
            'name': 'Negative check: Open doorway frame (1.2m opening)',
            'points': np.vstack([make_wall(2.0, [-2.0, -0.6], [0.05, 2.0]), make_wall(2.0, [0.6, 2.0], [0.05, 2.0])]),
            'expected_p': 0
        },
        {
            'id': 'EASY_06',
            'name': 'Far-range oncoming pedestrian at 4.6m near sensor edge',
            'points': make_human([4.6, 0.1, 0.08], num_points=220),
            'expected_p': 1
        }
    ]

    # =========================================================================
    # 2. HOSPITAL MEDIUM (8 Scenarios)
    # =========================================================================
    scenarios["Hospital Medium"] = [
        {
            'id': 'MED_01',
            'name': 'Direct head-on walker in corridor (medium_walker_02_head_on) at 2.8m',
            'points': make_human([2.8, 0.0, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'MED_02',
            'name': 'Room crossing walker (medium_walker_03) cutting perpendicular at 2.4m',
            'points': make_human([2.4, -0.2, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'MED_03',
            'name': 'Same-direction walking pedestrian (medium_walker_04) ahead at 3.2m',
            'points': make_human([3.2, 0.2, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'MED_04',
            'name': 'Standing group of 2 people talking (medium_standing_room_04)',
            'points': np.vstack([make_human([2.3, -0.35, 0.08]), make_human([2.3, 0.35, 0.08])]),
            'expected_p': 2
        },
        {
            'id': 'MED_05',
            'name': 'Person pushing trolley (medium_trolley_pusher): Person + Trolley 0.6m ahead',
            'points': np.vstack([make_human([2.8, 0.0, 0.08]), make_trolley([2.2, 0.0, 0.0])]),
            'expected_p': 1
        },
        {
            'id': 'MED_06',
            'name': 'Negative check: Independent medical trolley in hallway',
            'points': make_trolley([2.2, 0.0, 0.0]),
            'expected_p': 0
        },
        {
            'id': 'MED_07',
            'name': 'Yielding walker stepping into room pocket (medium_walker_08_yielding)',
            'points': np.vstack([make_human([2.5, 0.8, 0.08]), make_wall(2.5, [-1.5, 0.3], [0.05, 2.0])]),
            'expected_p': 1
        },
        {
            'id': 'MED_08',
            'name': 'Wait-then-cross walker emerging from Room 9 door with 30% side occlusion',
            'points': make_human([2.6, -0.3, 0.08], occlusion='side'),
            'expected_p': 1
        }
    ]

    # =========================================================================
    # 3. HOSPITAL HARD (8 Scenarios)
    # =========================================================================
    scenarios["Hospital Hard"] = [
        {
            'id': 'HARD_01',
            'name': 'Sudden door exit walker emerging into corridor (hard_walker_door_exit)',
            'points': np.vstack([make_human([2.2, -0.4, 0.08]), make_wall(2.2, [-2.5, -0.9], [0.05, 2.0])]),
            'expected_p': 1
        },
        {
            'id': 'HARD_02',
            'name': 'Walker turning into door entrance (hard_walker_door_entry)',
            'points': make_human([2.5, 0.3, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'HARD_03',
            'name': 'Group of 3 pedestrians walking abreast (hard_walker_group)',
            'points': np.vstack([
                make_human([3.0, -0.65, 0.08]),
                make_human([3.0, 0.0, 0.08]),
                make_human([3.0, 0.65, 0.08])
            ]),
            'expected_p': 3
        },
        {
            'id': 'HARD_04',
            'name': 'Multi-crossing walkers (2 pedestrians passing each other perpendicularly)',
            'points': np.vstack([make_human([2.2, -0.5, 0.08]), make_human([2.7, 0.4, 0.08])]),
            'expected_p': 2
        },
        {
            'id': 'HARD_05',
            'name': 'Cluttered corridor: 2 trolley pushers encountering in hallway',
            'points': np.vstack([
                make_human([2.2, -0.4, 0.08]), make_trolley([1.7, -0.4, 0.0]),
                make_human([3.6, 0.4, 0.08]), make_trolley([3.1, 0.4, 0.0])
            ]),
            'expected_p': 2
        },
        {
            'id': 'HARD_06',
            'name': 'Dense crowd corridor: 4 pedestrians staggered at 1.5m, 2.4m, 3.2m, 4.0m',
            'points': np.vstack([
                make_human([1.6, -0.4, 0.08]),
                make_human([2.4, 0.3, 0.08]),
                make_human([3.2, -0.3, 0.08]),
                make_human([4.0, 0.2, 0.08])
            ]),
            'expected_p': 4
        },
        {
            'id': 'HARD_07',
            'name': 'Heavy occlusion: Pedestrian behind mobile partition (50% lower occluded)',
            'points': np.vstack([
                make_human([2.4, 0.0, 0.08], occlusion='lower_body'),
                make_crates([2.4, 0.0, 0.0], size=(0.8, 0.8, 0.7))
            ]),
            'expected_p': 1
        },
        {
            'id': 'HARD_08',
            'name': 'Negative check: Stacked supply boxes/crates in corridor corner',
            'points': make_crates([2.2, 0.3, 0.0], size=(0.6, 0.6, 1.1)),
            'expected_p': 0
        }
    ]

    # =========================================================================
    # 4. SCHOOL ARENA (8 Scenarios)
    # =========================================================================
    scenarios["School Arena"] = [
        {
            'id': 'SCHOOL_01',
            'name': 'Head-on student walking in main hallway (student_head_on_A) at 2.6m',
            'points': make_human([2.6, 0.0, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'SCHOOL_02',
            'name': 'Student walking opposite direction along locker wall (student_opposite_A)',
            'points': np.vstack([make_human([2.8, -0.45, 0.08]), make_wall(2.8, [-2.0, -0.85], [0.05, 2.0])]),
            'expected_p': 1
        },
        {
            'id': 'SCHOOL_03',
            'name': 'Student navigating 90-degree corner blind spot (student_corner_B)',
            'points': np.vstack([
                make_human([2.2, 0.5, 0.08]),
                make_wall(2.2, [-1.5, 0.1], [0.05, 2.0]),
                make_wall(3.2, [0.1, 1.5], [0.05, 2.0])
            ]),
            'expected_p': 1
        },
        {
            'id': 'SCHOOL_04',
            'name': 'Student exiting classroom door into active corridor (student_door_exit)',
            'points': make_human([2.3, -0.2, 0.08]),
            'expected_p': 1
        },
        {
            'id': 'SCHOOL_05',
            'name': 'Student pair walking together to class (group_id = 1)',
            'points': np.vstack([make_human([2.7, -0.3, 0.08]), make_human([2.7, 0.3, 0.08])]),
            'expected_p': 2
        },
        {
            'id': 'SCHOOL_06',
            'name': 'Negative check: Classroom study desk (1.2m x 0.6m x 0.75m)',
            'points': make_school_desk([2.2, 0.0, 0.0]),
            'expected_p': 0
        },
        {
            'id': 'SCHOOL_07',
            'name': 'Cluttered school hall: Student walking past study desk and chair',
            'points': np.vstack([make_human([2.6, 0.45, 0.08]), make_school_desk([2.6, -0.35, 0.0])]),
            'expected_p': 1
        },
        {
            'id': 'SCHOOL_08',
            'name': 'Vertical crossing student in corridor intersection (student_vertical_cross)',
            'points': make_human([3.0, 0.1, 0.08]),
            'expected_p': 1
        }
    ]

    return scenarios


SYNTHETIC_PERSON_CENTERS = {'EASY_01': [[3.5, 0.0]], 'EASY_02': [[2.8, -0.6]], 'EASY_03': [[2.2, 0.4]], 'EASY_04': [], 'EASY_05': [], 'EASY_06': [[4.6, 0.1]], 'MED_01': [[2.8, 0.0]], 'MED_02': [[2.4, -0.2]], 'MED_03': [[3.2, 0.2]], 'MED_04': [[2.3, -0.35], [2.3, 0.35]], 'MED_05': [[2.8, 0.0]], 'MED_06': [], 'MED_07': [[2.5, 0.8]], 'MED_08': [[2.6, -0.3]], 'HARD_01': [[2.2, -0.4]], 'HARD_02': [[2.5, 0.3]], 'HARD_03': [[3.0, -0.65], [3.0, 0.0], [3.0, 0.65]], 'HARD_04': [[2.2, -0.5], [2.7, 0.4]], 'HARD_05': [[2.2, -0.4], [3.6, 0.4]], 'HARD_06': [[1.6, -0.4], [2.4, 0.3], [3.2, -0.3], [4.0, 0.2]], 'HARD_07': [[2.4, 0.0]], 'HARD_08': [], 'SCHOOL_01': [[2.6, 0.0]], 'SCHOOL_02': [[2.8, -0.45]], 'SCHOOL_03': [[2.2, 0.5]], 'SCHOOL_04': [[2.3, -0.2]], 'SCHOOL_05': [[2.7, -0.3], [2.7, 0.3]], 'SCHOOL_06': [], 'SCHOOL_07': [[2.6, 0.45]], 'SCHOOL_08': [[3.0, 0.1]]}

def run_multi_environment_benchmark():
    from spatial_metrics import match_positions
    results = []
    for environment, scenarios in build_all_scenarios().items():
        for scene in scenarios:
            pipeline = EndToEndPerceptionPipeline()
            start = time.perf_counter()
            tracks = pipeline.process_point_cloud(100_000_000, scene['points'], np.zeros(3))
            metrics = match_positions([t['xy_base'] for t in tracks], SYNTHETIC_PERSON_CENTERS[scene['id']])
            results.append(dict(environment=environment, id=scene['id'], latency_ms=(time.perf_counter()-start)*1000,
                                **metrics))
    print(__import__('json').dumps(dict(scope='synthetic geometry only; spatial matching; fresh tracker per scene',
                                      results=results, training_authorized=False), indent=2))
    return results

if __name__ == '__main__':
    run_multi_environment_benchmark()
