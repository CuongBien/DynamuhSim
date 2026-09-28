# Chạy lại và kiểm chứng Dataset V1.0

Tài liệu này dùng cho bước production, audit và freeze của PBL6. Chạy các lệnh từ thư mục `demo_1`:

```bash
cd /home/haduckien/nav_ws/src/custom_corridor/worlds/school_arena/demo_1
```

Entry point `produce_dataset_v1.py` tự nạp môi trường ROS từ `/home/haduckien/nav_ws/install/setup.bash` và chọn Fast DDS. Không cần `source` thủ công khi dùng entry point này.

## 1. Xem production đang chạy

```bash
systemctl --user status pbl6-dataset-v1-production --no-pager
tail -f dataset_v1.0/production.log
```

Xem số episode ở từng trạng thái mà không tác động đến batch:

```bash
python3 - <<'PY'
import csv
from collections import Counter
with open('dataset_v1.0/production_plan.csv', newline='', encoding='utf-8') as f:
    rows = list(csv.DictReader(f))
print('total:', len(rows))
print('status:', dict(Counter(row['status'] for row in rows)))
print('outcome PASS:', dict(Counter(row['outcome'] for row in rows if row['status'] == 'PASS')))
PY
```

`PASS` bao gồm các outcome navigation hợp lệ: `success`, `collision`, `timeout`, `stuck`, `nav_failure`. Lỗi Gazebo, HuNav, ROS, bag, builder hoặc QA là `FAIL_INFRASTRUCTURE`. Một hàng `RUNNING` có thể được retry bằng `--resume` nếu process dừng giữa episode.

## 2. Kiểm tra plan, duplicate và coverage

```bash
python3 - <<'PY'
from pathlib import Path
from collections import Counter
from dataset_production.plan import load_config, make_plan, read_plan, check_plan_identity
config = load_config(Path('configs/dataset_v1.yaml'))
rows = read_plan(config.output_dir / 'production_plan.csv')
check_plan_identity(rows, make_plan(config))
print('plan entries:', len(rows))
print('scenario families:', len(set(row['scenario_family'] for row in rows)))
print('densities:', dict(Counter(row['density'] for row in rows)))
print('duplicate episode IDs:', len(rows) - len(set(row['episode_id'] for row in rows)))
print('duplicate scenario+seed+density:', len(rows) - len(set((row['scenario_family'], row['seed'], row['density']) for row in rows)))
print('duplicate sampled configs:', len(rows) - len(set(row['scenario_sha256'] for row in rows)))
PY
```

Plan ban đầu có 605 episode cho S00–S23. Nếu một configuration lỗi hạ tầng hết lượt retry, production có thể thêm hàng với seed mới để bù coverage; vì vậy tổng hàng sau cùng có thể lớn hơn 605.

## 3. Chạy test mã nguồn

Các lệnh này không khởi động Gazebo:

```bash
python3 -m unittest discover -s scenario_generator/tests -v
python3 -m unittest discover -s dataset_runner/tests -v
python3 -m unittest discover -s dataset_production/tests -v
```

## 4. Tạo lại input và balance preview

Chỉ chạy khi service production không hoạt động. Lệnh giữ nguyên input hợp lệ đã có, kiểm tra chúng với plan và chỉ tạo input còn thiếu:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --prepare-only
```

Kết quả: `dataset_v1.0/episodes/`, `production_plan.csv`, `production_balance_preview.yaml`.

## 5. Chạy một episode hoặc resume batch

**Không chạy đồng thời với service production.** Xác nhận service đã dừng:

```bash
systemctl --user is-active pbl6-dataset-v1-production
```

Chạy một episode chưa PASS để kiểm chứng pipeline run → rosbag → structured samples → QA → manifest:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --resume --episode-id ep_000021 --max-episodes 1
```

`--resume` bỏ qua episode đã PASS; nếu `ep_000021` đã PASS, chọn một `episode_id` khác đang `PLANNED` hoặc còn lượt retry. Không sửa seed hay status bằng tay để chạy lại PASS. Chạy tiếp toàn bộ batch:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --resume
```

Để chạy bằng user service cố định đã cài trên máy này:

```bash
systemctl --user start pbl6-dataset-v1-production.service
systemctl --user status pbl6-dataset-v1-production.service --no-pager
```

Nếu chuyển workspace sang máy khác, cài unit một lần rồi dùng hai lệnh trên:

```bash
systemctl --user link "$PWD/systemd/pbl6-dataset-v1-production.service"
systemctl --user daemon-reload
```

Unit lưu tại `systemd/pbl6-dataset-v1-production.service`; preflight kiểm tra đúng image HuNav và dọn node mồ côi trước khi resume. `systemctl --user stop pbl6-dataset-v1-production.service` dừng batch; lần start sau sẽ tiếp tục từ registry. Không chạy CLI production foreground khi service đang active.

## 6. Audit và freeze

Sau khi batch dừng, tạo lại báo cáo audit:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --audit-only
```

Khi dataset chưa đủ hoặc còn lỗi integrity, lệnh in `AUDIT INCOMPLETE` và trả mã 1; đó là kết quả đúng. Xem `dataset_v1.0/dataset_audit.yaml`, `dataset_audit.csv`, `dataset_summary.yaml` và `scenario_coverage.csv` để biết phần thiếu. Mỗi episode PASS cần `execution.yaml`, bag đọc được và có `/odom`, `/scan`, `/robot/camera/image_raw`, `structured/samples.csv` với timestamp tăng, cùng `qa.yaml` PASS.

Khi audit `COMPLETE`, freeze thủ công hoặc kiểm chứng freeze tự động của batch:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --freeze
```

Kiểm tra đầu ra:

```bash
ls dataset_v1.0/splits/train.txt dataset_v1.0/splits/val.txt dataset_v1.0/splits/test.txt
cat dataset_v1.0/dataset_fingerprint.yaml
```

Chạy lại `--freeze` trên dataset đã đóng băng chỉ để **xác minh** assignment và fingerprint; chương trình từ chối nếu manifest, plan, config hoặc split đã đổi. Không sửa hay regenerate split sau freeze.

## File chính

| File/thư mục | Nội dung |
| --- | --- |
| `configs/dataset_v1.yaml` | target, seed, retry, backend và split seed |
| `dataset_v1.0/production_plan.csv` | identity, seed, density, status, outcome và số lần thử |
| `dataset_v1.0/episodes/<episode_id>/` | input, execution, raw bag, structured RGB/CSV, QA |
| `dataset_v1.0/manifest.csv` | các episode được chấp nhận |
| `dataset_v1.0/dataset_audit.yaml` | thống kê và lỗi integrity |
| `dataset_v1.0/scenario_coverage.csv` | coverage theo scenario |
| `dataset_v1.0/splits/` | train/val/test theo episode |
| `dataset_v1.0/dataset_fingerprint.yaml` | SHA-256 của các thành phần đã freeze |
