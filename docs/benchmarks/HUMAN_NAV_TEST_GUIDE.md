# Chạy thử Human Navigation và đọc metrics

Dùng hướng dẫn này để chạy một nhóm nhỏ trên MuJoCo, đối chiếu cùng điều kiện giữa các backend và đọc benchmark 900 lượt đã khóa. Các ví dụ ghi output vào thư mục mới dưới `runs/`. [Gói tái lập 08F](REPRODUCIBILITY_PACKAGE.md) · [Giao thức QA](DATA_QA_PROTOCOL.md)

## 1. Chọn cấu hình

| Cần chọn | Cấu hình thực thi |
| --- | --- |
| Backend người | `orca`, `sfm_jupedsim`, `avm_jupedsim`, `csm_jupedsim`. Tham số từng backend nằm trong [configs/humans](../../configs/humans/). |
| Card | Dùng ID đầy đủ trong bảng dưới. Hình học, bề rộng, spawn, tuyến, mật độ và lịch lệnh đã khóa trong [bindings.yaml](../../configs/benchmarks/bindings.yaml). |
| Level và seed | `easy`, `critical`, `hard`. Vòng 0 có 48 slot Easy với seed 1000–1001. Phase 1 có 540 slot lũy kế và dùng seed 1000–1004. Phase 2 có 900 slot lũy kế, thêm seed 1005–1019 chỉ ở Critical Advance. |
| Hành động robot | `advance` có ở cả sáu Card. `stop`, `yield`, `reverse` chỉ có ở FA, ND, IN Critical với seed 1000–1004. |
| Robot và đồng hồ | Benchmark chọn `mecanum_robot` và tắt cảm biến. Physics 2 ms, robot 50 ms, người 100 ms, GT xuất 10 Hz. Xem [cấu hình robot](../../configs/robots/mecanum_robot.yaml) và [mô phỏng](../../configs/simulation/mujoco.yaml). |
| Smoke kỹ thuật | Một lượt ORCA, seed 11, ngân sách 0,5 s. Dùng để kiểm đường chạy và ghi log. Timeout ở lượt này là kết quả dự kiến. [smoke.yaml](../../configs/benchmarks/smoke.yaml) |

| Card ID | Tình huống | Bề rộng Easy / Critical / Hard (m) |
| --- | --- | --- |
| `FA_Corridor_Passing` | Đối đầu trong hành lang | 2,10 / 1,20 / 0,90 |
| `ND_Doorway_Passing` | Hai phía tranh cửa hẹp | 1,20 / 0,90 / 0,60 |
| `BC_Corner_Turn` | Góc khuất | 2,10 / 1,20 / 0,90 |
| `IN_Crossing_Paths` | Giao cắt | 2,10 / 1,20 / 0,90 |
| `OT_Same_Direction` | Vượt cùng chiều | 2,10 / 1,60 / 0,90 |
| `MG_Small_Group` | Tương tác với nhóm nhỏ | 2,10 / 1,80 / 1,60 |

OT có hai subtype do seed quyết định. Dùng cùng Card, level, seed và action khi muốn đối chiếu backend. Dùng cùng backend, Card, level và seed khi muốn đối chiếu hành động. [Ma trận và lịch lệnh chi tiết](SCENARIO_MATRIX.md)

## 2. Chuẩn bị runtime và chọn đúng slot

Chạy các khối lệnh theo thứ tự trong cùng một phiên Bash. Ví dụ dùng checkout sạch và interpreter 04U đã nghiệm thu tại workspace này. `git status --short` của checkout cần rỗng để chạy slot nghiên cứu. Nếu thiếu checkout hoặc interpreter, xem cách phục hồi bundle và cài binary đã khóa. [Khôi phục runtime](REPRODUCIBILITY_PACKAGE.md#khôi-phục-vào-môi-trường-riêng)

```bash
repo=/home/haduckien/nav_ws
checkout="$repo/runs/task_08d/checkout"
py="$repo/runs/task_08b/jupedsim_04u/venv/bin/python"
cd "$checkout"
git status --short
git rev-parse --short HEAD
"$py" -V

work="$(mktemp -d "$repo/runs/human_nav_test_XXXXXX")"
plan="$work/phase2.json"
run="$work/nd_run"
report="$work/nd_report"

bench() {
  env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src" \
    "$py" -B -m scripts.benchmarks.plan "$@"
}

bench plan --phase 2 --bindings configs/benchmarks/bindings.yaml --output "$plan"
printf 'Output: %s\n' "$work"
```

Dự kiến Git HEAD bắt đầu bằng `0d86e376`, Python là 3.12.3, plan có 900 slot. `plan` chỉ sinh danh sách. `dry-run` nạp native backend và kiểm scene, route, khẩu độ và lịch lệnh, rồi dừng trước khi bước vật lý. Dùng CSV do `plan` sinh để tìm đúng `slot_id`, vì lọc chỉ theo backend, Card và seed có thể trả về nhiều level hoặc action.

```bash
find_slot() {
  "$py" -B - "$work/phase2.csv" "$@" <<'PY'
import csv
import sys

path, backend, card, level, action, seed = sys.argv[1:]
with open(path, newline="") as file:
    matches = [
        row["slot_id"]
        for row in csv.DictReader(file)
        if (row["backend"], row["card_id"], row["level"], row["action"], row["seed"])
        == (backend, card, level, action, seed)
    ]
if len(matches) != 1:
    raise SystemExit(f"Cần đúng một slot, tìm được {len(matches)}")
print(matches[0])
PY
}

run_slot() {
  local destination="$1" slot="$2"
  local -a resume=()
  if [[ -f "$destination/run.json" ]]; then
    resume=(--resume)
  fi
  bench dry-run --manifest "$plan" --episode "$slot" &&
    bench run --manifest "$plan" --output-dir "$destination" \
      --execute --episode "$slot" "${resume[@]}"
}
```

`--resume` dùng khi thư mục test đã có run lock và provenance vẫn khớp. Mỗi ví dụ chỉ thực thi đúng slot chỉ định. `--only-new` dành cho nối phase trên run có đủ kết quả trước đó. [Cú pháp, retry và provenance](BENCHMARK_REPRODUCIBILITY.md)

## 3. Chạy các cấu hình mẫu

### Bốn backend trên cùng cửa hẹp

Chọn **ND Critical, cửa 0,90 m, một người, Advance, seed 1010**. Khối này chạy tổng cộng bốn lượt và xuất báo cáo khi tất cả lệnh thực thi thành công.

```bash
(
  set -e
  for backend in orca sfm_jupedsim avm_jupedsim csm_jupedsim; do
    slot="$(find_slot "$backend" ND_Doorway_Passing critical advance 1010)"
    run_slot "$run" "$slot"
  done
  bench report --run-dir "$run" --output-dir "$report"
)
```

Kết quả tham chiếu của đúng bốn slot này trên snapshot đã khóa.

| Backend | Robot tới đích | An toàn | Nhãn ND | Hành vi người |
| --- | --- | --- | --- | --- |
| ORCA | Có | Đạt | Đạt | `GOALS_INCOMPLETE` |
| SFM | Có | Đạt | Trượt | `NO_PROGRESS` |
| AVM | Dừng do contact | Trượt | Trượt | `SAFETY_FAILURE` |
| CSM V2 | Có | Đạt | Đạt | `NO_PROGRESS` |

Ở SFM, robot qua cửa nhưng người chưa qua trước khi lượt kết thúc. Ở CSM V2, cả hai đã qua cửa nhưng người chưa đạt mục tiêu và có trên 5 s không cải thiện khoảng cách tốt nhất. Dùng bốn lượt này để kiểm đường chạy và diễn giải kết quả từng episode. Đối chiếu thống kê dùng các cặp đã nghiệm thu của benchmark. [Bằng chứng ND từng lượt](../../runs/task_08e2/report/nd_doorway_qa.csv)

### Một cặp Advance và Yield

Chọn **CSM V2, FA Critical 1,20 m, seed 1000** cho cả hai vế. Yield đổi lệnh ở 4 s. Báo cáo CLI xuất kết quả từng lượt. Để xem common prefix, chênh lệch chuyển động và mẫu số, dùng bảng action audit của Vòng 1. [Action contrasts](../../runs/task_08e1/report/action_contrasts.csv)

```bash
(
  set -e
  for action in advance yield; do
    slot="$(find_slot csm_jupedsim FA_Corridor_Passing critical "$action" 1000)"
    run_slot "$work/fa_action_run" "$slot"
  done
  bench report --run-dir "$work/fa_action_run" --output-dir "$work/fa_action_report"
)
```

### Smoke kỹ thuật và kiểm thử mã

```bash
bench plan --phase smoke --output "$work/smoke.json"
bench dry-run --manifest "$work/smoke.json" --episode benchmark_smoke
bench run --manifest "$work/smoke.json" --output-dir "$work/smoke_run" \
  --execute --episode benchmark_smoke

env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src" "$py" -B -m pytest -q -rs \
  --basetemp "$work/pytest" tests/unit/test_benchmark_08a.py \
  tests/unit/test_module_dependencies.py tests/simulation/test_collision_monitor.py
```

Smoke kỹ thuật có thể kết thúc bằng `TIMEOUT` với exit 0. Exit 0 của runner xác nhận attempt hợp lệ hoặc reuse thành công. Kết quả nhiệm vụ nằm trong `outcome` và `episode_task_pass`. Nếu runner exit 2, xem lỗi ở terminal và `qa.csv` nếu có attempt đã ghi. Giữ log gốc và chỉ retry lỗi hạ tầng hoặc gián đoạn theo giao thức. [Quy tắc xử lý lỗi](DATA_QA_PROTOCOL.md)

## 4. Đọc metrics của lượt test vừa chạy

`per_episode.csv` của ví dụ ND có các dòng kế hoạch Phase 2, gồm cả `NOT_RUN`. Lọc theo `attempts > 0` để thấy bốn lượt đã thử, và dùng `infrastructure_valid` để kiểm lượt nào có kết quả hợp lệ. `qa.csv` có một dòng cho từng attempt và đường dẫn đến dữ liệu gốc.

```bash
"$py" -B - "$report/per_episode.csv" <<'PY'
import csv
import sys

fields = (
    "backend", "outcome", "S", "C", "TO", "FP", "episode_task_pass",
    "scenario_label_valid", "human_behavior", "psc_fraction", "min_ttc_s",
)
print("\t".join(fields))
with open(sys.argv[1], newline="") as file:
    for row in csv.DictReader(file):
        if int(row["attempts"]) > 0:
            print("\t".join(row[field] or "—" for field in fields))
PY
```

| Cột hoặc chỉ số | Cách đọc |
| --- | --- |
| `S` và SR | `S` là robot tới đích theo engine. SR là số lượt có `S=True` chia cho số lượt hợp lệ có đo được `S`. |
| `C` và CR | `C` đếm các đợt contact solver robot–người và robot–tường. CR là tỷ lệ lượt có `C>0`, với mẫu số là các lượt hợp lệ có đo được contact. |
| `episode_task_pass` | Robot tới đích và qua cổng an toàn. Khoảng hở âm có thể làm cột này `False` dù `S=True` và `C=0`. |
| `scenario_label_valid` | Tiêu chí của Card có xuất hiện trong quỹ đạo. ND yêu cầu hai phía từng đồng thời ở gần tâm cửa và cả robot lẫn ít nhất một người cắt khẩu độ. |
| `TO`, `FP`, stall | Hết ngân sách, số sự kiện robot không cải thiện tiến độ trên 5 s, và thời gian tốc độ dưới 0,02 m/s. Khoảng chờ chủ ý tạm dừng bộ đếm FP nhưng vẫn tính vào stall. |
| Clearance và PSC | Clearance là khoảng hở bề mặt có dấu, đơn vị m. PSC là tỷ phần khoảng điều khiển kết thúc với tâm người cách tâm robot dưới 0,5 m. PSC `0.10` nghĩa là 10%. |
| TTC và SPL | TTC là thời gian dự báo theo vận tốc tương đối tức thời, đơn vị s. Xem `ttc_status` trong `validity.json` khi ô trống. SPL trống nếu thiếu đường tham chiếu đã duyệt. |
| Tốc độ, gia tốc, jerk, mục tiêu người | Xem `quality.robot`, `quality.humans` và `quality.human_goal_s` trong `validity.json`. Các đơn vị lần lượt là m/s, m/s² và m/s³. |

Trong `summary.csv` của CLI, `sr_n`, `cr_n` là mẫu số thô và `*_ci95` là khoảng Wilson 95%. Cột `pass` đếm lượt đạt nhiệm vụ an toàn **trong tập đạt nhãn**. Muốn đếm an toàn trên tập thô, dùng `episode_task_pass` của các lượt hợp lệ trong `per_episode.csv` hoặc `safe_task_pass_count` và `safe_task_pass_n` ở bảng coverage phân tầng. [Định nghĩa và mẫu số chuẩn](METRICS_AND_THRESHOLDS.md)

Mở cấu hình đã resolve và số đo gốc của một lượt.

```bash
slot="$(find_slot csm_jupedsim ND_Doorway_Passing critical advance 1010)"
attempt="$run/attempts/$slot/attempt-0001"
less "$attempt/resolved_config.yaml"
"$py" -B -m json.tool "$attempt/validity.json" | less
```

`manifest.json` có fingerprint và trạng thái kết thúc. `trace.jsonl` giữ pose, vận tốc và clearance tại 50 ms. `gt_10hz.jsonl` giữ GT tại 100 ms. Ba tệp `actions_*.jsonl` giữ requested, limited và executed. `events.jsonl` ghi contact solver tại mốc substep 2 ms. Kênh solver người–người và người–tường có thể là `null` vì mask vật lý. Giữ ô trống và `null` khi phân tích.

## 5. Xem thống kê benchmark 900 lượt

Có thể mở các CSV dưới đây trực tiếp trong trình xem bảng tính hoặc dùng Python `csv.DictReader`. Một số ô chứa mảng hoặc đối tượng JSON, nên dùng trình đọc CSV hỗ trợ ô có dấu ngoặc kép.

| Muốn xem | Tệp và cách lọc |
| --- | --- |
| Kết quả từng episode | [per_episode.csv](../../runs/task_08e2/report/per_episode.csv) có `backend`, `card_id`, `level`, `seed`, `action`, hard constraints và metric chính. [episode_audit.csv](../../runs/task_08e2/report/episode_audit.csv) thêm số đo từng nhóm và đường dẫn raw. |
| Tỷ lệ theo backend hoặc Card | [coverage_breakdown.csv](../../runs/task_08e2/report/coverage_breakdown.csv). Lọc `scope=backend`, `card`, `card_level_backend` hoặc `critical_backend`. Đọc tử số, mẫu số và KTC ở `*_count`, `*_n`, `*_ci95`. Tiền tố `labeled_` là tập đạt nhãn. |
| Chuyển động và tương tác | [backend_comparison.csv](../../runs/task_08f/backend_comparison.csv) tách `raw`, `labeled` và `labeled_hard_pass`. [motion_distributions.csv](../../runs/task_08e2/report/motion_distributions.csv) có `bin_edges`, `bin_counts`, `available`, `missing` và `excluded`. |
| So sánh ghép cặp | [paired_backend_summary.csv](../../runs/task_08e2/report/paired_backend_summary.csv). Chọn `scope=all` hoặc `critical_a`. Đọc hiệu ứng trái trừ phải trong `left_minus_right` và mẫu số trong `available_pairs_by_metric`. [Chi tiết từng cặp và lý do loại](../../runs/task_08e2/report/paired_backend_audit.csv) |
| Stop, Yield và Reverse | [action_contrasts.csv](../../runs/task_08e1/report/action_contrasts.csv) có Advance reference, xác nhận đổi hành động, lý do loại và chênh lệch quỹ đạo. [action_contrast_summary.csv](../../runs/task_08e1/report/action_contrast_summary.csv) tổng hợp theo backend × Card × action. |
| Histogram và các lượt lỗi | [Biểu đồ Advance của Vòng 1](../../runs/task_08e1/report/branch_a_histograms.svg) minh họa cách đọc phân phối. Dùng bảng phân phối Phase 2 ở trên cho 900 lượt. [Failure catalog](../../runs/task_08f/failure_catalog.csv) lưu contact, overlap, guard, tiến độ và nguyên nhân trượt ND theo evidence. |

Để in nhanh SR, CR và tỷ lệ đạt an toàn của bản 900 lượt ngay trong terminal.

```bash
"$py" -B - "$repo/runs/task_08f/statistical_summary.json" <<'PY'
import json
import sys

with open(sys.argv[1]) as file:
    data = json.load(file)
for backend, summary in data["backend"].items():
    print(backend)
    for label, key in (("SR", "engine_goal"), ("CR", "solver_collision"),
                       ("Đạt an toàn", "safe_task_pass")):
        row = summary["raw"][key]
        low, high = row["ci95"]
        print(f"  {label}: {row['count']}/{row['n']} = {row['rate']:.1%}"
              f"  [KTC 95% {low:.1%}–{high:.1%}]")
PY
```

Nếu cần tái tạo báo cáo CLI từ run đã khóa, truyền đường dẫn run vào lệnh `report` và dùng một output mới. Lệnh này đọc artifact, không chạy lại MuJoCo. Bảng phân tầng và kiểm toán mở rộng đã có sẵn trong gói 08F. [Báo cáo nghiệm thu](../reports/human_models/TASK_08F_REPORT.md) · [Chỉ mục toàn bộ artifact](../../runs/task_08f/release_manifest.json)

```bash
bench report --run-dir "$repo/runs/task_08d/benchmark_study" \
  --output-dir "$work/full_report"
```
