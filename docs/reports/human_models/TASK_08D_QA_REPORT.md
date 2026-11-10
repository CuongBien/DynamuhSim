# TASK 08D-QA · Kiểm toán Vòng 0 và cổng tái sử dụng

**`READY_FOR_ROUND_1` trên checkout cô lập `codex/task-08d-snapshot`, commit `0d86e376fff8ec54c6c9dd3b08330e283b2ad042`.** Vòng 0 đạt cổng hạ tầng. Cả 48 episode có thể tái dùng làm kết quả thực nghiệm thô trên checkout và runtime này. Trong đó 44 episode đủ điều kiện vào thống kê theo nhãn Card. Checkout làm việc chính `mujoco-hummodel` hiện tái dùng được 0/48 do khác provenance Git. Vòng 1 mới được kiểm tra bằng plan và dry-run. [Tổng hợp máy đọc](../../../runs/task_08d_qa/summary.json) · [Đối chiếu 48 slot](../../../runs/task_08d_qa/reuse_eligibility.csv)

## 1. Hạ tầng và dữ liệu

| Chỉ số | Kết quả kiểm lại |
| --- | ---: |
| Episode hợp lệ về hạ tầng | 48/48 · 100% |
| Đạt nhiệm vụ robot và an toàn | 41/48 · 85,42% |
| Có contact solver robot–người | 6/48 · 12,50% |
| Có khoảng hở hình học âm | 7/48 · 14,58% |
| Chỉ có khoảng hở âm, solver không ghi contact | 1/48 · 2,08% |
| Đạt nhãn Card | 44/48 · 91,67% |
| Nhãn `false` / `null` | 4 / 0 |

Mẫu số là 48 slot duy nhất, đủ tích Descartes của bốn backend, sáu Card Easy và hai seed 1000/1001. Mỗi slot có một attempt và một episode ID. Có 42 lượt engine kết thúc `goal_reached` và sáu lượt `robot_contact`. Số crash, timeout, lượt gián đoạn và retry đều bằng 0. Một lượt `goal_reached` trượt cổng an toàn hình học. Tỷ lệ đạt đích của engine là 42/48, tỷ lệ đạt nhiệm vụ và an toàn là 41/48. [Audit từng lượt](../../../runs/task_08d_qa/episode_audit.csv) · [Manifest và provenance](../../../runs/task_08d/benchmark_study/run.json)

- Đọc lại sáu kênh gốc của mỗi attempt. Đối chiếu SHA-256 cấu hình đã resolve, MJCF biên dịch, asset, seed luồng, snapshot reset, trạng thái kết thúc và version backend. 12 nhóm Card/seed có cùng geometry và reset giữa bốn backend. Hash toàn bộ cây artifact gốc khớp trước/sau audit. [Audit của 08D](../../../runs/task_08d/audit_round_0.py) · [Audit 08D-QA](../../../runs/task_08d_qa/audit.py)
- Đối chiếu 11.819 tick điều khiển, 295.475 substep vật lý và 5.941 mẫu GT trên lưới 10 Hz. Kiểm ID, frame và timestamp ở từng kênh. Tính lại limiter robot/người từ requested, so executed với sai phân pose và số substep guard. Sai số lớn nhất khi đối chiếu executed người là 0 m/s. Kênh solver giữ 84 mẫu robot–người trên lưới 2 ms, gộp thành sáu đợt. Mẫu robot–tường bằng 0. Khoảng hở người–tường/người–người không âm ở các tick công bố. Guard hoạt động ở 7 lượt, tổng 900 substep. [Hợp đồng QA](../../benchmarks/DATA_QA_PROTOCOL.md)
- Kiểm luồng lệnh ở commit chạy. Requested robot lấy từ `command_at(case["schedule"], stamp.time_ns)`. Mọi requested robot khớp lịch đã khóa tại từng tick. GT dùng để ghi dữ liệu, tính executed và chấm nhãn sau episode. Robot chạy lịch vòng hở với cảm biến tắt. [Runner tại commit chạy](../../../runs/task_08d/checkout/src/narrow_nav/evaluation/runner.py)

## 2. Bảy lỗi an toàn

Mọi dòng đều dùng hành động Advance. Tiền tố episode ID trong bảng trỏ tới ID đầy đủ trong CSV. Khoảng hở là phép đo đĩa tại tick 50 ms. Khoảng thời gian contact lấy từ các mẫu solver 2 ms.

| Episode | Backend | Card / seed | Rộng (m) | Kết thúc | Contact solver (s) | Khoảng hở robot–người nhỏ nhất (m) | Loại |
| --- | --- | --- | ---: | --- | --- | ---: | --- |
| `2cd2c7da` | AVM | ND / 1000 | 1,2 | `robot_contact` | 9 mẫu, 4,634–4,650 | −0,021741 | A |
| `6d1354a4` | AVM | ND / 1001 | 1,2 | `robot_contact` | 10 mẫu, 4,332–4,350 | −0,023773 | A |
| `d9c7dc97` | AVM | MG / 1000 | 2,1 | `robot_contact` | 23 mẫu, 3,856–3,900 | −0,025786 | A |
| `5de6ac3c` | AVM | MG / 1001 | 2,1 | `robot_contact` | 20 mẫu, 3,462–3,500 | −0,024930 | A |
| `9f3f85f4` | ORCA | ND / 1000 | 1,2 | `goal_reached` | 0 mẫu | −0,0000595 | B |
| `735f8835` | SFM | ND / 1000 | 1,2 | `robot_contact` | 5 mẫu, 4,492–4,500 | −0,065118 | A |
| `2801b867` | SFM | ND / 1001 | 1,2 | `robot_contact` | 17 mẫu, 4,318–4,350 | −0,067738 | A |

**A · Lỗi tương tác quan sát được.** Sáu lượt có contact robot–người và kết thúc theo điều kiện đã khóa. Requested và limited robot vẫn tiến theo lịch Advance tại các khoảng va chạm. Executed khớp chuyển động thực tế, gồm phản ứng động lực học ở tick va chạm. Cả sáu robot chưa tới đích. Các dòng CSV lưu tiến độ từng actor, đủ timestamp solver và ba tầng lệnh quanh mỗi mốc lỗi. Bằng chứng đo trên cặp tác nhân, nên chưa quy kết lỗi cho riêng robot hay người.

**B · Chênh lệch giữa hai phép đo tiếp xúc.** ORCA ND seed 1000 có giao lấn đĩa sâu 0,0595 mm tại 6,20 s. Solver dùng hình học va chạm MJCF, còn clearance dùng bán kính quy ước. Tệp hiện có ghi sự kiện và pose 50 ms, nên nguyên nhân chính xác ở substep chưa quan sát được. Lượt này vẫn là `SAFETY_FAILURE` theo ngưỡng đã khóa. Robot tới đích, người còn cách đích 0,634 m khi episode kết thúc.

Trong bảy lượt lỗi, AVM MG seed 1000 ghi 294 substep guard và ORCA ND seed 1000 ghi 30. Hai lượt này không ghi guard ở các cửa sổ lỗi đã trích xuất. Các lượt còn lại ghi 0 guard. Telemetry chi tiết về đồng bộ state của backend không được xuất theo episode. Audit không phát hiện lỗi dữ liệu hoặc hạ tầng ở bảy lượt này và không tìm thấy bằng chứng để xếp loại C hay D. [Bằng chứng từng ca và cửa sổ lệnh](../../../runs/task_08d_qa/safety_failures.csv) · [Hình học robot MJCF](../../../runs/task_08d/checkout/sim/mujoco/models/robots/mecanum_robot/robot.xml)

## 3. Bốn nhãn Card không đạt

Bốn ca đều là ND Easy của AVM/SFM ở hai seed. Cả robot và người ở trong bán kính 3 m quanh tâm cửa ngay từ reset. Người đã cắt qua khẩu độ ở khoảng 3,31–4,12 s. Robot đi 2,05–2,22 m rồi dừng do contact khi còn cách mặt phẳng cửa 0,48–0,65 m. Tiêu chí không đạt là **quỹ đạo robot cắt qua cửa**. Tương tác đối đầu đã xảy ra và được ghi bằng contact. Route danh nghĩa của cả hai vẫn hợp lệ qua cửa, geometry và lịch lệnh khớp binding.

Đây là loại **3 · kết quả hành vi hợp lệ với label `false`**. Bốn lượt vẫn thuộc mẫu số thô của Vòng 1, gồm tỷ lệ va chạm và thất bại. Các lượt này nằm ngoài thống kê điều kiện theo nhãn và ngoài những cặp yêu cầu hai nhãn hợp lệ. Không có nhãn `null`. [Bốn nhãn và thời điểm qua cửa](../../../runs/task_08d_qa/card_label_failures.csv) · [Quy tắc ND](../../research/HUMAN_MODEL_FINAL.md#L187)

Phân loại hành vi người của 48 lượt gồm 32 `GOALS_REACHED`, 7 `GOALS_INCOMPLETE`, 2 `NO_PROGRESS` và 7 `SAFETY_FAILURE`. Hai ca `NO_PROGRESS` là CSM V2 ở ND, dù robot đạt nhiệm vụ. Có 20 mục tiêu người chưa được quan sát là đạt trước lúc episode kết thúc. SPL trống ở 48 lượt do thiếu chiều dài đường tham chiếu. TTC có giá trị hữu hạn ở 43 lượt, còn năm lượt mang trạng thái `no_predicted_contact`. Solver người–tường/người–người ngoài khả năng quan sát, cảm biến tắt. Các chỉ số realism khảo sát/học, legibility, PI, cohesion và khoảng cách crossing riêng chưa có số đo. [Định nghĩa metric](../../benchmarks/METRICS_AND_THRESHOLDS.md) · [Tổng hợp](../../../runs/task_08d_qa/summary.json)

## 4. Cổng tái sử dụng Vòng 1

Biên dịch Vòng 1 trên chính snapshot sạch của Vòng 0 bằng binding 08B. Dry-run đạt 540/540 slot, gồm 48 slot Easy Advance từ nhánh A và 492 slot mới. Với từng slot cũ, audit resolve lại cấu hình, so Card/level/seed, subtype, policy, spawn/goal, schedule, timing và tiêu chí dừng. Sau đó so hash cấu hình, MJCF và asset, version và build native, Python/MuJoCo/dependency, commit/dirty/code fingerprint, tham chiếu phê duyệt 08B và `plan_id`. Tính lại `execution_key` rồi gọi cổng `_reuse` của runner. Cả 48 key khớp attempt hợp lệ. [Dry-run](../../../runs/task_08d_qa/round_1_dry_run.log) · [Manifest Vòng 1](../../../runs/task_08d_qa/round_1.json) · [So khớp từng slot](../../../runs/task_08d_qa/reuse_eligibility.csv)

| Đích được kiểm tra | `REUSABLE` | `NOT_REUSABLE` | `INDETERMINATE` |
| --- | ---: | ---: | ---: |
| Snapshot sạch `0d86e376`, đúng interpreter 04U và run directory gốc | 48 | 0 | 0 |
| Workspace chính `b50d0844`, đang dirty | 0 | 48 | 0 |

Workspace chính có cùng code fingerprint tại thời điểm kiểm toán. Runner vẫn yêu cầu provenance chính xác, và `git_branch`, `git_commit`, `git_dirty` khác nhau. Cổng sẵn sàng áp dụng cho snapshot và runtime trong `summary.json`. Bất kỳ checkout khác nào phải qua so khớp lại trước khi tiếp tục.

## 5. Tái lập và nghiệm thu

Lệnh chạy từ snapshot đã ghi trong `summary.json`. `repo` là đường dẫn tuyệt đối của workspace. Kết quả audit đặt dưới `runs/task_08d_qa/`.

```bash
repo=/home/haduckien/nav_ws
source_run="$repo/runs/task_08d"
out="$repo/runs/task_08d_qa"
P="$repo/runs/task_08b/jupedsim_04u/venv/bin/python"
(cd "$repo" && env -u PYTHONPATH "$P" -B -c 'import json; from scripts.benchmarks.plan import _runtime; print(json.dumps(_runtime()))') > "$out/primary_runtime.json"
cd "$source_run/checkout"
headless() { env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$PWD/src:$PWD" "$P" -B "$@"; }
headless -m scripts.benchmarks.plan plan --phase 1 --bindings configs/benchmarks/bindings.yaml --output "$out/round_1.json"
headless -m scripts.benchmarks.plan dry-run --manifest "$out/round_1.json" > "$out/round_1_dry_run.log"
headless "$out/audit.py" "$source_run" "$out"
headless -m pytest -q -rs --basetemp "$out/test_fixtures" tests/unit/test_benchmark_08a.py tests/unit/test_module_dependencies.py tests/unit/test_seed.py tests/unit/test_config_validation.py tests/unit/test_core_states.py tests/unit/test_core_actions.py
headless -m pytest -q -rs --basetemp "$out/regression_fixtures" tests/simulation/test_collision_monitor.py tests/simulation/test_mjcf_compile.py tests/integration/test_simulation_contract.py
```

Audit tính lại metrics và labels từ dữ liệu gốc. Reporter tái sinh chín tệp CSV/JSON trong thư mục tạm và tất cả khớp byte với output 08D. Hai nhóm pytest đạt 29 và 7 test, không skip. SHA-256 của nguồn nghiên cứu vẫn là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Phạm vi kiểm định gồm dữ liệu đã ghi, cấu hình resolve và fixture test. [Log test audit/schema/report](../../../runs/task_08d_qa/pytest.log) · [Log hồi quy contact/MJCF](../../../runs/task_08d_qa/pytest_regression.log)

1. **Vòng 0 đạt infrastructure gate** với 48/48 attempt hợp lệ.
2. **Bảy lỗi an toàn là kết quả thực nghiệm** gồm sáu lỗi tương tác có solver contact và một lỗi theo phép đo đĩa. Không phát hiện lỗi dữ liệu hoặc hạ tầng trong các ca này.
3. **Bốn label failures được giữ trong Vòng 1.** Chúng góp vào thống kê thô và bị loại khỏi thống kê yêu cầu nhãn hợp lệ.
4. **48/48 reusable trên snapshot đã xác minh.** Có 44/48 đủ điều kiện theo nhãn và 41/48 đạt đồng thời nhãn và cổng an toàn/nhiệm vụ.
5. **Scenario Card và runner hiện đáp ứng cổng kiểm toán.** Các ca quan sát được có đường danh nghĩa hợp lệ, và runner đã phân biệt contact, clearance, nhãn và trạng thái thực thi.
6. **Có thể mở TASK 08E-1 trên snapshot và runtime đã xác minh.** Vòng 1 dùng 48 lượt có sẵn và 492 slot mới. Workspace chính còn bị cổng provenance chặn.
