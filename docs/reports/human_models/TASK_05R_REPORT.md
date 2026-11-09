# TASK 05R · Backend SFM và kiểm định tương tác

| Cổng | Kết quả | Bằng chứng |
| --- | --- | --- |
| Implementation | **IMPLEMENTATION_PASS** | Registry `sfm_jupedsim` chạy native SFM qua lifecycle, shared limiter, MuJoCo và GT. Smoke và kiểm thử dùng build 04S thật. |
| Đồng bộ | **SYNC_PASS** | Rebase người và readback robot khớp GT tại callback. Hai nhịp 50/100 ms dùng 5/10 bước native 10 ms. Không có lỗi runtime trong 12 lượt cuối. |
| Tương tác | **INTERACTION_FAIL** | Free và cả bốn nhánh Easy đạt hard constraints. Critical và Hard có solver contact dù qua điều kiện hình học cần. |
| Giới hạn model | **MODEL_LIMITATION** | Thay vận tốc robot tại cùng XY trước overlap không đổi đáp ứng một bước. SFM 1.4.2 dùng relative velocity trong ma sát tiếp tuyến khi hai footprint chồng lấn. [Nguồn SFM 1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/SocialForceModel.cpp#L133-L173) |
| Stock runtime | **BLOCKED** | JuPedSim stock 1.4.2 thiếu API external/rebase. Registry trả lỗi cấu hình khi chọn SFM. |

## Runtime và phạm vi

- Nhánh `mujoco-hummodel`, HEAD `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, upstream local 0/0. Checkout có thay đổi chưa commit từ 04R/04S và hai file `orca_progress_tools/`. Index sạch. SHA-256 nguồn trước/sau là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.
- Dùng dependency của `.venv` qua interpreter cô lập `runs/task_04s/reproduction/venv/bin/python`, CPython 3.12.3, JuPedSim `1.4.2+task04s` cp312 x86-64, MuJoCo 4.0.0. Commit nguồn `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`, patch SHA-256 `0d79faa084acf5b863a3944258499837b46f7f9ccae56d8119b3a555bab710e4`, binary SHA-256 `77257fc9679c3ac2691495c509ef444937301351c175525231628bc1ceae091f`. Bản stock vẫn ở `.venv`. [Preflight](../../../runs/task_05r/preflight.log) · [Báo cáo 04S](TASK_04S_REPORT.md)
- Thêm backend và YAML `sfm_jupedsim`, đăng ký factory, cấp `PlanarWorld` tùy chọn từ collision đã load cho backend qua config v1. Thêm runner, metrics, phân tích offline, biểu đồ và tests. Cập nhật ADR, schema, backend docs, ledger và tiến độ. Thay assertion registry lịch sử trong test 04R bằng test 05R xác nhận stock từ chối rõ ràng và build 04S chạy qua registry.
- Tham số native mặc định gồm body force 120.000 kg/s², friction 240.000 kg/(m·s), mass 80 kg, desired speed 0,8 m/s, reaction time 0,5 s, agent/obstacle scale 2.000 N và force distance 0,08 m. Bán kính lấy từ scene, robot mecanum 0,18 m và người 0,25 m. Waypoint tolerance 0,1 m. Đọc pose/velocity hiện hành và gửi lệnh requested. Không đổi guard, limiter, timestep vật lý, phương trình SFM hay policy backend khác. [YAML](../../../configs/humans/sfm_jupedsim.yaml) · [Hợp đồng backend](../../backends/SFM_JUPEDSIM.md) · [ADR](../../decisions/ADR-0007-jupedsim-external-actor.md)

## Fixture A–F

Các lượt đối đầu dùng seed 11, corridor dài 10 m, cùng trạng thái đầu và mục tiêu. Bốn nhánh Easy trùng trace đến 4 s. Trong cửa sổ [4, 6) s, mỗi nhánh áp advance, stop, yield hoặc reverse. Yield dạt ngang 1 s rồi trở về làn 1 s. Từ 6 s robot tiếp tục tiến. Phiên bản thử đầu chỉ dạt một chiều, bỏ lỡ làn đích và chạm biên footprint cuối hành lang tại 24,70 s. Lượt đó giữ ở `initial/easy_yield.json` dưới dạng lỗi thiết kế lịch lệnh, trước khi áp lịch cuối. Không đổi lực SFM hay độ rộng để chỉnh kết quả.

| Fixture | Min clearance robot–người (m) | Mẫu solver / số chuỗi contact | Kết quả |
| --- | ---: | ---: | --- |
| Free và lặp lại | 2,570119 | 0 / 0 | Hai lượt PASS. GT trùng nhau, người vào dung sai lúc 8,25 s. |
| Static robot | 0,127175 | 0 / 0 | Người lệch ngang tối đa 0,368889 m. An toàn footprint, FAIL nhiệm vụ do robot cố định nên không tới đích, timeout và FP. |
| Easy advance, 2,1 m | 0,043438 | 0 / 0 | PASS, robot tới đích 17,20 s. |
| Easy stop | 0,079206 | 0 / 0 | PASS, 19,20 s. |
| Easy yield | 0,069752 | 0 / 0 | PASS, 19,40 s. |
| Easy reverse | 0,044384 | 0 / 0 | PASS, 20,60 s. |
| Critical 1,2 m, 100 ms | −0,025456 | 13 / 1 | FAIL, robot–người tại tick kết thúc 5,70 s. |
| Critical 1,2 m, 50 ms | −0,024781 | 7 / 1 | FAIL, robot–người tại tick kết thúc 5,65 s. |
| Hard 0,9 m, 100 ms | −0,028984 | 12 / 1 | FAIL, robot–người tại tick kết thúc 5,95 s. |
| Replay 04S, 0,85 m, 100 ms | −0,062092 | 610 / 1 | FAIL, 822 guard substep. Dư bề rộng −0,01 m. |
| Replay 04S, 0,85 m, 50 ms | −0,062159 | 610 / 1 | FAIL, 805 guard substep. Dư bề rộng −0,01 m. |

Đo độ rộng từ collision ở mặt cắt giữa hành lang. Dư bề rộng khi qua song song là `w − 2(r_robot+r_human)`, lần lượt 1,24 m, 0,34 m và 0,04 m cho Easy, Critical và Hard. Đây là điều kiện cần cho hai footprint tròn, chưa chứng minh đủ khoảng động để nhường. Hai replay 0,85 m được giữ và gắn nhãn bất khả thi khi qua song song. Critical và Hard kết thúc do contact trước khi đủ thời lượng để đánh giá deadlock kéo dài trên 5 s. [Summary](../../../runs/task_05r/summary.json) · [Biểu đồ Easy và Free](../../../runs/task_05r/paired-and-free-timeline.png)

Free walking giữ sai lệch heading so với vận tốc khi di chuyển dưới `1,33e-10 rad`, tốc độ người tối đa 0,799909 m/s và cuối lượt cách đích 0,000091 m. Gia tốc cực đại đo ở tick robot là 3,0 m/s², phản ánh bước nhảy vận tốc giới hạn 0,15 m/s ở biên người 100 ms khi đo trên bước 50 ms. Shared limiter dùng giới hạn tốc độ 1,4 m/s và giới hạn gia tốc 1,5 m/s² trên lưới người. Không áp nhãn `cooperative` hay `non_cooperative` cho hành vi lực thuần.

Phép thử D ở cùng XY cách 0,60 m cho `vx_h = −0,013858 m/s` khi `vy_robot` là 0 hoặc 0,40 m/s. Dời robot ra 1,20 m cho `vx_h = +0,015983 m/s` với robot đứng. Tại overlap 0,35 m, thêm vận tốc tiếp tuyến tạo chênh `vy_h = +0,96 m/s`. Đây là đo riêng hiệu ứng vị trí và vận tốc bằng một bước native. [Trace một bước](../../../runs/task_05r/velocity-sensitivity.json)

## Truy vết thất bại 04S

- Mốc dương cuối trước overlap là **0,85 s**, clearance **+0,006881 m** và khoảng cách tâm **0,436881 m**. Mẫu đầu âm ở **0,90 s**, clearance **−0,002588 m**, khoảng cách tâm **0,427412 m**, vận tốc tương đối X người trừ robot **−0,204874 m/s**. Tick 50 ms chỉ định vị được khoảng `(0,85; 0,90]` s. Solver contact đầu tiên ghi ở **1,782 s** theo substep 2 ms. Guard đầu tiên xuất hiện ở tick kết thúc **1,40 s**.
- Tại mẫu callback 0,80 s, robot được giữ trong 100 ms và MuJoCo đã đi **0,034683 m** khi lấy mẫu 0,90 s. Trong tick phát hiện overlap, requested X người **0,894503 m/s**, limited và executed **0,155784 m/s**, guard **0**. Replay offline tại các trạng thái GT giống nhau, lấy chênh vận tốc sau một bước native có/không có robot nhân `mass / 0,01 s`, cô lập được lực X do robot khoảng **1.835 N** ở 0,85 s và **2.376 N** ở 0,90 s. Đây là hiệu ứng lực SFM, không phải force từ solver.
- Limiter đã thu nhỏ đáp ứng trước overlap. Guard chỉ bắt đầu về sau khi người tiến sát biên tường. Hình học 0,85 m thiếu 0,01 m cho qua song song. Lượt 50 ms giảm hold lag tối đa từ **0,034780 m** xuống **0,017480 m**. Min clearance và 610 mẫu contact gần như giữ nguyên. Kết quả xác nhận nhạy với nhịp lấy mẫu và sự khác nhau giữa các lớp lệnh. Phép thử chỉ đổi chu kỳ người, nên cả hold và cadence limiter thay đổi cùng nhau. Chưa cô lập được đóng góp riêng của hold, limiter hay lực native vào thời điểm va chạm.
- 610 mẫu trên cùng cặp ở các substep liên tiếp là **một** chuỗi contact từ 1,782 s đến 3 s. Đếm chuỗi mới khi cặp đó tái xuất sau ít nhất một substep vắng mặt. Clearance âm và guard giữ riêng, không gộp vào solver contact. [Chi tiết overlap và lực](../../../runs/task_05r/wall-04s-analysis.json) · [Biểu đồ thời gian](../../../runs/task_05r/narrow-timeline.png)

## Định nghĩa đo và lệnh chạy

S là robot đến đích, C là số chuỗi contact, TO là hết ngân sách, FP là số streak quá 5 s không giảm khoảng cách tốt nhất tới đích. Dùng epsilon 1 µm cho tiến triển và ngưỡng tốc độ 0,02 m/s để đếm stalled time kỹ thuật. PSC là tỷ lệ tick có khoảng cách tâm dưới 0,5 m. TTC dùng nghiệm va chạm hai đĩa với vận tốc tương đối world, ghi `null` khi không có nghiệm hữu hạn. Vận tốc lấy từ GT mỗi 50 ms. Gia tốc là chuẩn của sai phân vector vận tốc. Jerk là độ lớn của sai phân bậc hai của tốc độ tuyến tính theo nguồn. Continuity kiểm dịch chuyển với executed tích phân trên tick. Các ngưỡng chất lượng TTC 1,5 s và jerk 2,5 m/s³ giữ làm tham chiếu cho benchmark. Báo cáo này ghi số đo thô và chỉ dùng hard constraints cùng kiểm tra vật lý để phân loại. Toàn bộ ca FAIL còn trong CSV/JSON.

Chạy từ root repo với build 04S đã xác minh:

```bash
P=runs/task_04s/reproduction/venv/bin/python
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B \
  -m experiments.social_navigation.evaluate_sfm --output runs/task_05r
MPLBACKEND=Agg MPLCONFIGDIR=/tmp/task05r-mpl \
  env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B \
  -m experiments.social_navigation.analyze_sfm \
  --reference runs/task_04s/paired-final.json --output runs/task_05r
```

Không mở 06R/07R. 04S kiểm được API SFM. Chưa có kiểm định độc lập rằng state rebase và actor coupling phù hợp AVM hoặc CSM với nguyên phương trình native. [Deviation ledger](../../research/HUMAN_MODEL_DEVIATIONS.md) · [Tiến độ](PROGRESS.md)

## Kiểm thử và tệp thay đổi

Các lệnh pytest dùng `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY` và `python -B -m pytest`. `$P` là interpreter build 04S ở trên. Hai lượt toàn suite dùng cùng checkout.

| Kiểm định | Lệnh | Kết quả |
| --- | --- | --- |
| Regression 04S trước khi triển khai | `$P -B -m pytest -q tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py tests/integration/test_jupedsim_external_actor.py` | 23 passed. [Log](../../../runs/task_05r/preflight-regression.log) |
| Toàn suite, build 04S | `$P -B -m pytest -q` | 216 passed, 2 skipped. Test chỉ dành cho stock được đánh dấu theo distribution. [Log](../../../runs/task_05r/pytest-experimental-full.log) |
| Toàn suite, stock | `.venv/bin/python -B -m pytest -q` | 209 passed, 4 skipped. Kiểm tra từ chối backend trên stock đã chạy. [Log](../../../runs/task_05r/pytest-stock-full.log) |
| Fixture A–F và replay | Lệnh `evaluate_sfm` và `analyze_sfm` ở trên | 12 episode, 6 PASS, 6 FAIL theo hard constraints. Phép thử native một bước được lưu riêng. [Log](../../../runs/task_05r/fixtures-final.log) · [Phân tích](../../../runs/task_05r/analysis.log) |
| Lint và diff | `.venv/bin/ruff check src tests experiments/social_navigation` và `git diff --check` | Cả hai đạt. Lint toàn repo còn 7 lỗi có sẵn trong hai tệp `orca_progress_tools/`. [Lint giới hạn](../../../runs/task_05r/ruff-scoped.log) · [Lint toàn repo](../../../runs/task_05r/ruff-root.log) |

| Nhóm | Tệp tạo hoặc sửa trong 05R |
| --- | --- |
| Runtime | `src/narrow_nav/humans/backends/jupedsim_sfm.py`, `src/narrow_nav/humans/registry.py`, `src/narrow_nav/core/protocols.py`, `src/narrow_nav/simulation/lifecycle.py`, `configs/humans/sfm_jupedsim.yaml` |
| Kiểm định | `experiments/social_navigation/evaluate_sfm.py`, `experiments/social_navigation/sfm_metrics.py`, `experiments/social_navigation/analyze_sfm.py`, `tests/integration/test_sfm_research_backend.py`, `tests/integration/test_sfm_coupling_probe.py`, `tests/unit/test_sfm_research_metrics.py` |
| Tài liệu | `docs/reports/human_models/TASK_05R_REPORT.md`, `docs/reports/human_models/PROGRESS.md`, `docs/backends/SFM_JUPEDSIM.md`, `docs/decisions/ADR-0007-jupedsim-external-actor.md`, `docs/schemas/STATE_AND_ACTION_SCHEMA.md`, `docs/research/HUMAN_MODEL_DEVIATIONS.md` |
