# TASK 06 · Nghiệm thu hành vi AVM

**Quyết định `PARTIAL`.** Tích hợp AVM native trên wheel 04U đạt. Cổng `BEHAVIOR_PASS` còn vướng giao cắt người–người đối xứng, tái lập tình trạng hai người không tiến triển trước khi hết giờ. Cửa 0,60 m khi hai bên cùng khởi hành cũng kết thúc trước khi người đến đích. [Báo cáo tích hợp 04U](TASK_04U_REPORT.md) · [Dữ liệu nghiệm thu](../../../runs/task_06_07/acceptance-evidence.json)

## Checklist nghiệm thu

| Tiêu chí TASK 06 | Kết quả |
| --- | --- |
| Điều kiện bridge, exact `AnticipationVelocityModel` và `AnticipationVelocityModelAgentParameters` | Đạt trên 04U. Xác minh class/state khi chạy, chữ ký installed và C++ trùng archive. Dùng nguyên backend, Hydra, bridge, limiter và geometry. |
| Tham số và provenance | Manifest lưu `desired_speed=1.2`, `time_gap=1.06`, `anticipation_time=1.0`, `reaction_time=0.3`, `wall_buffer_distance=0.1`, neighbor strength/range `8.0/0.1`, pushout `0.3`, actor horizon `1.0` và radius người 0,25 m từ scene. RNG lấy seed luồng `human_backend`. |
| Nhận robot động và causality | Đạt. Giữ XY/yaw và chỉ đổi vận tốc robot cho đáp ứng khác trước tiếp xúc. Readback khớp GT tại boundary. Advance/Stop/Yield/Reverse trùng lịch sử trước 4 s khi giữ cùng backend. |
| End-to-end free, head-on, crossing, blind corner và doorway | 11/12 lượt người–robot đạt tiêu chí fixture, gồm một replay. Giao cắt người–người có đáp ứng native trước tiếp xúc nhưng lượt MuJoCo đối xứng hết giờ. Cửa tranh lối 0,60 m giữ kết quả FAIL. |
| GT, contact, clearance, giới hạn, TTC, jerk và replay | Đạt kiểm định dữ liệu. Hai replay độc lập trùng trace/contact theo phép thử tương ứng. Jerk và nguy cơ TTC được ghi riêng khỏi hard constraints. |
| ORCA/SFM đối chứng | Bốn khóa head-on, seed 11, cùng geometry, trạng thái đầu và lịch lệnh robot, kèm đường dẫn dữ liệu gốc. Giữ profile backend đã công bố, gồm 1,2 m/s của AVM và 0,8 m/s của SFM/ORCA. ORCA Reverse có clearance âm, giữ trong kết quả. |

[API và tham số AVM](../../backends/AVM_JUPEDSIM.md) · [Hợp đồng 04U](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Nguồn AVM v1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp)

## Độ nhạy vận tốc robot

Phép thử native 2D lấy state kỹ thuật từ probe 04U. Robot ở `(1,20; 0,15)` m, yaw `π−0,2`, radius 0,18 m, giữ mẫu qua 10 bước 10 ms. Người bắt đầu tại `(0,05; 0)` m, vận tốc `(0,4; 0)` m/s, radius 0,25 m. Mỗi nhánh chỉ đổi `vx` robot. Bảng ghi chuẩn chênh XY so với `vx=0`. Các seed episode 11, 23, 47 cho cùng số trong fixture này. Mỗi nhánh lặp lại chính xác. Clearance nhỏ nhất trong sweep là 0,6638 m.

| `vx` robot (m/s) | Δ XY người 10 ms (mm) | 50 ms (mm) | 100 ms (mm) |
| ---: | ---: | ---: | ---: |
| −0,70 | 0,084 | 0,411 | 0,808 |
| −0,35 | 0,084 | 0,411 | 0,808 |
| 0 | 0 | 0 | 0 |
| +0,35 | 0,103 | 1,709 | 6,406 |
| +0,70 | 0,106 | 2,387 | 9,200 |

Hai mức tiến gần trùng đáp ứng trong cấu hình này. Kết quả xác nhận độ nhạy trước overlap, không cung cấp quan hệ tuyến tính theo vận tốc hay độ phủ RNG ở các điểm rẽ đối xứng. Cơ chế native dùng vận tốc và hướng hiện tại để dự đoán, với khoảng cách dự báo bị chặn về 0 khi âm. [Dữ liệu sweep](../../../runs/task_06_07/native-sensitivity.json) · [Phép tính native](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp#L251-L289)

## Kết quả người–robot trên MuJoCo

Các lượt dùng seed 11. Head-on dài 10 m, rộng 2,1 m và dùng lịch 05R. Đổi lệnh robot trong `[4; 6)` s rồi tiếp tục tiến. Yield dạt ngang và trở làn. Crossing dùng tuyến vuông góc trong fixture mở. Blind corner dùng world chữ L của repo và lịch lệnh theo thời gian. Đường ngắm XY bị che lúc reset, thấy lần đầu ở 0,85 s. Cửa dùng hai phòng từ fixture 04U, đo khẩu độ từ MJCF biên dịch. Lịch tuần tự dừng robot 2 s rồi tiến. Lượt tranh cửa cho hai bên cùng khởi hành với spawn đã khai báo riêng. Các fixture này giữ tách khỏi Card benchmark.

| Fixture | Kết quả | Kết thúc / người vào đích (s) | Clearance robot–người / người–tường (m) | TTC min (s) | Δ dự báo–GT max (m) |
| --- | --- | --- | --- | --- | --- |
| Head-on Advance | PASS | 17,20 / 5,40 | 0,269 / 0,271 | 1,363 | 0,197 |
| Head-on Stop | PASS | 19,20 / 5,40 | 0,269 / 0,270 | 1,363 | 0,199 |
| Head-on Yield | PASS | 19,40 / 5,40 | 0,289 / 0,272 | 1,363 | 0,199 |
| Head-on Reverse | PASS | 20,60 / 5,45 | 0,269 / 0,270 | 1,363 | 0,199 |
| Free | PASS | 17,20 / 5,30 | 2,570 / 1,390 | ∞ | 0,195 |
| Crossing người–robot | PASS | 10,95 / 4,00 | 0,505 / 0,502 | 1,388 | 0,198 |
| Blind corner | PASS | 18,15 / 7,90 | 0,228 / 0,248 | 0,709 | 0,195 |
| Cửa 1,20 m | PASS | 9,35 / 4,60 | 0,500 / 0,164 | 1,399 | 0,194 |
| Cửa 0,90 m | PASS | 9,35 / 4,75 | 0,411 / 0,117 | 1,046 | 0,193 |
| Cửa 0,60 m tuần tự | PASS | 9,35 / 5,65 | 0,503 / 0,000583 | 1,194 | 0,206 |
| Head-on Advance replay | PASS | 17,20 / 5,40 | 0,269 / 0,271 | 1,363 | 0,197 |
| Cửa 0,60 m tranh lối | FAIL | 5,05 / — | 0,422 / 0,000070 | 1,214 | 0,180 |

12 lượt không ghi contact solver, timeout robot hay `FP>0`. PASS fixture đòi hard constraints của robot và người vào dung sai 0,10 m ít nhất một lần. Ở ba cửa tuần tự, người AVM từng vào dung sai nhưng cuối lượt cách đích **0,150–0,228 m**. Vào đích và duy trì trạng thái dừng là hai kết quả khác nhau. Cửa 0,60 m có dư đi riêng +0,10 m và dư song song −0,26 m. Lượt tranh lối có 1.274 substep guard, người chưa qua tâm cửa và cách đích 3,433 m khi robot tới đích. Khoảng không tiến triển người dài nhất 2,50 s, chưa tới mốc 5 s. [Trace, GT và kết quả](../../../runs/task_06_07/acceptance-evidence.json)

## Native dự báo và executed

So cùng cửa sổ 100 ms giữa native sau 10 bước và MuJoCo tại `t+100 ms`. Loại phần cửa sổ cuối thiếu snapshot đủ kỳ khỏi phép so, giữ tick cuối trong episode. Có 1.727 cửa sổ hoàn chỉnh, Δ XY tối đa 0,20625 m, tương đương Δ vận tốc trung bình 2,0625 m/s ở cửa 0,60 m. XY/vận tốc đầu callback khớp GT, bước native khớp lịch và requested bằng sai phân dự báo. Executed từ sai phân pose MuJoCo. Mọi lệnh limited tuân 1,4 m/s và Δ vận tốc tối đa 0,15 m/s mỗi callback; peak speed người 1,2 m/s. Nhịp 50 ms và giữ lệnh 100 ms cho peak jerk thường khoảng 60 m/s³. Cửa 0,60 m có peak gia tốc GT **12,02 m/s²** và jerk **445,51 m/s³** ở tick 1,65 s, có 8 substep guard trong vùng ±100 ms. Đây là chuyển động đo sau guard, với giới hạn gia tốc áp trên lệnh limited. TTC theo đĩa tương đối, jerk theo sai phân bậc hai của tốc độ. Ngưỡng 1,5 s và 2,5 m/s³ dùng làm tham chiếu chất lượng. [Bản ghi theo episode](../../../runs/task_06_07/avm/) · [Định nghĩa](../../schemas/EPISODE_SCHEMA.md)

## Phân loại timeout và blocker

- **Ba lượt advance/stop/replay 3 s của 04U.** Đây là cửa sổ chẩn đoán. Người còn 1,143–1,157 m tới đích, vẫn tiến và không có guard hay contact. Kết thúc theo ngân sách đã định.
- **Đối đầu người–người 8 s của 04U.** Robot đỗ xa trong toàn lượt. Hai người vào đích ở 3,85 s. `time_budget` của lifecycle không cản kết quả thành công của fixture người–người.
- **Giao cắt người–người 8 s của 04U.** Lần chạy lại trùng trace và kết quả. Guard bắt đầu ở tick kết thúc 2,10 s, tổng 2.743 substep, mỗi người không cải thiện khoảng cách trong 5,25 s. Cuối lượt, hai người còn cách đích 2,204 m. Native và limited vẫn đề xuất khoảng 0,480 m/s mỗi người, executed bằng 0. Tâm dự báo cuối kỳ cách nhau 0,499803 m, dưới tổng radius 0,500000 m. Khoảng hở thực thi còn 2,48 µm, không có contact solver. Bằng chứng cho thấy đề xuất đóng vòng vi phạm khoảng hở và guard chặn thực thi. Chưa cô lập đóng góp riêng của tính đối xứng, giữ mẫu và limiter vào sự hình thành bế tắc. Native 2D test có đáp ứng trước tiếp xúc nhưng không giải quyết ca đóng vòng này. [Phân tích timeout](../../../runs/task_06_07/avm-04u-timeouts.json) · [Replay](../../../runs/task_06_07/crossing-replay.log)
- **Contact thật ở ca âm 0,80 m của 04U.** Regression tái lập 9 mẫu solver và clearance −0,062310 m. Đây là thất bại an toàn riêng, không phải timeout hoặc lỗi runtime. [Log](../../../runs/task_06_07/pytest.log)

Cổng còn lại là hoàn thành giao cắt người–người đối xứng trong MuJoCo và phân tích nguyên nhân khiến AVM không ổn định gần đích cửa. Các quan sát hiện tại chưa biện minh sửa backend hay phương trình.

## Lệnh, kết quả và provenance

`P=runs/task_04u/reproduction/venv/bin/python`, lệnh Python chạy từ root với `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY`. Dùng thư mục output mới khi chạy lại fixture.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/avm --backend avm_jupedsim` | 0 | 11 lượt PASS. [Log](../../../runs/task_06_07/avm.log) |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/contested --backend avm_jupedsim --backend csm_jupedsim --scene door_060_contested` | 0 | Hai FAIL kịch bản, trace giữ nguyên. [Log](../../../runs/task_06_07/contested.log) |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/baselines --backend sfm_jupedsim --backend orca` | 0 | 7 PASS, 1 FAIL. [Log](../../../runs/task_06_07/baselines.log) |
| `$P -B -m experiments.social_navigation.analyze_avm_csm --root runs/task_06_07 --historical runs/task_04u/final-fixtures` | 0 | Sweep 90 số đo cho hai model, audit timestamp/limiter/sync, replay và 12 cặp CSM đối chứng; xuất GT 10 Hz. [Kết quả](../../../runs/task_06_07/acceptance-evidence.json) |
| `$P -B -m pytest -q -rs -s --basetemp runs/task_06_07/bridge-fixtures tests/integration/test_avm_bridge_capability.py tests/integration/test_jupedsim_universal.py tests/unit/test_sfm_research_metrics.py` | 0 | 31 đạt, 2 skip chỉ dành cho runtime cũ. [Log](../../../runs/task_06_07/pytest.log) |
| `$P -B -m experiments.social_navigation.jupedsim_04u_parity` rồi `cmp` với `runs/task_04u/parity-stock.json` | 0 | Trùng từng byte. [Trace parity](../../../runs/task_06_07/parity-04u.json) |
| `.venv/bin/ruff check src tests experiments/social_navigation scripts/diagnostics`, `ruff format --check` trên hai script mới, `git diff --check` | 0 | Lint, format và diff đạt. 32 bộ CSV/JSON/GT và liên kết báo cáo đã đối chiếu. [Lint](../../../runs/task_06_07/ruff-scoped.log) |
| `.venv/bin/ruff check .` | 1 | 7 lỗi có sẵn trong hai tệp `orca_progress_tools/`. [Log](../../../runs/task_06_07/ruff-root.log) |

Nhánh `mujoco-hummodel`, HEAD `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, upstream local 0/0, working tree đã dirty trước task. Đọc MASTER và `TASK_06_AVM.md`/`TASK_07_CSM.md` từ bundle `~/Downloads/CODEX_HUMAN_MODEL_TASKS`, AGENTS, nguồn, architecture, protocols, schemas, ADR và báo cáo 04U. Bản gốc và working copy `HUMAN_MODEL_FINAL.md` có cùng SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.

Runtime CPython 3.12.3, MuJoCo 4.0.0, JuPedSim `1.4.2+task04u`, NumPy 2.5.4, Shapely 2.2.0, `pyrvo2 0.0.0`. JuPedSim upstream commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`. Wheel SHA-256 `f3cf132b3987c368448c11913e1d11bb73e335032553e6222a043c0720437e3c`, extension `789bc8576238c4243e8ce393fcad882499ca03621434eb1ff0ef5313620e0e03`. Tái lập wheel bằng `bash scripts/diagnostics/build_jupedsim_04u.sh runs/task_04s/jupedsim-1.4.2.tar.gz <thu_muc_moi>`. Archive, hai patch, cờ build và lệnh cài đã ghi tại 04U. [Provenance kiểm lại](../../../runs/task_06_07/provenance.log) · [Build 04U](TASK_04U_REPORT.md)

Tệp thực thi mới `experiments/social_navigation/{evaluate_avm_csm,analyze_avm_csm}.py`. Cập nhật báo cáo lịch sử, tài liệu AVM, progress và deviation ledger `hm_jps_007`. Backend, YAML model, wheel, nguồn nghiên cứu và benchmark giữ nguyên.
