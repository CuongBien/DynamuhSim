# TASK 07 · Nghiệm thu JuPedSim CollisionFreeSpeedModelV2

**Quyết định `PASS` theo cổng nghiệm thu backend TASK 07.** CSM V2 native, đồng bộ 2D–3D, reset/replay và dữ liệu truy nguyên đạt. Kết quả hành vi giữ riêng. Lượt tranh cửa 0,60 m và giao cắt người–người đối xứng chưa hoàn thành. [Tài liệu model](../../backends/CSM_JUPEDSIM_V2.md) · [Dữ liệu nghiệm thu](../../../runs/task_06_07/acceptance-evidence.json)

## Checklist nghiệm thu

| Tiêu chí TASK 07 | Kết quả |
| --- | --- |
| Đúng `CollisionFreeSpeedModelV2` và `CollisionFreeSpeedModelV2AgentParameters` | Đạt. Chữ ký kiểm từ runtime `1.4.2+task04u`, state native đúng loại V2. C++ khớp từng byte với archive pin, trace native 300 bước khớp wheel stock. |
| Tái dùng backend/config, geometry, radius, seed và adapter chung | Đạt. Dùng nguyên `jupedsim_csm.py` và `csm_jupedsim.yaml` từ 04U. Một backend mỗi episode. Không có đường hybrid hoặc logic CSM tự viết. |
| No-neighbor, head-on, crossing, near-wall và cửa 1,20 / 0,90 / 0,60 m | Đã chạy. Free và near-wall có tâm robot cách người trên cutoff native 3 m. Ca crossing người–robot đạt, crossing hai người đối xứng hết giờ. Ba cửa tuần tự đạt, ca tranh cửa 0,60 m người chưa qua. |
| Stop/Yield/Reverse | Đã chạy ở hành lang 2,1 m với cùng phần mở đầu, chỉ đổi lệnh trong `[4; 6)` s. Cả bốn nhánh CSM đạt. |
| GT, solver, clearance, tốc độ/gia tốc, jerk, deadlock, replay | Đạt kiểm định đo. Tái lập trace cùng seed. Giữ ca timeout, footprint bị chặn và probe contact thật. |
| Đối chứng ORCA/SFM/AVM | 12 cặp trên bốn khóa head-on, xác nhận seed, geometry, trạng thái đầu và lệnh robot giống nhau. Giữ lượt Reverse ORCA có clearance âm. |

Tham số resolved gồm `time_gap=1.0`, `desired_speed=1.2`, neighbor strength/range `8.0/0.1`, geometry strength/range `5.0/0.02`, radius người 0,25 m và robot 0,18 m từ scene. `v0` là tham số alias trong signature API và fixture dùng `desired_speed`. Robot là external actor trong index native. CSM V2 dùng XY và radius để tính spacing/repulsion, không dùng vận tốc actor. Sweep với 5 vận tốc, 3 seed và 3 cửa sổ cho Δ XY bằng 0 khi giữ XY/yaw. Rebase người cập nhật XY/hướng, velocity executed lấy riêng từ GT. [Hợp đồng 04U](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Nguồn CSM V2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/CollisionFreeSpeedModelV2.cpp) · [Sweep native](../../../runs/task_06_07/native-sensitivity.json)

## Kết quả người–robot trên MuJoCo

12 lượt seed 11, gồm một replay và một ca tranh cửa. PASS dùng `S=1,C=0,TO=0,FP=0`, clearance không âm và người vào dung sai 0,10 m ít nhất một tick. TTC theo đĩa tương đối. TTC 1,5 s và jerk 2,5 m/s³ là mốc chất lượng của dự án. Mọi lượt CSM trong bảng không có contact solver. Cửa khẩu độ đo từ collision MJCF của hai phòng fixture 04U. Crossing người–robot dùng tuyến vuông góc trong fixture mở. [Trace và manifest](../../../runs/task_06_07/csm/) · [Dữ liệu tổng hợp](../../../runs/task_06_07/acceptance-evidence.json)

| Fixture | Kết quả | Kết thúc / người vào đích (s) | Clearance robot–người / người–tường (m) | TTC min (s) | Δ dự báo–GT max (m) |
| --- | --- | --- | --- | --- | --- |
| Free | PASS | 17,20 / 5,30 | 2,570 / 1,390 | ∞ | 0,195 |
| Head-on Advance | PASS | 17,20 / 6,30 | 0,136 / 0,084 | 0,511 | 0,195 |
| Head-on Stop | PASS | 19,20 / 6,10 | 0,190 / 0,142 | 0,636 | 0,180 |
| Head-on Yield | PASS | 19,40 / 6,00 | 0,217 / 0,272 | 0,636 | 0,195 |
| Head-on Reverse | PASS | 20,60 / 6,05 | 0,203 / 0,188 | 0,636 | 0,195 |
| Crossing người–robot | PASS | 10,95 / 4,30 | 0,055 / 0,444 | 0,236 | 0,195 |
| Near-wall | PASS | 17,20 / 5,30 | 2,606 / 0,056 | ∞ | 0,195 |
| Cửa 1,20 m tuần tự | PASS | 9,35 / 6,00 | 0,092 / 0,300 | 0,308 | 0,195 |
| Cửa 0,90 m tuần tự | PASS | 9,35 / 6,00 | 0,091 / 0,151 | 0,306 | 0,195 |
| Cửa 0,60 m tuần tự | PASS | 9,35 / 6,20 | 0,080 / 0,040 | 0,281 | 0,195 |
| Head-on Advance replay | PASS | 17,20 / 6,30 | 0,136 / 0,084 | 0,511 | 0,195 |
| Cửa 0,60 m tranh lối | FAIL | 5,05 / — | 0,078 / 0,031 | 0,285 | 0,105 |

Near-wall có clearance đầu 0,080 m, nhỏ nhất 0,0563 m và dịch ngang tối đa 0,0403 m. Người đạt goal và giữ trong dung sai ở cuối các lượt PASS. Mọi trace có peak speed người 1,2 m/s, gia tốc GT tới 3,0 m/s² và jerk tới 60,0 m/s³ ở lưới 50 ms. Lệnh limited luôn ≤1,4 m/s và đổi tối đa 0,15 m/s theo chu kỳ 100 ms. Sai phân gia tốc/jerk trên GT 50 ms phản ánh bậc giữ lệnh và thực thi. [Số đo theo tick](../../../runs/task_06_07/csm/)

## Native dự báo và executed

1.718 cửa sổ 100 ms hoàn chỉnh. Từng callback giữ số bước native, vị trí và vận tốc đo tại boundary. So endpoint native sau 10 bước với MuJoCo tại cùng mốc. Sai lệch lớn nhất **0,19524 m** ở near-wall, tương đương chênh vận tốc trung bình 1,952 m/s. Trung bình theo episode ví dụ 0,01850 m ở near-wall và 0,04961 m ở cửa 0,60 m tuần tự. Không có guard trong 12 lượt CSM này. Sai lệch xảy ra sau bước dự báo và thực thi qua limiter. Nó khác sai số ghi/đọc rebase, vốn khớp tại boundary. `requested`, `limited`, `executed`, prediction windows, GT 10 Hz và contact substep được lưu riêng. [Hợp đồng thời gian](../../protocols/HUMAN_MODELS_TIMING_GEOMETRY.md) · [Trace](../../../runs/task_06_07/acceptance-evidence.json)

## Phân loại ca lỗi và hình học cửa

- Khẩu độ 1,20 / 0,90 / 0,60 m cho dư **đi riêng** 0,70 / 0,40 / 0,10 m và dư **đi song song** 0,34 / 0,04 / −0,26 m. Ở 0,60 m, đường tuần tự vẫn khả thi. Lịch robot giữ lệnh zero 2 s cho người qua tâm cửa ở 1,65 s, robot qua ở 7,05 s. Kết quả này kiểm thứ tự đã cấp, chưa chứng minh V2 tự thương lượng quyền đi trước.
- **Tranh cửa 0,60 m.** Robot tiến từ t=0 và qua cửa ở 2,75 s. Khi robot đạt đích ở 5,05 s, người chưa qua tâm cửa và còn cách đích 3,405 m. Không solver contact, không guard, khoảng không tiến triển người dài nhất 1,35 s. Lỗi là chưa hoàn thành mục tiêu người tại kết thúc episode, không có bằng chứng bế tắc quá 5 s. [Episode](../../../runs/task_06_07/contested/csm_jupedsim/door_060_contested_advance/episode.json)
- **Giao cắt hai người đối xứng.** Replay trùng 04U. Hết 8 s, mỗi người còn 1,776 m tới đích, không cải thiện khoảng cách trong 4,50 s. Clearance cuối 0,2235 m, không guard/contact. Requested, limited và executed cuối vẫn khớp ở khoảng 0,0646 m/s theo hướng chéo không đưa người tới đích. Timeout và không tiến triển được giữ. Khoảng đứng tiến triển chưa vượt mốc 5 s. [Timeout 04U](../../../runs/task_06_07/csm-04u-timeouts.json) · [Replay](../../../runs/task_06_07/crossing-replay.log)
- **Hai probe âm của 04U.** Hành lang 0,80 m có dư song song −0,06 m. CSM hết 4 s, clearance min 0,12816 m, không solver contact. Probe `desired_speed=0` cô lập đường đo solver và tái lập 20 mẫu robot–người, clearance −0,061514 m. Hai probe giữ nguyên tách khỏi tập hành vi. [Regression và bằng chứng](../../../runs/task_06_07/pytest.log)

Các giới hạn hành vi còn mở là giao cắt đối xứng, mục tiêu người ở ca tranh cửa và chất lượng TTC thấp trong một số lượt hợp lệ. Nghiệm thu backend không cấp kết luận an toàn cho các điều kiện này.

## Đối chứng ghép cặp

Mỗi cột là clearance robot–người nhỏ nhất (m). Cùng cặp dùng seed 11, cấu hình ngoài `humans` và MJCF fingerprint giống nhau, cùng pose đầu và lệnh robot theo thời gian. Phần prefix tới 4 s của bốn hành động trùng trong từng backend. Giữ nguyên profile backend đã công bố, gồm desired speed 1,2 m/s của AVM/CSM V2 và 0,8 m/s của SFM/ORCA cùng quy tắc actor riêng của ORCA. Sai khác trajectory thuộc toàn profile theo cặp, chưa cô lập hiệu ứng thuật toán hay có phân bố thống kê. [Khóa đầy đủ, nguồn và Δ quỹ đạo](../../../runs/task_06_07/acceptance-evidence.json)

| Hành động | CSM V2 | AVM | SFM | ORCA |
| --- | ---: | ---: | ---: | ---: |
| Advance | 0,136349 | 0,2688 | 0,043438 | 0,0003 |
| Stop | 0,1904 | 0,2690 | 0,079206 | 0,0002 |
| Yield | 0,2173 | 0,2887 | 0,069752 | 0,0017 |
| Reverse | 0,2029 | 0,2690 | 0,044384 | −0,0011, FAIL |

ORCA Reverse ghi clearance âm với 0 mẫu solver, vẫn là FAIL vật lý theo tiêu chí fixture. Không gộp ca này vào nhóm PASS. Δ quỹ đạo người lớn nhất giữa CSM và đối chứng trên các cặp lần lượt nằm trong 0,696–1,027 m với AVM, 1,809–2,192 m với SFM và 1,120–1,353 m với ORCA.

## Lệnh, kết quả và provenance

`P=runs/task_04u/reproduction/venv/bin/python`, lệnh Python chạy từ root với `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY`. Dùng thư mục output mới khi chạy lại fixture.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/csm --backend csm_jupedsim` | 0 | 11 lượt PASS. [Log](../../../runs/task_06_07/csm.log) |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/contested --backend avm_jupedsim --backend csm_jupedsim --scene door_060_contested` | 0 | Hai FAIL kịch bản. [Log](../../../runs/task_06_07/contested.log) |
| `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/avm --backend avm_jupedsim` và `$P -B -m experiments.social_navigation.evaluate_avm_csm --output runs/task_06_07/baselines --backend sfm_jupedsim --backend orca` | 0 | AVM 11 PASS. Baseline 7 PASS, 1 FAIL. [AVM](../../../runs/task_06_07/avm.log) · [SFM/ORCA](../../../runs/task_06_07/baselines.log) |
| `$P -B -m experiments.social_navigation.analyze_avm_csm --root runs/task_06_07 --historical runs/task_04u/final-fixtures` | 0 | 12 cặp hợp lệ, kiểm giới hạn/sync, phân tích timeout và xuất GT 10 Hz. [Kết quả](../../../runs/task_06_07/acceptance-evidence.json) |
| `$P -B -m pytest -q -rs -s --basetemp runs/task_06_07/bridge-fixtures tests/integration/test_avm_bridge_capability.py tests/integration/test_jupedsim_universal.py tests/unit/test_sfm_research_metrics.py` | 0 | 31 đạt, 2 skip dành cho runtime cũ. [Log](../../../runs/task_06_07/pytest.log) |
| `$P -B -m experiments.social_navigation.jupedsim_04u_parity` rồi `cmp` với `runs/task_04u/parity-stock.json` | 0 | V2 free walk native khớp từng byte. [Trace parity](../../../runs/task_06_07/parity-04u.json) |
| `.venv/bin/ruff check src tests experiments/social_navigation scripts/diagnostics`, `ruff format --check` trên hai script mới, `git diff --check` | 0 | Lint, format và diff đạt. 32 bộ CSV/JSON/GT và liên kết báo cáo đã đối chiếu. [Lint](../../../runs/task_06_07/ruff-scoped.log) |
| `.venv/bin/ruff check .` | 1 | 7 lỗi có sẵn trong hai tệp `orca_progress_tools/`. [Log](../../../runs/task_06_07/ruff-root.log) |

Nhánh `mujoco-hummodel`, HEAD `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, upstream local 0/0, working tree đã dirty trước task. Đọc MASTER và hai đặc tả task từ bundle trong `~/Downloads/CODEX_HUMAN_MODEL_TASKS`, AGENTS, nguồn, architecture/protocols/schemas, ADR và báo cáo 04U. Bản gốc và working copy `HUMAN_MODEL_FINAL.md` giữ cùng SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.

CPython 3.12.3, MuJoCo 4.0.0, JuPedSim `1.4.2+task04u`, NumPy 2.5.4, Shapely 2.2.0, `pyrvo2 0.0.0`. JuPedSim upstream commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`. Wheel SHA-256 `f3cf132b3987c368448c11913e1d11bb73e335032553e6222a043c0720437e3c`, extension `789bc8576238c4243e8ce393fcad882499ca03621434eb1ff0ef5313620e0e03`. Dùng `bash scripts/diagnostics/build_jupedsim_04u.sh runs/task_04s/jupedsim-1.4.2.tar.gz <thu_muc_moi>` để tái lập. Archive, hai patch, flags và lệnh cài đã pin tại 04U. [Provenance kiểm lại](../../../runs/task_06_07/provenance.log) · [Build 04U](TASK_04U_REPORT.md)

Tệp mới là hai script `experiments/social_navigation/{evaluate_avm_csm,analyze_avm_csm}.py`, tài liệu CSM V2 và báo cáo. Progress/deviation `hm_jps_007` cập nhật. Cấu hình, backend, wheel, phương trình và nguồn nghiên cứu giữ nguyên. Fixture không nhập vào chiến dịch benchmark.
