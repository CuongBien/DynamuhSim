# Các phần social navigation đã được commit

## Cập nhật22/10/2026 — cùng nhiệm vụ Nav2 cho ba framework

[Đối chiếu Nav2 và ảnh/GIF thật](benchmark/shared_nav2_pilot/reports/SHARED_NAV2_REPORT.md): đã chạy thêm2 CrowdSimPlus ORCA+ và2 ComplexityNav geometry, cùng6 người/cảnh/start-goal/MPPI/AMCL như2 SocNavGym v2 đã commit. Crowd01 đạt; crowd02 action SUCCEEDED nhưng world XY20.59cm vượt20cm nên giữ không đạt. Cả2 CN geometry đạt. Bốn lượt mới không ghi overlap>5mm theo metric, nhưng robot–người có lúc chỉ2.88–5.80cm, chưa chứng nhận social comfort.

YAML Nav2 byte-identical; world giống sau chuẩn hóa socket/native dt. CSPdt0.1s vàCNdt0.25s, native speed/observation khác nhau được giữ; không xếp hạng framework hoặc suy success rate từ2 lượt/model. GPU NVIDIA/cleanup đạt4 lượt, không sửa ROS/Nav2/vendor. Một assertion analyzer do pose tail trongshutdown đã sửa; không chạy thay simulator hoặc nâng ngưỡng. Source cũ,staging12 mục vàoutside worktree bảo toàn.

Cả3 nhánh đã có Gazebo vàpilot Nav2; vẫn ưu tiên SocNavGym lifecycle cho nhóm, CSP ORCA+ baseline cá nhân, CN geometry hybrid nghiên cứu (không trained RGL). **Bước tiếp: pilot SocNavGym nhóm+Nav2 cho nhóm đi vànhóm đứng trò chuyện**, đo goal thật/clearance/giữ-tan nhóm rồi chốt bộdemo vàlựa chọn mô hình. Chưa nghiệm thu nhóm+Nav2 hoặcdense crowd; không World Model/Isaac. Mốc này lưu bằng `feat: compare crowd and ComplexityNav on shared Nav2 warehouse task`.

Các mục dưới giữ lịch sử theo thời điểm đo.

## Cập nhật22/10/2026 — pilot Nav2 với SocNavGym v2

[Nav2 pilot đã đo](benchmark/socnavgym_nav2_pilot/reports/NAV2_PILOT_REPORT.md):1 empty control và2 lượt6 người giao cắt đều Nav2 SUCCEEDED, actual XY<=0.20m/yaw<=0.25rad. Hai lượt có người sai số XY17.86cm/14.54cm, min robot–human nominal disc gap6.18cm/19.71cm; không ghi overlap>5mm theo metric. Robot từ(−28,0) tới(−20,0), thực sự do Nav2 MPPI điều khiển; native người gate trongwarmup, bắt đầu sau accepted goal.

Không sửa robot/Nav2 baseline/native/vendor; chỉ AMCL initial pose trongruntime copy. GPU NVIDIA được kiểm ở3 lượt, people RTF0.24–0.40 nên không gọi realtime hoặc benchmark tốc độ thuần. Source/staging ngoài experiment giữ nguyên. Mốc này dùng commit message `feat: validate SocNavGym v2 with bounded Nav2 warehouse pilot`. Bước kế: cùng bài Nav2 cho CrowdSimPlus ORCA+ và ComplexityNav geometry, rồi đối chiếu; SocNavGym group/formation+Nav2 cần pilot riêng. Không World Model/Isaac Sim.

Các mục dưới giữ kết quả các vòng trước theo thời điểm đo.

## Cập nhật22/10/2026 — sửa SocNavGym response v2

Baseline9 lượt Gazebo đã commit **`9ab3c9f`** (`feat: benchmark shared Gazebo scenes and retain SocNavGym crossing failure`). [Bản sửa response v2](benchmark/socnavgym_response_fix/reports/RESPONSE_FIX_REPORT.md) nằm trong experiment riêng:4 lượt bình thường hoàn tất (2 giao cắt,đối đầu,lối hẹp),2 test worker lỗi/socket loss đạt Pause interlock,6/6 offline và IPC exact20-step. Hai lượt crossing giữ khoảng hở robot–người đo thực khoảng4cm và không còn lỗi5.5s trong cửa sổ10s đã đo.

Plugin Pause world và phát zero cmd_vel khi terminal fault; hai test có robot drift bằng0 theo samples, Play chỉ tiến1tick2.5ms rồi Pause lại. Đây là simulation interlock, không chứng nhận phanh robot thật. Không sửa ROS/Nav2/vendor hoặc baseline cũ, GPU NVIDIA được kiểm. Bằng chứng này cho6 cá nhân/seed42, không tự chuyển thành nghiệm thu formation/standing groups. Bước tiếp: pilot Nav2 nhỏ dùng v2, kiểm robot chủ động tránh/nhường trước benchmark navigation. Không World Model/Isaac Sim. Mốc sửa dùng commit message `fix: resolve SocNavGym crossing proposals and pause Gazebo on worker faults`.

Các mục dưới đây ghi lại những vòng trước theo thời điểm đo.

## Cập nhật22/10/2026 — cohort Gazebo chung

Offline36 ca đã commit **`ff79dbf`** (`feat: benchmark shared offline social navigation scenarios`). [Cohort Gazebo chung](benchmark/common_gazebo/reports/COMMON_GAZEBO_REPORT.md) đã thực hiện9 lượt (3 cấu hình ×3 near scenes,6 người/radius0.36m):8 hoàn tất10s; SocNavGym guarded crossing lỗi terminal tại5.5025s. Có actual poses/LiDAR/camera thật, GPU NVIDIA, shutdown/cleanup; không có Nav2 hoặc formation trong cohort này.

Soc crossing còn gap+5.12mm trước fault nhưng robot trôi khi teardown gây signed gap−10.83cm. Chưa nghiệm thu cấu hình này cho tương tác sát robot/Nav2; bước kế là sửa proposal né robot và dừng robot khi worker fault trong phiên bản mới, tái kiểm đúng ca lỗi. Giữ SocNavGym cho mục tiêu nhóm theo bằng chứng lifecycle trước, CrowdSimPlus ORCA+ làm baseline và ComplexityNav geometry làm biến thể phụ. Không suy từ8 lượt hoàn tất ra an toàn/tự nhiên mọi cảnh. Kết quả Gazebo mới chưa commit; không sửa code robot/vendor hay staging ngoài experiment.

Các commit dưới đây lưu từng phần công việc đã hoàn tất, theo thứ tự phụ thuộc.
Không bao gồm các thay đổi cleanup/di chuyển file hoặc phần staged có sẵn
ngoài `experiments/social_navigation/`. Các mốc chẩn đoán bổ sung ở cuối trang.

| Commit | Phần công việc | Bằng chứng |
|---|---|---|
| `5a14b38` | SocNavGym: nhóm có tránh robot, opt-in adapter | [Report/GIF](benchmark/avoidance_comparison/reports/run01/AVOIDANCE_REPORT.md) |
| `e288ff6` | CrowdSimPlus: backend Gazebo độc lập | [Report/GIF](benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md) |
| `2106ec2` | Native SFM: selector và benchmark đối chiếu ORCA+ | [Report/GIF](benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md) |
| `fa9738a` | Endpoint tuần tra riêng cho mỗi người | [Report/GIF](benchmark/individual_goals/reports/run01/INDIVIDUAL_GOALS_REPORT.md) |
| `c970154` | Người đi giao cắt trong Gazebo | [Report/GIF](benchmark/crossing/reports/run01/CROSSING_REPORT.md) |
| `df4841e` | Lối hẹp và bằng chứng failure định vị | [Report/GIF](benchmark/bottleneck/reports/run01/BOTTLENECK_REPORT.md) |

Các report và metadata giữ nguyên provenance khi chạy: HEAD lúc đo là
`954d577` với các thay đổi chưa commit trong working tree. Các dòng trạng thái
“chưa commit” trong báo cáo là trạng thái **ở thời điểm đo**. Việc chia commit
sau đó không có nghĩa đã chạy lại thí nghiệm trên từng commit trung gian.
Một số plumbing kernel chung nằm ở commit backend; commit SFM bổ sung selector
benchmark và kết quả đánh giá riêng.

Đã kiểm tra cú pháp Python/shell của các phiên bản đưa vào từng phần, phạm vi
file của từng commit và sự khớp chính xác giữa source cuối cùng với working
files đã đo. Native config giữ byte-for-byte, kể cả blank line cuối do
ConfigParser tạo. Log thô, runtime/generated và build products không đưa vào
các commit mới; report, cấu hình, số đo, ảnh/GIF và manifest được lưu.

Kết quả quan trọng: crossing có 4/4 goal đạt. Bottleneck có empty control
qua khe, nhưng chỉ 2/4 goal có người đạt; hai failure seed42 có yếu tố AMCL/
planner, ORCA+ có max position error 2,23 m. Không dùng tỷ lệ này để xếp hạng
framework. Native CrowdSimPlus ở đây vẫn là ba người độc lập; ComplexityNav
chưa nối Gazebo. Không triển khai World Model hay chuyển Isaac Sim.

Bước chẩn đoán nêu trên đã được thực hiện trong các commit bổ sung dưới đây.

| Commit bổ sung | Nội dung | Bằng chứng |
|---|---|---|
| `1186e1d` | Offline AMCL/scan/TF/costmap và native sensor cross-check | [Report](benchmark/localization_diagnosis/reports/run01/LOCALIZATION_DIAGNOSIS.md) |
| `e8bf1c0` | Global navigation/particle capture và hai diagnostic reruns | [Report](benchmark/navigation_diagnostics/reports/run03/NAVIGATION_DIAGNOSTICS.md) |
| `2f2bdff` | Hardware GPU preflight và child-process cleanup | [Report](benchmark/gpu_runtime/reports/run01/GPU_RUNTIME.md) |

Các report bổ sung giữ HEAD tại lúc đo là8dc9913 và trạng thái chưa commit
lúc đó; không có nghĩa đã đo lại trên từng commit trung gian. Hai Ogre logs
GPU được lưu ngoại lệ dưới dạng raw evidence; runtime/build logs khác vẫn ignored.

Sau ba commit, [sáu lượt stability](benchmark/stability/reports/run01/STABILITY_REPORT.md)
chạy trên HEAD2f2bdff, source cố định: SFM2/3, ORCA+1/3. Đã thu mode switch
và global reroute, có human proxy overlap. Phần stability mới chưa commit;
chưa sửa Nav2/human model hoặc bắt đầu framework tiếp theo.
Bước tiếp theo đã chạy trong working tree, chưa commit mới: [workaround shutdown WSL D3D12](benchmark/shutdown_fix/README.md), ba lifecycle checks (ORCA/SFM/SocNavGym) và [sáu lượt AMCL ablation](benchmark/localization_trials/reports/run01/LOCALIZATION_PILOT.md). Gazebo thoát sạch ở cả ba checks và sáu pilot; tất cả xác nhận fresh Ogre NVIDIA, cleanup không còn owned process/container. Bộ kiểm chứng giữ nguyên Git index và worktree ngoài social_navigation.

Pilot mới giữ từng kết quả: baseline SFM ABORTED, baseline ORCA tới đích; LF-prob SFM tới đích nhưng ORCA báo SUCCEEDED khi vị trí thật còn cách2.597m; LF-prob+beam-skip tới đích cả SFM/ORCA, vị trí thật lệch0.119/0.140m. Chỉ một lượt mỗi ô, chưa promote profile. Dấu âm khe người–người vẫn lưu, lớn nhất khoảng5mm; không đổi native policy. 1009/1009 cloud đối chiếu cluster khớp AMCL sau circular-mean lift. Shutdown workaround chỉ opt-in `--shutdown-mode wsl_d3d12`, default lịch sử và code ROS gốc được giữ.

Các phần nêu trên đã được commit riêng trước cohort nhiều seed mới:

| Commit | Phần hoàn tất |
|---|---|
| `25e41573` | `feat: record repeated bottleneck stability and localization failures` |
| `78bf3905` | `fix: keep WSL D3D12 resident for clean Gazebo shutdown` |
| `d0a7bbb2` | `feat: benchmark isolated AMCL likelihood and beam skipping profiles` |

Các dòng “chưa commit” ở các đoạn/report phía trước mô tả thời điểm đo. Không chạy lại các cohort cũ để thay provenance. Đã bảo toàn12 mục staged của người dùng ngoài experiment.

Sau các commit này, [cohort12 lượt seed42/43/44](benchmark/localization_validation/README.md) được tạo riêng để so baseline với LF-prob+beam-skip; không thay kết quả pilot6 lượt. Seed44 được sinh mới theo cùng thuật toán bottleneck, seed42/43 giữ nguyên scene byte-for-byte; native policy và robot controller không đổi.

Cohort12 lượt sau commit đã kết thúc và kiểm định: baseline5/6, LF-prob+beam-skip5/6 goal thật,1998 cloud đối chiếu,12/12 shutdown sạch/NVIDIA,24 ảnh/GIF decode hợp lệ. Giữ cả hai ABORTED, không retry. Không chọn profile mặc định mới. Source/kết quả cohort12 mới vẫn chưa commit; không sửa các outcome trước đó.

Cohort12 lượt được lưu riêng cùng runner, mở rộng fixture seed44, analyzer, ảnh/GIF và validation. Provenance vẫn là HEADd0a7bbb tại lúc chạy cộng source hashes; commit lưu kết quả sau đó không phải một lần chạy lại. [Kế hoạch tiếp theo](NEXT_STEPS.md) ưu tiên6 probe replay chung cho ORCA+/SFM, rồi quay lại nhóm SocNavGym; chưa thực hiện các bước mới và chưa đổi AMCL mặc định.


Các probe hành vi và bản sửa observer đã được lưu riêng:

| Commit | Phần hoàn tất |
|---|---|
| `b3f59e2a` | `feat: benchmark native human responses with shared robot replay inputs` |
| `195f4e85` | `fix: observe retained AMCL poses in isolated Gazebo trials` |

[Replay](benchmark/human_response_replay/README.md) gồm6 CrowdSimPlus +18 SocNavGym, đầu vào robot định trước, native parity và repeatability đạt. Bản sửa AMCL chỉ đổi QoS subscriber theo tùy chọn trong harness; không reset localization hoặc đổi ROS robot. Các report giữ HEAD lúc chạy và source hashes. Native ConfigParser output giữ nguyên blank line cuối để bảo toàn byte và SHA256; kiểm tra whitespace chỉ miễn blank-at-eof cho các artifact này.

Phần commit tiếp theo lưu [Gazebo nhóm](benchmark/group_gazebo_demo/README.md): ba lượt standing và ba lượt counterflow hợp lệ, một startup error gốc được giữ. Sáu action SUCCEEDED, năm lượt đạt sai số thật≤0.20m; GUI kéo/zoom/Pause/Play được người dùng xác nhận. Nhóm cố định ba người, chưa có lifecycle formation trong Gazebo.

Commit `e749fb51` đã lưu phần Gazebo nhóm và GUI nêu trên. Sau commit, [kiểm định lifecycle độc lập](benchmark/socnavgym_lifecycle/README.md) chạy trên HEAD này với source hashes riêng; báo cáo mới chưa commit. Ba seed đều giữ ID/reset contract, nhưng hình học formation chưa đạt để promote vào Gazebo.


Cập nhật22/10/2026, **chưa có commit mới**: bản sửa lifecycle standalone và [pilot formation trong nhà xưởng Gazebo](benchmark/socnavgym_warehouse_lifecycle/README.md) đã hoàn tất trong working tree. Ba trace offline đạt; hai lượt live120s giữ riêng, seed42 PARTIAL về lifecycle và seed43 PASS gồm nhóm đứng/đi/tan. Adapter mới không sửa vendor, code robot hoặc adapter nhóm cố định. Report giữ HEADe749fb51 và source hashes tại lúc đo; ảnh/GIF từ camera Gazebo thật,4×. Các file lifecycle mới sẽ cần commit riêng khi người dùng yêu cầu.


Các phần lifecycle nói trên đã được lưu riêng trước khi bắt đầu ComplexityNav:

| Commit | Nội dung |
|---|---|
| `248015807a` | `test: record native SocNavGym formation and reset limitations` |
| `779c338892` | `fix: preserve continuous SocNavGym group formation motion` |
| `b3f0022b26` | `feat: validate six-person SocNavGym lifecycle in Gazebo factory` |

Commit qua index riêng, sau đó chỉ đồng bộ các path experiment đã commit vào index thật;12 mục staged ban đầu và outside worktree giữ byte-for-byte. Source/report giữ HEAD/source hashes lúc đo, không giả định đã chạy lại trên ba commit này. Bước [contract ComplexityNav](benchmark/complexitynav_contract/README.md) thực hiện sau đó và chưa commit: contract đạt nhưng native geometry xuyên kệ, nên chưa nối Gazebo.


Commit `b0b3c336` — `test: validate ComplexityNav contracts and expose missing factory geometry` — đã lưu preflight trước khi làm adapter mới; index12 mục và worktree ngoài experiment giữ nguyên. Sau commit này, [adapter hình học ComplexityNav](benchmark/complexitynav_geometry/README.md) mới chạy6 cặp native/variant và reset80 bước, đạt offline nhưng chưa Gazebo. Phần adapter/báo cáo mới đang chưa commit; provenance HEAD/source hash tại lúc chạy giữ riêng.


Commit `be816c3da3` — `feat: route ComplexityNav humans around factory obstacles` — đã lưu adapter offline trước pilot. Sau commit, [ComplexityNav Gazebo pilot](benchmark/complexitynav_gazebo/README.md) đã chạy4 người/60s và fault-freeze test, đạt proxy/LiDAR/clock/NVIDIA/GUI/cleanup; pilot mới chưa commit. Các SHA/HEAD lúc đo được giữ, không gọi đây là lần chạy trên commit lưu kết quả sau này. Staging12 mục của người dùng và outside worktree vẫn giữ nguyên.


Commit `c9dffe25` — `feat: validate ComplexityNav Gazebo pilot and terminal worker faults` — đã lưu pilot trước bộ so sánh chung. Raw Ogre/worker console giữ byte-identical, whitespace checks chỉ miễn trailing/blank-at-eof cho4 file log raw, không sửa log để passcheck; source khác kiểm nghiêm. Sau commit, [cohort chung offline36 lượt](benchmark/common_response/README.md) đã chạy đủ6 nhánh và18 near/far pairs, còn chưa commit. Đây không phải Gazebo/Nav2 benchmark chung; metadata ghi rõ native observation/speed và variant khác nhau.
