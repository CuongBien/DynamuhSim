# Các phần social navigation đã được commit

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
