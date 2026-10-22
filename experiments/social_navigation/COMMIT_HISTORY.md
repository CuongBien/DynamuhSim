# Các phần social navigation đã được commit

## Cập nhật 22/10/2026 — đã thu và kiểm chứng seed 43 mới

[Seed43 report](benchmark/depth_seed43_validation/reports/SEED43_REPORT.md): một recording mới trên NVIDIA, cùng layout/depth mesh/Nav2, đổi seed 42→43; frozen engine trước capture, không tune. Có 1051 depth khớp raw bitwise, 802 cửa sổ/1 episode, dữ liệu/cleanup/audit đạt; quỹ đạo người thực tế khác seed42. **Navigation TIMEOUT 90.025 s, goal còn 3.7013 m**, không đổi thành success.

Combined giữ 1983 paired rows, unpaired 900→268, paired ratio 68.78→88.09%; fragmentation 9→9 và còn một pairing change nên **gate kết hợp FAIL**. Chưa nghiệm thu re-ID/semantic/association/train, baseline cũ giữ nguyên. Case GT5→GT6 ở 13.5→13.9 s đã lưu LABEL ONLY cho regression; nếu dùng để tune bản sau phải kiểm trên seed mới khác. Không thu batch hoặc sửa ROS; HuNavSim chưa dùng.

Mốc lưu: `feat: validate frozen depth tracking on a new seed`.

## Cập nhật 22/10/2026 — depth track refinement có giới hạn profile

[Refinement report](benchmark/depth_track_refinement/reports/REFINEMENT_REPORT.md): frozen shape filter + dormant ID được so sánh bốn ablation trên ba recording. Hai lượt mesh depth cải thiện paired ratio 77.74→93.71% và 75.62→90.09%, fragmentation proxy 13→10 và 10→8. Đây là diagnostic hình học, không phải semantic accuracy.

**Validation proxy cũ FAIL:** paired rows 997→603 dù tỷ lệ ghép tăng; prior đã loại trực tiếp 383 baseline-paired rows của người dạng cylinder. Giữ failure và baseline cũ, không triển khai filter hay bật mask train. Lượt mesh bổ sung chọn sau failure, không phải held-out test; navigation nguồn vẫn TIMEOUT. 13 test/audit integrity đạt, không thay kết luận semantic. Bước kế: recording depth mesh với seed/layout khác và nhãn visibility/semantic trước batch; HuNavSim chưa dùng. Không chạy Gazebo mới hoặc sửa ROS trong lượt này.

Mốc lưu: `feat: evaluate depth track refinements across sensor profiles`.

## Cập nhật 22/10/2026 — pilot track cảm biến và nhãn BEV tương lai

[Perception/target pilot](benchmark/perception_target_pilot/reports/PERCEPTION_TARGET_REPORT.md) chạy offline cùng episode: depth + odom tạo candidate tracks, GT chỉ ở nhánh label. 2577/3315 confirmed candidate rows ghép hình học được (77.74%, median 13.62 cm); đây không phải accuracy detector. Có phân mảnh ID và ứng viên chưa ghép, nên semantic/association/training masks vẫn false.

Đã dựng 2469 target BEV ở +0.5/+1/+3 s cho 823 cửa sổ: future depth thực quan sát, warp về khung robot tại anchor, unknown được mask khỏi loss. GT ego poses chỉ dùng cho LABEL; input BEV warp bằng odom nhân quả và vẫn có sai số ước lượng. 12 test và audit độc lập đạt; portable bundle gốc/raw/ROS không đổi. Bước kế: nghiệm thu person detector từ depth, occlusion/đứt ID và alignment trên episode độc lập trước batch/train. Giữ HuNavSim cho test cuối.

Mốc lưu: `feat: prototype sensor tracks and anchor-frame future depth BEV labels`.

## Cập nhật 22/10/2026 — đã xuất portable episode, chưa train-ready

[Portable episode](benchmark/portable_episode/reports/PORTABLE_EPISODE_REPORT.md): đã xuất **823 cửa sổ từ 1 episode**, history 2 s / nhãn tương lai 3 s, đủ **1013 ảnh depth khớp raw từng byte**. Loader NumPy đọc được ở thư mục khác mà không cần ROS/Gazebo; input và ground-truth supervision đọc riêng. Bundle đầy đủ khoảng 539 MB ở `benchmark/portable_episode/generated/episode01/`; Git giữ công cụ, báo cáo và một mẫu nhỏ.

Depth khả dụng già 100 ms so với anchor theo rule sampling bảo thủ; BEV ở thời điểm đo depth, chưa bù chuyển động tới anchor. 194/823 goal quá tuổi được mask. H1–H5, sensor association, future BEV và training eligibility vẫn false; không coi cửa sổ chồng lấn là episode độc lập. Giữ split pilot_only và HuNavSim cho test cuối. Bước kế: xác thực sensor track/association và chốt target BEV tương lai, rồi nghiệm thu độc lập nhỏ trước batch. Lượt này xử lý offline, không chạy lại Gazebo hoặc sửa ROS/Nav2.

Mốc lưu: `feat: export a portable episode with causal windows and masked supervision`.

## Cập nhật22/10/2026 — một episode tránh nhóm sớm đã tới đích thật

[Pilot Nav2 tránh sớm](benchmark/early_avoidance_pilot/reports/EARLY_AVOIDANCE_REPORT.md):profile runtime radius0.25m/inflation0.8m, speed0.30/0.20mps đạt1lượt standingv3 seed42, AMCL/odom gốc:87.31sim s trong budget90s, goal thật14.42cm/0.069rad, nominal gapmin59.25cm, không hull intrusion nhóm standing theo metric. Đủ tail3.35s;16topicMCAP,1013depth/507scan/10120IMU;1008cặp depth đối chứng.5frame phụ thiếu đều warmup, shared OFF khớp raw bitwise; GPU NVIDIA/cleanup đạt.

**Đính chính mốc động trước:**camera copier làm gz:expressed_in→ns0:expressed_in, khiến Gazebo bỏ frame ma sát4bánh; canonical XML audit đã bỏ sót. [Errata](benchmark/early_avoidance_pilot/reports/ERRATA.md). Fix riêng33427f4 khôi phục literal prefix trong runtime, SDKtest gốc4/bad0/fix4; src/robot và dataset pilot cũ không đổi. Ba lượt exploratory setup sai bị loại; slow fixed vẫn timeout và giữ FAIL. Không coi oracle/wz_std là fix hay tính success rate.

Chỉ là1pilot cấu hình riêng, chưa default/robust social nav/train-ready. Native nhóm có thể tan; H1–H5 vẫn masked, association/futureBEV còn thiếu. Bước kế: chuẩn hóa episode đạt và provenance thành sample portable, rồi xác nhận độc lập nhỏ trước batch. Mốc lưu: `feat: complete an early avoidance pilot with native robot physics`.

Các mục dưới giữ lịch sử theo thời điểm đo.

## Cập nhật22/10/2026 — đối chứng cảnh03 và pilot depth động

[Lifecycle và pilot động](benchmark/depth_proxy_lifecycle/reports/LIFECYCLE_MOTION_REPORT.md):cảnh03 đã đủbaseline/masked/null, camera ON/OFF/null và LiDAR chẩn đoán khớp bitwise,3exit0 trên NVIDIA với pause→stats dừng→đợi0.5s→stop. Control nguyên bản lượt này cũng đóng sạch; lỗi treo cũ chưa tái hiện/biết root cause, không tuyên bố sửa dứt điểm.

Một pilot standingv3+Nav2 thu153cặp depth thật (68active), robot đi2.662m, người1/3 nhích4.09/11.26cm;153OFF khớp MCAP bitwise, renderer/cleanup đạt. **Điều hướng chưa đạt:** human guard fatal15.2025s, người1 cần0.311m/s trong khi cap0.20; replay offline tái hiện rejection. Lượt này quarantine, không train hoặc tính success. Không sửa ROS/Nav2/native worker/cap/clearance/defaultcamera/pilot cũ. Bước kế: replay và thử planner giảm tốc/tránh nhóm sớm trong runtime riêng, rồi1episode hoàn chỉnh; lifecycle vẫn cần theo dõi. Mốc lưu: `feat: validate ordered shutdown and record depth motion failure`.

Các mục dưới giữ lịch sử theo thời điểm đo.

## Cập nhật22/10/2026 — ablation trụ trong camera depth (nghiệm thu giới hạn)

[Kiểm trụ depth và LiDAR](benchmark/depth_proxy_ablation/reports/DEPTH_PROXY_REPORT.md):2 cảnh tĩnh,6 fixture GPU NVIDIA sạch,160depth/30scan gốc/30scan chẩn đoán. Chỉ ẩn trụ ở camera OFF giúp thấy mesh người; camera ON và LiDAR chẩn đoán bằng bitwise với baseline,55 định nghĩa collision giữ nguyên. Audit raw/SHA và3test đạt. BEV có khe chân và khác đĩa bảo vệ LiDAR, không dùng để tự cho robot đi giữa nhóm.

**Chưa đạt đủ3 cảnh hoặc ổn định shutdown:** cảnh03 masked đo đủ nhưng2 lượt đóng phải SIGKILL; giữ lỗi run07/run08 và dừng retry. run06 lỗi GPU startup cũng giữ nguyên. Chỉ ghép2 cảnh hoàn chỉnh có lineage; không gọi toàn bộ run thành công. Không sửa ROS/Nav2/robot source hoặc tích hợp mặc định, không train. Bước kế: chẩn đoán lifecycle cảnh03, rồi1pilot động riêng trước recorder; còn association/persona và tương laiBEV. Mốc lưu: `feat: evaluate depth-only proxy filtering with static lidar controls`.

Các mục dưới giữ lịch sử theo thời điểm đo.

## Cập nhật22/10/2026 — kiểm chiều cao camera và hình BEV

[A/B camera12/40cm](benchmark/camera_height_ab/reports/CAMERA_HEIGHT_REPORT.md) đã kiểm3snapshot tĩnh,2camera cùng timestamp trên GPU NVIDIA:60ảnh raw ổn định bitwise,5test đạt,3server clean exit/cleanup rỗng.40cm tăng độ phủ BEV ở3cảnh; tại cảnh sát nhóm, knownarea8.1%→11.5% nhưng62.7%→77.8%pixeldepth hợp lệ vẫn khớp mặt trụ proxy. Nâng camera chưa loại được hình người dạng cột; chưa chọn40cm làm mặc định. Thử nghiệm camera riêng, không Nav2/native human steps và không đổi robot source/pilot cũ.

Giữ các lỗi setup/animation/renderer run01–03; run04 là cohort tĩnh đạt. Bước kế ưu tiên ablation hình học mà depth nhìn thấy và kiểm lại LiDAR/collision trước tích hợp hoặc thu train lớn. Các nhãn H1–H5 vẫn cần kiểm chứng, robot chen nhóm vẫn cần metric/cost planner; không chờ World Model tự sửa những vấn đề này. Mốc commit: `feat: compare camera heights with paired Gazebo depth observations`.

Các mục dưới giữ lịch sử theo thời điểm đo.


## Cập nhật22/10/2026 — đã thu dataset pilot đầu tiên

[Recorder và dataset pilot](benchmark/dataset_pilot/reports/PILOT_REPORT.md) đã thu1episode Gazebo headlessNVIDIA với6người/standingv3: goal thật14.72cm,404depth,202fullscans,4039IMU;16topicMCAP đọc lại,213cửa sổ lịch sử20bước/tương lai30bước. Audit độc lập xác nhận404ảnh depth bitwise bằng raw, không có quan sát tương lai và đủ nhãn quỹ đạo3s. Bản chính local ở `benchmark/dataset_pilot/episodes/pilot01/normalized`; raw/NPY lớn Gitignore, báo cáo/SHA/ảnh/GIFthật lưu Git. Chỉ runtime copy sửa metadata/TFcamera bị trùng; ROS/Nav2/human worker cũ giữ nguyên.

Chốt nguồn theo năng lực: SocNavGym + CrowdSimPlus + ComplexityNav cho train/validation tách episode/seed/scene; HuNavSim hiện có giữ testcuối, không tuning. Giữ H1–H5 nhưng persona hiện tại null/maskfalse, không gán nhãn theo simulator hoặc đứng=H4. Chưa có bộ train hoàn chỉnh: cần kiểm persona xuyên guard/projection, sensor-track association và định nghĩa BEV tương lai, rồi pilot recorder từng nguồn còn lại trước batch12 nhỏ. Chưa triển khai World Model hay Isaac Sim. Mốc commit: `feat: record and validate a portable social navigation dataset pilot`.

Các mục bên dưới giữ lịch sử theo thời điểm đo.


## Cập nhật22/10/2026 — pilot nhóm với Nav2

[Nhóm đi/đứng với Nav2 và camera thật](benchmark/socnavgym_group_nav2/reports/GROUP_NAV2_REPORT.md) đã được đo. Hai walking v2 quan sát được nhóm đi thật (seed42 tâm nhóm đi7.06m), nhưng actual goal lệch21.12/29.77cm nên chưa đạt ngưỡng20cm dù action SUCCEEDED. Standing v2 bị terminal stop sau15.06sim s do thành viên bị khóa; giữ nguyên đối chứng và không chạy seed43 v2.

Standing v3 riêng cho phép thành viên nhóm đứng gần robot nhích tránh≤0.2m/s, giữ floor hình học2mm và không đổi robot/Nav2. Replay interval lỗi cũ, offline/gate đạt; hai lượt headless NVIDIA tới đích thật18.87/15.85cm, max nhích0.150/0.126m/s. Bốn lượt hoàn tất không ghi overlap>5mm theo metric, nhưng clearance không tự chứng nhận social comfort. Nhóm ban đầu là fixture; native lifecycle tiếp tục, không gọi là formation tự phát.

Có hai lỗi GUI GPU/Qt được giữ; startup tuần tự chưa sửa dứt điểm. Standing cuối dùng camera GPU thật không mở GUI. Nav2 vẫn có thể đi giữa nhóm trò chuyện, chưa group-aware; goal thật walking chưa đạt. Đây là kết quả pilot có giới hạn, không nghiệm thu toàn bộ social navigation.

[Bộ hướng dẫn demo](benchmark/socnavgym_group_nav2/DEMO_GUIDE.md) chốt cách trình bày ba framework và chọn SocNavGym variant cho nhóm; CSP ORCA+ giữ baseline cá nhân, CN geometry hybrid nghiên cứu. Đã đủ bằng chứng cho giai đoạn khảo sát/chọn mô hình; không tự mở thêm framework, World Model/Isaac hoặc ma trận AMCL. Các việc còn lại về GUI ổn định, goal thật và không gian xã hội nhóm cần chốt phạm vi riêng. Mốc lưu: `feat: evaluate SocNavGym groups with Nav2 and bounded standing avoidance`. Code ROS/Nav2/vendor cũ và12 mục staged của người dùng được bảo toàn.

Các mục dưới giữ lịch sử theo thời điểm đo.

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
