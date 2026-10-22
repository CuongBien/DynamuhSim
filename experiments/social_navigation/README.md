# Nhà xưởng và vòng khảo sát SocNavGym

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

[Đã có bộ so sánh offline chung36 lượt](benchmark/common_response/README.md):6 cấu hình của3 framework,18 cặp far/near, một seed và6 người. Giữ các overlap native/soft-force; ưu tiên SocNavGym lifecycle variant cho mục tiêu nhóm, CrowdSimPlus ORCA+ làm baseline avoidance. Đây chưa phải benchmark chung Gazebo/Nav2. Pilot ComplexityNav đã commit `c9dffe2`.

[ComplexityNav đã chạy pilot Gazebo4 người](benchmark/complexitynav_gazebo/README.md):60s có robot tiến/ngang, actual proxy/clock/LiDAR đạt, đủ4 IDs,98.33% tia human đủ điều kiện khớp15cm; NVIDIA/GUI/shutdown được kiểm chứng. Có thêm lượt lỗi worker để xác nhận freeze. Đây là geometry variant, chưa benchmark Nav2 hoặc dense crowd. Bản offline đã commit `be816c3`.

[ComplexityNav: adapter tránh vật cản đạt offline](benchmark/complexitynav_geometry/README.md):6 cảnh ghép cặp native/variant,1040 update adapter; đi vòng kệ tới đích, hai chiều qua khe1.8m, đủ42 AABBs, reset80 bước. Chưa nối biến thể mới vào Gazebo. Native preflight trước đó đã commit `b0b3c33`; nhánh native vẫn giữ nguyên hạn chế xuyên kệ.

[ComplexityNav: kiểm định trước Gazebo](benchmark/complexitynav_contract/README.md): ID/clock/reset/native-step đạt; hai probe native ORCA/SFM đi xuyên kệ. Đủ42 AABBs được đối chiếu nhưng chưa được native human solver dùng, nên chưa nối Gazebo; bước kế tiếp là adapter hình học riêng. Ba phần SocNavGym đã lưu ở `2480158`, `779c338`, `b3f0022`.

[Formation/tách nhóm đã chạy trong Gazebo](benchmark/socnavgym_warehouse_lifecycle/README.md): bản sửa adapter riêng hỗ trợ6 người, đủ42 vật cản nhà xưởng và robot có vận tốc đo thực. Ba trace offline đạt; hai lượt Gazebo120s đạt kiểm tra kỹ thuật. Seed42 chỉ có formation, seed43 có nhóm đứng/đi rồi tan; tâm nhóm đi được2.17m. Camera thật, NVIDIA và GUI đã kiểm chứng. Đây là pilot tại một lối đi, chưa chạy Nav2 hoặc benchmark chọn framework.

[Bản sửa formation độc lập](benchmark/socnavgym_lifecycle_fix/README.md) là cơ sở của adapter trên. [Native reference](benchmark/socnavgym_lifecycle/README.md) vẫn giữ nguyên cả nhảy vị trí và overlap đã đo; không gọi bản sửa là hành vi upstream nguyên bản.

[Probe phản ứng người](benchmark/human_response_replay/README.md):6 CrowdSimPlus +18 SocNavGym offline, cùng input robot định trước; hình replay được gắn nhãn riêng. [Gazebo nhóm — hoàn tất lượt bổ sung](benchmark/group_gazebo_demo/README.md): sửa QoS subscriber AMCL trong harness, ba lượt nhóm đi bộ đã chạy; GUI kéo/zoom và Pause/Play hoạt động. Tổng sáu lượt hợp lệ, năm lượt đạt sai số đích thật ≤0.20m; giữ lại một lỗi khởi tạo cũ.

[Kiểm tra nhiều seed](benchmark/localization_validation/README.md):12 lượt so AMCL gốc với LF-prob+beam-skip, seed42/43/44; dữ liệu giữ riêng với pilot cũ.

Đợt thử mới: [workaround shutdown Gazebo trên WSL D3D12](benchmark/shutdown_fix/README.md) và [pilot AMCL gồm 3 cấu hình × 2 human policies](benchmark/localization_trials/README.md). Cả hai nằm trong harness thử nghiệm, chưa thay cấu hình robot gốc.

Tất cả file mới nằm trong `experiments/social_navigation/`. Không sửa launch/model, cấu hình Nav2 gốc, plugin C++ hay môi trường Python ROS
đang có. Dùng workspace
`install/` hiện có; không rebuild các thay đổi chưa commit của người dùng.

## Chạy nhà xưởng trên Ubuntu/WSL

Từ thư mục gốc DynamuhSim:

```bash
bash experiments/social_navigation/run.sh warehouse
```

Lệnh mở Gazebo + RViz + Nav2 MPPI Omni. Robot bắt đầu tại `(-27, 0, 0)`.
Trong RViz dùng **2D Goal Pose**, ví dụ `(26, 0)` để đi qua lối hẹp.
Mặc định ROS domain 73 và Gazebo partition `dynamuhsim_social_warehouse`.
Không mở hai lần cùng domain/partition. Nhấn Ctrl+C trong terminal để dừng.

Chỉ xem world và robot, chưa chạy Nav2:

```bash
bash experiments/social_navigation/run.sh warehouse nav:=false
```

Headless và kiểm tra tự động có timeout, dừng đúng process group được tạo:

```bash
bash experiments/social_navigation/run.sh verify --world warehouse
bash experiments/social_navigation/run.sh verify --world warehouse --nav --goal-x 26 --timeout 240
bash experiments/social_navigation/run.sh verify --world warehouse --nav --goal-x 26 --timeout 240 --gui
```

Không cần colcon build: wrapper nhận world/map bằng đường dẫn tuyệt đối và dùng
launch cũ qua IncludeLaunchDescription. `obstacle:=human` trong wrapper chỉ giữ
nguyên SDF, tránh tạo file `/tmp/corridor_*` dùng chung của launch cũ; world này
chưa có actor/người hoặc plugin RVO2 nên không có người xuất hiện tự động.

Nhà xưởng 60×40m có 8 dãy kệ, sân rộng, cột, khu loading, vùng nhóm trò chuyện,
lối đi chính và cửa hẹp 1.8m dài 8m. Cột hỗ trợ LiDAR 3.5m trên tuyến chính;
chưa khẳng định định vị tốt ở mọi vị trí trong sân rộng. Không có mái để dễ xem
và giảm tải. Bản đồ 0.05m/pixel sinh từ cùng hình học collision của SDF, không
phải bản đồ thu bằng SLAM. Khu màu xanh chỉ đánh dấu sàn, chưa có hành vi người.

![Bố cục nhà xưởng](warehouse/layout.png)

Sinh lại world/map/config riêng sau khi chỉnh generator:

```bash
python3 experiments/social_navigation/generate_warehouse.py
```

Generator cần Pillow và PyYAML (máy này đã có). Việc sinh lại cũng cập nhật
`nav2_warehouse.yaml` từ cấu hình Mecanum hiện có; hãy lưu chỉnh sửa riêng trước.

## SocNavGym độc lập

Venv hiện đã cài xong trong `.venv-socnavgym/`. Để tái tạo trên máy này:

```bash
bash experiments/social_navigation/setup_socnavgym.sh
```

Cần Python 3.12, compiler C++, CMake, curl và Git. Script dùng venv
`--without-pip` + bootstrap pip chính thức nếu thiếu `ensurepip`, không cần cài
pip vào Python hệ thống. Các dependency và source được ghim phiên bản; toàn bộ
freeze thực tế nằm trong `reports/environment_freeze.txt`. Không dùng `sudo pip`.
PyTorch dùng bản CPU, không cài CUDA/DGL cho reward `sparse_gotopoint` đang thử.

Chạy demo theo API/source v2 hiện tại với **cấu hình tests/test_env.yaml gốc**:

```bash
bash experiments/social_navigation/run.sh socnavgym --scenario original --steps 500 --show
```

Runner ngoài source upstream gọi reset/step/render, dùng action ngẫu nhiên như
ví dụ README; không dùng joystick. Không chạy nguyên script tests/test_env.py
vì script đó yêu cầu joystick. README v1 và nhiều config cũ không đồng bộ với
source main v2; không sửa source để che lỗi này.

Các lượt quan sát tùy chỉnh, chạy lần lượt:

```bash
bash experiments/social_navigation/run.sh socnavgym --scenario groups --steps 600 --show
bash experiments/social_navigation/run.sh socnavgym --scenario formation --steps 1200 --show
bash experiments/social_navigation/run.sh socnavgym --scenario reactive --steps 600 --show
bash experiments/social_navigation/run.sh socnavgym --scenario blind --steps 600 --show
```

Bỏ `--show` để render video/ảnh không mở cửa sổ. Mặc định seed 42; có thể đổi
`--seed`. Mỗi lượt ghi config, video MP4, 4 ảnh, quỹ đạo JSON và summary trong
`reports/`. Cùng scenario/seed sẽ ghi đè kết quả trước. Nhóm groups/formation:
robot đứng yên để quan sát. Reactive/blind: robot cố tiếp cận người gần nhất;
đây là stress test, không phải một policy điều hướng an toàn hay benchmark Nav2.

Các cấu hình demo là 2D 14×14m, **chưa phải nhà xưởng Gazebo**. Không suy diễn
hiệu năng hay tỷ lệ va chạm của robot ROS từ các lượt này.

## Bằng chứng và trạng thái

Xem [báo cáo vòng đầu](reports/ROUND1.md), video và JSON liên kết trong báo cáo.
SocNavGym đã có [adapter Gazebo thử nghiệm](gazebo_adapter/README.md) cho một nhóm 3 người đi cùng nhau hoặc đứng trò chuyện trong nhà xưởng. Clock, robot feedback, pose proxy và LiDAR đã được kiểm tra. Đã có [pilot Nav2 RVO2 ↔ SocNavGym](benchmark/reports/BENCHMARK_REPORT.md) gồm 9 lượt cùng scene/map/params. Formation/tách nhóm hiện có pilot riêng6 người nêu đầu trang; không gộp kết quả vào benchmark Nav2 cũ.

CrowdSimPlus đã có vòng thử độc lập tại [round2](round2/README.md); ComplexityNav đã có vòng thử độc lập tại [round3](round3/README.md). CrowdSimPlus hiện đã có [adapter Gazebo + Nav2](crowdsimplus_adapter/README.md), với [7 lượt thử và GIF](benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md). ComplexityNav chưa nối vào Gazebo. Xem [bảng so sánh](FRAMEWORK_COMPARISON.md) và [báo cáo adapter](gazebo_adapter/reports/ADAPTER_REPORT.md) trước khi chọn hành vi nhóm.

## Tiến độ CrowdSimPlus SFM

Đã kiểm thử native SFM qua container trong Gazebo, cùng robot Mecanum/Nav2:
[report SFM/ORCA+](benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md).
Bốn lượt đến đích, không overlap >1 cm trong mẫu đo; SFM có hạn chế dừng quanh
waypoint chung và không hoàn tất patrol. Chưa có hành vi nhóm/hội thoại.

## Tiến độ fixture đích riêng

Đã kiểm tra SFM/ORCA+ với endpoint riêng: 4/4 goal Gazebo/Nav2 đạt, mỗi người
hoàn tất 2–4 chặng; không ghi nhận overlap >1 cm trong sample. [Report và GIF](benchmark/individual_goals/reports/run01/INDIVIDUAL_GOALS_REPORT.md).
Giữ native policy và robot/Nav2; đây vẫn là ba người độc lập, chưa có hội thoại.

## Tiến độ cảnh giao cắt

Đã chạy native SFM/ORCA+ với ba người đi cắt ngang: 4/4 goal Gazebo/Nav2 đạt,
mỗi người có 4–9 giao cắt được xác nhận bằng pose Gazebo, không ghi nhận
overlap >1 cm trong sample. [Report, GIF và lệnh demo](benchmark/crossing/reports/run01/CROSSING_REPORT.md).
Robot có vòng tránh; chưa phải kiểm chứng nhường đường trong lối hẹp.

## Tiến độ lối hẹp

Đã chạy một empty control và bốn lượt có người trong khe 1,8×8 m. Cả SFM và
ORCA+ qua được ở seed43, bị Nav2 hủy goal ở seed42; ORCA+ seed42 có AMCL drift
tới 2,23 m. [Report, GIF, chẩn đoán và lệnh demo](benchmark/bottleneck/reports/run01/BOTTLENECK_REPORT.md).
Chưa dùng các lượt này để xếp hạng simulator; cần tách lỗi định vị trước.

## Các phần đã lưu Git

[Xem sáu commit tính năng và bằng chứng tương ứng](COMMIT_HISTORY.md). Các
trạng thái “chưa commit” trong report phản ánh thời điểm chạy thí nghiệm.

## Chẩn đoán bottleneck offline

Đã đối chiếu năm lượt đã lưu: ORCA+ seed42 có AMCL jump 1,992 m trong khi
odom khớp Gazebo trong 1 cm. Sensor score dựng lại đã khớp thư viện AMCL
native; failure SFM có chuỗi khác, cần thêm global plan/particle cloud.
[Report và bằng chứng](benchmark/localization_diagnosis/reports/run01/LOCALIZATION_DIAGNOSIS.md).
Chưa chỉnh ROS/Nav2 hoặc thay kết quả benchmark cũ.

Chẩn đoán bổ sung: [global plan/costmap và particle cloud trong Gazebo](benchmark/navigation_diagnostics/reports/run03/NAVIGATION_DIAGNOSTICS.md). Hai lượt mới tới đích nhưng chưa tái hiện/sửa lỗi cũ; benchmark gốc giữ nguyên.

Lặp sáu lượt sau cleanup: [đã bắt trực tiếp mode AMCL trong khối máy và global reroute](benchmark/stability/reports/run01/STABILITY_REPORT.md). SFM2/3, ORCA+1/3; dữ liệu mới giữ riêng, vẫn chưa có bản sửa Nav2.
