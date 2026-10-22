# Kế hoạch tiếp theo: đánh giá hành vi người, tách khỏi nhiễu định vị

## Cập nhật 23/10/2026 — depth planar-surface veto có nghiệm thu giới hạn

[Surface guard](benchmark/depth_surface_guard/reports/SURFACE_GUARD_REPORT.md): giữLiDAR+depth theo người dùng, khôngRGB. Quy tắc CUDA chốt trước regression và fixture mới: rút8false-support rows trên hộp, không gắn planar cho người, giữ75proposal/40person-owned rows và true support1→1. Cũfull/null sai4→0/3→0; cảnh mới hộp xoay+trụ cong sai2→1, còn nhận nhầm trụ tại5s và thiếu recall.

7test/75CPU–GPU comparisons/18270point residual audit đạt. Cảnh mới75depth+20scan GPU NVIDIA đóng sạch; source/config không đổi saufreeze. Chỉ surface-only gatePASS; full semantic vẫnFAIL, không defaultROS/Nav2/WorldModeltrain/HuNavSim, H1–H5 masked. Bước kế: bộ mẫu perception depth có nhãn nhỏ cho người/vật cong/chuyển động/occlusion; chưa batch trainWorldModel. Mốc lưu: `feat: add depth surface veto with independent clutter checks`.

## Cập nhật 23/10/2026 — pretrained LiDAR semantic pilot chưa đạt

[DR-SPAAM report](benchmark/lidar_person_pilot/reports/LIDAR_PERSON_REPORT.md): API gốc/checkpoint JRDB ann chạy PyTorch2.7.1 CUDA12.8 trên RTX4060 trong venv riêng, parity88tensors/60312cutout/3synthetic sequences đạt. Hai capture GPU cùng fixture full/null đóng sạch:150depth+40scan. Full4frame khả dụng có5supported rows=1người+4đồ vật; null3false rows; cả qualitygate FAIL. Không hạ confidence0.5 hoặc bỏ các cụm chưa rõ.

Math/model/config chốt trước capture; chỉ sửa JSON NumPy-bool writer sau capture/trước quality evaluation, nguồn cũ/diff/failure được giữ. Audit14360rays/6real-prefix/10causal selections và9test đạt; integrity không thay semanticFAIL. Không ROS/Nav2/default/WorldModel/train/HuNavSim. Người dùng chốt giữLiDAR+depth, không RGB. Bước tiếp: conservative depth surface/object evidence với abstention, regression cũ rồi fixture mới. Mốc lưu: `feat: evaluate pretrained LiDAR person detection on GPU`.

## Cập nhật 23/10/2026 — pixel visibility và clutter control

[Visibility report](benchmark/depth_visibility_pilot/reports/VISIBILITY_REPORT.md): một fixture tĩnh native mesh,15camera×5timestamp=75ảnh GPU NVIDIA. Nhãn10649pixel người không mâu thuẫn, ownership ổn định; audit57070pixel/9test đạt. Bốn người nhìn thấy đều có proposal, nhưng shape filter vẫn giữ2cụm đồ vật cả khi ẩn hết người: chưa thể gọi track là người hoặc bật association/train. Người phía sau chỉ còn11.92%silhouette nhìn thấy; chưa có fully-occluded case thật hoặc dynamic semantic acceptance.

Giữ lỗi wiring01–03 và strict inter-frame bitwise FAIL04. Protocol2 chốt trước05 dùng tolerance số học1mm đã có cho label, vẫn yêu cầu full/control cùng timestamp khớp từng byte;05 đạt numeric gate, strict bitwise vẫnfalse. Không thay inference/ROS/Nav2/dataset cũ; H1–H5/trainfalse, HuNavSim chưa dùng, navigationTIMEOUT giữ riêng. Bước tiếp: semantic person-vs-object sensor-only, kiểm lại clutter rồi động/occlusion và scene mới trước batch. Mốc lưu: `feat: add static depth visibility and clutter validation`.

## Cập nhật 23/10/2026 — sửa gap association, kiểm seed44 mới

[Identity guard](benchmark/depth_identity_guard/reports/IDENTITY_GUARD_REPORT.md): candidate05 chặn case GT5→GT6 ở seed43 (pairing changes1→0), giữ đủ1983 paired rows; mesh42 giữ fragmentation10, repeat42 giảm8→7. Các bản01–04 chưa đạt được giữ code/hash/kết quả. 14 behavior tests và audit đạt; code/gate chốt trước seed44.

Seed44 mới trên NVIDIA:1023depth bitwise,801cửa sổ/1episode. Parent/guard cùng2587paired,0pairing changes,fragmentation7: kiểm không suy giảm, chưa chứng minh cải thiện trên seed này. **Navigation TIMEOUT90.04s, goal còn4.012m** giữ nguyên. Identity-safety gate đạt; gate đầy đủ cũ vẫn FAIL trên43/44 vì fragmentation không giảm. Semantic/association/H1–H5/train vẫnfalse; không thay mặc định ROS. Bước kế: visibility/person-instance và semantic association trước batch; HuNavSim chưa dùng.

Mốc lưu: `fix: guard depth track identities across observation gaps`.

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

**Trạng thái mới nhất22/10/2026:** [bản sửa formation đã nối Gazebo và kiểm thử](benchmark/socnavgym_warehouse_lifecycle/reports/WAREHOUSE_LIFECYCLE_REPORT.md). Các mục bên dưới giữ tiến trình và quyết định ở thời điểm trước; kế hoạch hiện hành ở cuối trang.

Cơ sở: cohort12 lượt seed42/43/44 cho baseline và LF-prob+beam-skip đều5/6 goal thật. SFM thay đổi2/3→3/3, ORCA+3/3→2/3; đây là mẫu nhỏ, không chứng minh cấu hình mới tốt hơn. Giữ AMCL mặc định và dữ liệu hai failure, chưa chọn framework. [Bằng chứng](benchmark/localization_validation/reports/run01/MULTISEED_REPORT.md).

Cập nhật: bước1 đã chạy đủ6 probe CrowdSimPlus; bước2 đã chạy18 probe nhóm SocNavGym. [Báo cáo replay](benchmark/human_response_replay/reports/run01/REPLAY_REPORT.md). Bước3 đã có đủ sáu tổ hợp scene/variant hợp lệ sau đợt bổ sung ba lượt đi bộ: sáu action báo SUCCEEDED, năm lượt đạt sai số thật ≤0.20m. Một lỗi khởi tạo cũ được giữ riêng; QoS nhận AMCL đã sửa bằng tùy chọn trong harness. GUI kéo/zoom và Pause/Play đã được người dùng xác nhận. [Báo cáo Gazebo](benchmark/group_gazebo_demo/reports/recovery01/RECOVERY_REPORT.md). Bước4 chưa bắt đầu tích hợp. Không triển khai World Model, chuyển Isaac Sim, sửa ROS robot source hoặc thêm model học mới.

## 1. Bước triển khai kế tiếp: sáu probe replay cho CrowdSimPlus

Mục tiêu là đo phản ứng người khi nhận cùng đầu vào robot, tránh dùng lỗi AMCL/Nav2 để đánh giá human policy.

- Hai native policies: ORCA+ và SFM; cùng fixture, initial state, map geometry, radius, timestep0.1s và seed42.
- Ba input robot định trước: ở xa làm đối chứng; tiến gần đối đầu; đi ngang cắt hướng. Mỗi trace tối đa20sim s, dùng chung byte-for-byte cho hai policies. Ghi pose/vận tốc ground truth, không đưa AMCL vào human input.
- Chạy offline tuần tự trong runtime CrowdSimPlus có sẵn trước; không chạy Nav2/Gazebo trong giai đoạn replay. Giữ act/step và tham số native; không thêm clipping, tách người hoặc luật nhường để làm đẹp kết quả.
- Trước khi đo, kiểm tra trace không bắt đầu với overlap, đúng frame/clock và đúng geometry. Lưu trace, config/source hashes và mọi outcome.
- Báo cáo thời điểm đổi vận tốc/hướng khi robot tiến gần, khoảng hở signed, overlap và tiến độ đến waypoint. Quỹ đạo phản ứng khác nhau là dữ liệu; không yêu cầu mọi model phải tránh thành công mới coi probe là hợp lệ.

Nghiệm thu harness: đủ6 probe, đầu vào robot giống nhau trong mỗi cặp, parity với native step được kiểm tra, kết quả có thể tái lập trong runtime đã ghim. Không dùng6 probe để xếp hạng thống kê hoặc chứng nhận an toàn. Replay là thử nghiệm riêng, không trình bày hình vẽ replay như camera Gazebo.

## 2. Quay lại SocNavGym: nhóm và trò chuyện

Sau khi contract replay được kiểm định, dùng cùng cách ghi input/output để khảo sát một nhóm3 người ở hai scene đã hỗ trợ: đi nhóm và đứng trò chuyện.

- Tách rõ native SocNavGym và adapter robot-aware đã chỉnh; giữ native làm đối chứng.
- Quan sát giữ nhóm/membership, khoảng cách giữa thành viên, hướng quay và phản ứng robot trước contact; ghi cả overlap.
- Với CrowdSimPlus, cảnh đứng yên chỉ là stationary fixture, không gọi là conversation hoặc group model.
- Formation/dispersal tiếp tục ở demo độc lập; chỉ đưa vào Gazebo sau khi kiểm định lifecycle/ID/geometry riêng. Không giả định adapter nhóm hiện tại đã hỗ trợ phần này.

Nghiệm thu: phân biệt được năng lực native, biến thể adapter và fixture bên ngoài; có trace và hạn chế của từng scene, chưa chọn model chính thức chỉ từ hình ảnh.

## 3. Trình diễn và benchmark lại trong Gazebo

Sau replay, chạy từng backend tuần tự trong cùng nhà xưởng. Giữ robot Mecanum, LiDAR/IMU, controller và baseline AMCL; dùng mode shutdown WSL D3D12 đã kiểm chứng.

Có thể dùng lệnh robot được lập trước trong harness riêng để minh họa phản ứng; phải đo pose/vận tốc Gazebo thực tế. Cùng lệnh không bảo đảm cùng quỹ đạo do tương tác vật lý, nên không gọi đây là replay robot input chính xác. Nếu input thực tế khác nhau, chỉ kết luận trên phép thử kín vòng và công bố độ khác biệt đó.

Ghi hai nhóm kết quả riêng: chất lượng hành vi người và chất lượng điều hướng robot. Nav2 SUCCEEDED phải được đối chiếu với goal error thật; không gộp success với không va chạm. Ảnh/GIF lấy từ camera Gazebo, ghi policy/seed/sim time và tốc độ phát; biểu đồ từ log được gắn nhãn riêng. Kiểm tra renderer NVIDIA, cleanup và khả năng thao tác GUI trong phiên demo.

## 4. ComplexityNav sau cùng

Giữ demo độc lập hiện có làm đối chứng. Chỉ nối Gazebo sau khi adapter contract pose/velocity/ID/reset/clock và phép ánh xạ geometry nhà xưởng đã đạt kiểm định với các backend trước. Bắt đầu population nhỏ, đối chiếu native step và LiDAR, rồi mới tăng mật độ/lối hẹp. Group clustering không đồng nghĩa chủ động giữ nhóm hay hội thoại.

## Điều kiện chọn mô hình

Ưu tiên khả năng đúng mục tiêu (nhóm, đứng tương tác, tránh người/robot), hành vi quan sát được và hạn chế có số đo. Sau đó mới xét chi phí runtime và độ thuận tiện tích hợp. Mỗi đợt có protocol cố định, không thay failure bằng lượt đẹp; chưa mở rộng thêm các ma trận AMCL nếu chưa có câu hỏi chẩn đoán cụ thể.


## Công việc kế tiếp sau đợt này

Đã lưu ba commit: replay `b3f59e2`, observer AMCL `195f4e8`, Gazebo nhóm/GUI `e749fb5`. [Lifecycle SocNavGym](benchmark/socnavgym_lifecycle/README.md) đã kiểm tra ba seed với4100 update native: ID và reset prefix500 bước đạt, nhưng formation có nhảy vị trí tới32.6cm và overlap proxy hình tròn56–70cm. Quyết định: giữ formation độc lập, không promote vào Gazebo trong milestone này; không sửa policy để che hạn chế. Adapter nhóm cố định vẫn giữ nguyên.

Bước kế tiếp chuyển sang contract/geometry ComplexityNav trước khi nối Gazebo: giữ runtime Python3.8 riêng, native human update và source gốc; kiểm tra robot feedback, ID/reset/clock và toàn bộ collision AABB nhà xưởng bằng fixture nhỏ, rồi native parity và LiDAR. Chưa chạy thêm ma trận AMCL hoặc thay kiến trúc robot. Khi backend đủ điều kiện, chạy bộ cảnh chung và chốt bảng lựa chọn; formation không đạt sẽ được ghi là hạn chế của phương án, không giữ toàn dự án chờ vô hạn.


## Cập nhật sau yêu cầu sửa formation

Đã triển khai [continuous lifecycle adapter riêng](benchmark/socnavgym_lifecycle_fix/README.md), không sửa vendor hoặc robot.5476 update kiểm chứng: ba seed gốc, reset lúc có nhóm và fixture nhóm đi. Giữ vị trí khi chốt formation, giới hạn quay1.57rad/s, vận tốc0.8m/s, global individual-disc ORCA và swept fallback. Các phép đo độc lập không ghi nhận xuyên proxy; formation/dispersal và chuyển động vẫn hoạt động. Native reference được giữ nguyên.

**Ưu tiên tiếp theo vẫn là SocNavGym:** mở rộng kiểm định obstacle nhà xưởng và robot có vận tốc đo thực, rồi nối formation variant vào Gazebo và kiểm tra actor/proxy/LiDAR/reset/clock. Không chuyển ComplexityNav trước khi chốt phần này. Bản sửa hiện mới nhận robot đứng yên trong phòng vuông native, chưa nghiệm thu Gazebo hay hứa hành vi tự nhiên ở mọi cảnh. Các thay đổi lifecycle mới chưa commit.


## Sau pilot formation trong nhà xưởng —22/10/2026

Đã hoàn tất kiểm định obstacle/robot chuyển động và nối adapter riêng vào Gazebo:3 trace offline ×1200 bước,2 lượt live ×120s,6 người,42 AABBs. Seed42 đạt kỹ thuật nhưng chưa thấy tách nhóm; giữ PARTIAL. Seed43 có hai nhóm đứng hình thành/tan và một nhóm đi2.17m rồi tan. Proxy/clock/LiDAR/GPU/shutdown đạt; người dùng xác nhận thấy6 người và kéo/zoom. Chỉ restart session mới được kiểm tra; reset ngay trong world bị từ chối. [Report, hạn chế và camera thật](benchmark/socnavgym_warehouse_lifecycle/reports/WAREHOUSE_LIFECYCLE_REPORT.md).

Phần sửa standalone và tích hợp mới vẫn chưa commit. Trước khi mở milestone khác, lưu riêng các phần đã hoàn tất theo yêu cầu commit của người dùng; bảo toàn staging/code ngoài experiment.

Bước kỹ thuật tiếp theo là kiểm định contract ComplexityNav trong runtime độc lập hiện có: ID/frame/clock/reset, robot feedback và42 collision boxes; so native step trước khi nối population nhỏ vào Gazebo. Chỉ sau khi đạt mới chạy chung các scene để so hành vi, tăng mật độ/lối hẹp và chốt bảng chọn mô hình. Không mở thêm ma trận AMCL. Pilot formation hiện tại không thay thế benchmark Nav2 và không chứng minh tính tự nhiên ở mọi cảnh.


## Sau kiểm định ComplexityNav —22/10/2026

Ba phần SocNavGym đã commit riêng: `2480158` native reference, `779c338` sửa continuous formation, `b3f0022` pilot Gazebo6 người. Phần uncommitted hiện tại là [contract ComplexityNav và báo cáo](benchmark/complexitynav_contract/README.md). ID/frame/clock/pause/reset/native parity đạt ở runtime ghim, nhưng centralized ORCA/SFM đi xuyên kệ thật trong fixture độc lập; gate NOT_READY_FOR_GAZEBO được giữ.

Bước tiếp theo: adapter hình học riêng cho42 AABBs, giữ native đối chứng; kiểm định obstacle/swept disc/robot vận tốc/progress/reset trước khi nối population nhỏ vào Gazebo. Không gọi ledger42 boxes là native avoidance đã hỗ trợ. Sau khi đạt mới đối chiếu proxy/LiDAR rồi chạy các scene chung để chốt framework. Không đổi AMCL mặc định, robot/Nav2 hoặc kiến trúc project.


## Sau adapter geometry ComplexityNav —22/10/2026

Preflight native đã commit `b0b3c33`. [Adapter hình học riêng](benchmark/complexitynav_geometry/reports/GEOMETRY_REPORT.md) đã vượt gate offline:6/6 cảnh,1040 update adapter/960 native reference,42 boxes, mixed6 người/robot chuyển động, đi vòng kệ tới đích và hai người qua khe1.8m; reset80 bước khớp. Native xuyên kệ giữ làm đối chứng. Waypoint routing + ORCA safety projection là biến thể chỉnh hành vi, gồm cả nhánh SFM; chưa được gọi là native hoặc benchmark Gazebo.

Bước kế tiếp: commit riêng phần đã đạt, sau đó pilot Gazebo population nhỏ, timestep native0.25s với endpoint/interpolation đúng clock. Worker lỗi phải freeze/fail-until-restart; đối chiếu proxy/LiDAR/actual robot feedback, pause/owned cleanup/GPU trước khi tăng mật độ hoặc chạy bảng so sánh chung. Không thay robot/Nav2, AMCL mặc định hoặc kiến trúc project.


## Sau pilot ComplexityNav Gazebo —22/10/2026

Adapter offline đã commit `be816c3`; [pilot mới](benchmark/complexitynav_gazebo/reports/GAZEBO_PILOT_REPORT.md) chạy4 người trong Gazebo60s, measured robot tiến/ngang,23 arrival, actual proxy/LiDAR/pause/GPU/GUI/cleanup đạt. Fault injection xác nhận freeze rồi restart session mới chạy tốt. Pilot này chưa commit. ComplexityNav geometry variant hiện có Gazebo integration; trained RGL/native geometry chưa được promote.

Bước tiếp theo: commit pilot riêng, chốt protocol bộ cảnh chung cho3 backend (same warehouse/robot inputs, population/attributes/metrics khai báo rõ, native vs adapter variant tách riêng), rồi chạy tuần tự và chốt bảng lựa chọn. Bao gồm phản ứng robot tiến gần, vòng kệ/lối hẹp và nhóm/conversation đúng năng lực backend; không gọi clustering hay người đứng fixture là social formation. Không dùng local-aisle pilot này để kết luận dense crowd hoặc Nav2 đã đạt, không thay AMCL/controller gốc.


## Sau cohort chung offline —22/10/2026

Pilot ComplexityNav đã commit `c9dffe2`. [36 phép thử common response](benchmark/common_response/reports/run01/COMMON_RESPONSE_REPORT.md) đã hoàn tất,6 repeat-prefix đạt; đầu vào physical discs/targets/robot trace chung, native dt và prediction assumptions khác nhau được công bố. Guarded Soc, CSP ORCA+ và CN geometry không ghi overlap>1mm ở6 case/control; native Soc/CN và SFM giữ các hạn chế đã đo. Formation tắt trong cohort avoidance, không áp số6/6 cho nhóm.

Theo mục tiêu người đi nhóm/đứng tương tác/formation, ưu tiên SocNavGym lifecycle variant đã có Gazebo; CrowdSimPlus ORCA+ giữ baseline tránh người. CN geometry là biến thể phụ, không gọi clustering là social formation hoặc ngầm dùng RGL chưa có checkpoint. Chưa chuyển backend mặc định của robot.

Bước còn lại: commit cohort này, đưa đúng normalized fixture/config vào cohort Gazebo riêng, đoactual pose/LiDAR/robot path; chỉ sau đó so Nav2 cùngscene nếu cần. Không trộn pilot4 người radius0.3m với cohort6 người radius0.36m thành kết quả chung. Không thêm framework, World Model, Isaac Sim hoặc mở rộng ma trận AMCL.
