# So sánh khả năng đã đo — cập nhật22/10/2026

## Cập nhật23/10/2026 — hoàn thành train thử classifier người/vật

[Classifier report](benchmark/depth_person_classifier/reports/CLASSIFIER_REPORT.md):4cấu hình train CUDA RTX4060 trên54train/30validation; chọn linear depth31tham số,threshold0.5 trước khi mở48test. Validation11/12người,0/18đồ vật nhận nhầm; test23/24người,0/24đồ vật nhận nhầm, còn1FN dưới che khuất. Gate pilot đặt trước fit đạt; không suy ra end-to-end detector recall hoặc tổng quát ngoài một avatar/nhà xưởng.

10tests và audit68choice/132portable replay đạt; model/features/normalization/testseal/trọng số/reports/GIF validation thật lưu Git. Corpus/ROS/Nav2/staged user giữ nguyên, HuNavSim chưa dùng, H1–H5/association/WMtraining vẫnfalse. Mốc classifier đã đóng; bước kế quay lại thu episode robot chuyển động phục vụ WM/RL với nhãn chưa đủ tin cậy được mask, không chờ detector hoàn hảo. Mốc lưu: `feat: train and validate a depth person classifier pilot`.

## Cập nhật23/10/2026 — bộ pilot perception có nhãn đã xuất

[Corpus report](benchmark/depth_perception_corpus/reports/CORPUS_REPORT.md):7cảnh GPU NVIDIA,525depth đối chứng/140scan native; xuất19frame/132proposal (64người,68môi trường), chia toàn họ bố cục train/validation/test trước thu. Static chỉ giữ midpoint; đầu vào depth+LiDAR tách LABEL, khôngRGB/H1–H5. 3frame đầu/21proposal thiếu scan nhân quả được mask, không dùng scan tương lai.

9test và auditor02/portable QA đạt (84sample đổi nhãn không đổi input,13frame bỏ tương lai không đổi); auditor01 có giả định dư về scan và được giữ FAIL/source/diff. Không model test/tuning/preview test. [Mẫu train](benchmark/depth_perception_corpus/examples/train_p003/manifest.json) và [GIF replay thật](benchmark/depth_perception_corpus/reports/media/depth_labels.gif) lưu Git; full bundle/raw ở generated cần backup riêng. Chỉ nghiệm thu data pilot, một mesh/nhà xưởng và scripted motion; chưa detector/association/train WorldModel, không sửa ROS/Nav2/HuNavSim. Bước kế: baseline person-vs-environment trên train/validation rồi đóng băng trước test. Mốc lưu: `feat: add a labelled depth perception pilot corpus`.

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

[Benchmark pilot offline chung36 lượt](benchmark/common_response/reports/run01/COMMON_RESPONSE_REPORT.md) đã hoàn tất với human radius/initial geometry/robot trace chung. Đây là18 cặp far/near, một seed, native dt/speed/observation khác nhau; không phải bảng xếp hạng thuật toán thuần hoặc benchmark chung Gazebo/Nav2.

| Năng lực / trạng thái hiện tại | SocNavGym | CrowdSimPlus | ComplexityNav |
|---|---|---|---|
| Nhóm đi có membership, đứng tương tác | Có scene/model riêng; đã quan sát native và Gazebo | Chưa có model giữ nhóm/hội thoại trong bản thử | Clustering/static agents, chưa phải social formation |
| Formation/tách nhóm | Native có snap/overlap; adapter lifecycle riêng đã đạt pilot Gazebo6 người | Chưa có model tương đương | Chưa có model tương đương |
| Gazebo đã chạy thật | Nhóm cố định và lifecycle variant, có robot/LiDAR | ORCA+/SFM, có Mecanum/Nav2/crossing/bottleneck | Geometry hybrid4 người/60s; proxy/LiDAR/fault-freeze/NVIDIA/GUI đạt |
| Benchmark chung offline6 người | Native5/6,guarded6/6 không overlap>1mm; formation tắt ởcohort này | ORCA+6/6,SFM3/6 không overlap>1mm | Native4/6 (không dùng nội vật cản),geometry6/6 không overlap>1mm |
| Vai trò đề xuất cho mục tiêu hiện tại | **Ứng viên chính cho nhóm/đứng tương tác/formation, dùng variant gắn nhãn rõ** | **ORCA+ làm baseline tránh người; SFM làm đối chứng soft-force** | Phương án phụ cho nghiên cứu; chưa trained RGL, chưa Nav2 |

Các số /6 gồm far control vànear condition, không phải nhiều seed hoặc success probability. Native Soc dự đoán robot từ goal nội bộ; CN native thiếu nội vật cản; hybrid dùng ORCA projection. Giữ signed gaps, không đổi policy để xóa failure. [Kế hoạch còn lại](NEXT_STEPS.md): cohort Gazebo normalized riêng trước khi so chung Nav2; AMCL/controller gốc giữ nguyên.

## Ghi chép các vòng trước — giữ theo thời điểm đo


Bước kiểm tra tiếp theo là [pilot localization riêng](benchmark/localization_trials/README.md), không thêm simulator mới hoặc thay native hành vi người. [Workaround shutdown](benchmark/shutdown_fix/README.md) chỉ giải quyết vòng đời Gazebo trên WSL D3D12 đã thử.

Đây là bảng so tính năng quan sát được ở các version cụ thể, không xếp hạng
hiệu năng, độ an toàn hoặc mức tự nhiên của hành vi người.

| Mục tiêu | SocNavGym vòng 1 | CrowdSimPlus vòng 2 | ComplexityNav vòng 3 |
|---|---|---|---|
| Nhóm đi có membership | Đã quan sát | Chưa có model tương đương trong bản thử | Clustering nhận diện, chưa có luật giữ nhóm |
| Nhóm đứng trò chuyện | Scene bố trí người quanh tâm, hướng vào nhau | Chưa có model tương đương | Static agents, chưa phải hội thoại |
| Hình thành/tan nhóm | Đã ghi sự kiện | Chưa có model tương đương | Phân cụm quỹ đạo, không phải tạo/tan nhóm theo ý định |
| Tránh người, phản ứng robot | Người đi riêng có phản ứng; nhóm chưa chủ động tránh robot | ORCA/SFM; đã thử visibility và choke point | ORCA/SFM/mixed policies; đã thử visibility |
| Crowd/lối hẹp | Chưa benchmark riêng đủ | Có cảnh hallway/bottleneck và case kẹt | Có width/density/directionality/policy mixture |
| Runtime đã chạy | Python 3.12 venv | Python 3.8.13 container | Python 3.8.13 container |
| Điều khiển người trong Gazebo | Adapter thử nghiệm 1 nhóm 3 người, native/modified variants | Đã chạy 3 người ORCA+/SFM qua Docker, cùng Mecanum/Nav2 | Chưa |

Bằng chứng: [vòng 1](reports/ROUND1.md), [vòng 2](round2/reports/ROUND2.md),
[vòng 3](round3/reports/ROUND3.md).

Nhà xưởng 60×40m + Mecanum/Nav2 đã chạy trong Gazebo. SocNavGym hiện có thêm [adapter một nhóm 3 người](gazebo_adapter/README.md), với hai mode đi cùng nhau và đứng trò chuyện. CrowdSimPlus đã có [backend Gazebo riêng](crowdsimplus_adapter/README.md), dùng native human ORCA+ kernel trong Docker Python 3.8. ComplexityNav vẫn chỉ ở demo độc lập. Adapter chưa đưa formation/dispersal vào Gazebo và đã có [pilot Nav2 ghép cặp RVO2/SocNavGym](benchmark/reports/BENCHMARK_REPORT.md).

[Kiểm định adapter](gazebo_adapter/reports/ADAPTER_REPORT.md) xác nhận simulation clock, pause/resume, robot feedback, pose proxy và phản xạ LiDAR. Giới hạn đáng chú ý: native ORCA nhóm không thêm robot vào phép tính tránh; probe gần/xa cùng seed cho quỹ đạo giống nhau khi chưa tiếp xúc. Cơ chế contact native có thể dừng nhóm khi overlap với robot. Lượt tuần tra dài còn ghi 199/440 trạng thái overlap giữa thành viên, clearance cặp nhỏ nhất −0.354 m. Vì vậy kết quả reactive của người đi riêng trong vòng 1 không chứng minh group walking đã chủ động tránh robot.

Không so trực tiếp tỷ lệ thành công hay clearance giữa ba vòng vì khác môi trường, controller, timestep và điều kiện sampling. Vòng 3 có thêm screening ban đầu; benchmark chung phải áp dụng cùng kiểm tra chất lượng cảnh cho mọi backend.

Pilot gồm 9 goal đạt, nhưng SocNavGym counterflow seed 43 có overlap với footprint Nav2 khoảng 2.0 s, sâu nhất 0.142 m; seed 42 có overlap giữa người trên 1 cm khoảng 0.8 s. RVO2 không có overlap trên 1 cm trong mẫu đo của pilot. Đây chưa phải xếp hạng thống kê. Sau pilot đã có [chẩn đoán seed 43](benchmark/diagnostic/reports/run01/DIAGNOSIS_REPORT.md), [normalization geometry](benchmark/geometry_comparison/reports/run01/GEOMETRY_REPORT.md) và [robot-aware group adapter](benchmark/avoidance_comparison/reports/run01/AVOIDANCE_REPORT.md). Cần ghi rõ sự khác nhau giữa tránh robot của người đi riêng và của nhóm; mọi chỉnh sửa hành vi được gắn nhãn adapter variant, không trình bày như upstream native. Sau khi lượt đó ổn định mới tăng mật độ và thử bottleneck. Không chạy hai backend điều khiển cùng người.

## CrowdSimPlus trong Gazebo

[Report 7 lượt mới](benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md): 4 CrowdSimPlus visible, 2 SocNavGym robot-aware adapter và 1 CrowdSimPlus invisible-robot control, tất cả đến đích. Hai CrowdSimPlus counterflow visible không có overlap robot–người/người–người trong sample đã ghi, robot clearance min khoảng 14,6/13,2 cm. Blind control có overlap sâu; goal success không đồng nghĩa an toàn.

Comparator SocNavGym ở vòng này là bản **modified robot-aware group adapter**, không phải native. CrowdSimPlus vẫn không có membership/cohesion/conversation; stationary fixture chỉ giữ người đứng yên. Runtime CrowdSimPlus hiện khá chậm với 168 static segments, RTF khoảng 0,52; khác biệt timing và các lượt Nav2 chưa được khóa chung noise không cho phép xếp hạng framework từ vài con số.

Cả hai backend dùng cùng initial fixture, map, robot, Nav2 và timestep 0,1 s (CrowdSimPlus original demo là 0,25 s). Source/runtime native được ghim, wrapper parity kiểm tra sai số 0 qua 30 bước. ComplexityNav chưa đưa vào Gazebo. Chưa chọn model người chính thức hoặc thay kiến trúc robot/World Model.

## CrowdSimPlus SFM trong Gazebo

[Report SFM/ORCA+ cùng fixture](benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md):
đã chạy demo SFM upstream trước, kiểm định parity sai số 0, rồi 4/4 goal Gazebo
đạt và không ghi nhận overlap trên 1 cm trong sample. SFM giữ khoảng hở robot
25,67/28,27 cm, ORCA+ 13,61/13,72 cm ở hai seed. **SFM không hoàn tất patrol**:
ba người dừng cách waypoint chung khoảng 0,616 m và không đổi goal, còn ORCA+
tiếp tục tuần tra. Không dùng số này xếp hạng policy hoặc gọi là hội thoại.

SFM vẫn là người độc lập, không giải quyết group walking/conversation/formation.
Bước kế cần fixture có đích riêng cho từng người, dùng chung cho SFM/ORCA+,
rồi mới thử giao cắt/lối hẹp. ComplexityNav vẫn chỉ ở demo độc lập.

## Fixture đích riêng: đã kiểm tra tuần tra trong Gazebo

[Report đích riêng](benchmark/individual_goals/reports/run01/INDIVIDUAL_GOALS_REPORT.md):
4/4 goal Nav2 đạt, mỗi người hoàn tất 2–4 chặng, 39/39 arrival events khớp pose
Gazebo và không ghi nhận overlap >1 cm trong sample. Chỉ đổi endpoint của mỗi
người, giữ initial poses, native force/arrival rule, map, robot và Nav2.

SFM đã tiếp tục tuần tra ở hai seed mới. Shared-endpoint control với robot cố
định cho 1/0/2 chặng, nên không khẳng định goal chung luôn làm cả ba kẹt.
Không có group/conversation model, chưa xếp hạng tính tự nhiên. Bước tiếp:
giao cắt rồi lối hẹp; ComplexityNav vẫn chưa nối Gazebo.

## Giao cắt SFM/ORCA+: đã chạy Gazebo

[Report crossing](benchmark/crossing/reports/run01/CROSSING_REPORT.md): kiểm định
native trước rồi 4/4 goal Gazebo/Nav2 đạt, mỗi người cắt qua tim lối 4–9 lần.
Không overlap >1 cm trong mẫu pose; 78/78 arrival events khớp proxy Gazebo.
SFM robot gap min 21,9/25,4 cm, ORCA+ 11,8/11,9 cm; không dùng bốn lượt để
xếp hạng tính tự nhiên. Nav2 vòng xuống phía dưới khu người đi qua lại.

Crossing có initial poses và endpoint mới, cùng fixture trong mỗi cặp policy;
giữ native parameters, map, robot và Nav2. Đây là ba người độc lập, chưa
có group/conversation. Bước tiếp theo là lối hẹp thật trong nhà xưởng.
ComplexityNav vẫn chưa nối Gazebo.

## Lối hẹp nhà xưởng: đã chạy, còn nhiễu định vị

[Report bottleneck](benchmark/bottleneck/reports/run01/BOTTLENECK_REPORT.md):
empty control qua khe, SFM/ORCA+ mỗi policy 1/2 goal đạt. Seed42 cả hai bị
Nav2 hủy goal; AMCL max error 2,23 m ở ORCA+, 0,45 m ở SFM, pose ước lượng
nằm vào khối máy. Seed43 cả hai qua hết khe thật và đến đích.

Cả ba người qua ít nhất một lần ở mọi lượt, nhưng SFM tiến sát tường và có
overlap cặp người 1,33 mm ở seed42. Không ghi overlap >1 cm trong sample.
Giữ native parameters/controller; chỉ đổi spawn/goal và AMCL initial pose
trong runtime config copy. Cần chẩn đoán định vị/scan/costmap trước khi dùng
kết quả lối hẹp để chọn model. Đây vẫn là ba người độc lập; ComplexityNav
chưa nối Gazebo, chưa có World Model hoặc chuyển Isaac Sim.

## Sau bottleneck: đã khoanh vùng yếu tố định vị

[Chẩn đoán offline](benchmark/localization_diagnosis/reports/run01/LOCALIZATION_DIAGNOSIS.md)
xác nhận ORCA+ seed42 có AMCL jump 1,992 m, trong khi odometry khớp Gazebo
trong khoảng 1 cm. Có mode sensor score cao trong khối occupied, đã kiểm
với thư viện AMCL native. Đây chưa phải replay PF hoặc bản sửa hành vi.
SFM seed42 dừng tịnh tiến trước khi tâm AMCL vào occupied; cần thu global
plan/costmap và particle cloud để tách tiếp. Kết quả 2/4 goal giữ nguyên.

Bổ sung [capture global navigation/particle cloud](benchmark/navigation_diagnostics/reports/run03/NAVIGATION_DIAGNOSTICS.md): SFM/ORCA+ seed42 mới đều qua khe và tới đích, chưa tái hiện failure cũ; không đổi kết quả ma trận gốc hoặc dùng để xếp hạng mô hình.

[Loạt lặp seed42 sau cleanup](benchmark/stability/reports/run01/STABILITY_REPORT.md): SFM2/3 và ORCA+1/3 goal; một mode switch AMCL được xác nhận từ cloud, SFM có proxy overlap2,65cm. Không dùng số lượt nhỏ bị ảnh hưởng định vị để chọn framework hoặc kết luận safety.

## Kiểm tra định vị nhiều seed: chưa chọn profile chung

[Cohort12 lượt](benchmark/localization_validation/reports/run01/MULTISEED_REPORT.md) giữ riêng với các cohort cũ: baseline đạt5/6 goal thật (SFM2/3, ORCA+3/3), LF-prob+beam-skip đạt5/6 (SFM3/3, ORCA+2/3). Không có false-success theo ngưỡng XY thật0.20m trong đợt này. Không kết luận profile mới tốt hơn hoặc policy người nào tự nhiên/an toàn hơn.

Đối chiếu1998 cloud, world và scene từng cặp, native parameters và GPU renderer. Có human-human overlap2.369cm ở baseline SFM seed44 dù goal đạt; dữ liệu vẫn giữ. Native robot/controller/map không đổi. Cấu hình mới chưa promote; mọi lượt shutdown sạch trong mode opt-in WSL D3D12. Phần runner/mở rộng seed44 và kết quả cohort mới chưa commit ở thời điểm ghi chú.
