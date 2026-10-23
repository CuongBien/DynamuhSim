# Bộ pilot perception LiDAR + depth có nhãn — 23/10/2026

**Nghiệm thu việc thu, tạo nhãn và xuất dữ liệu pilot; chưa nghiệm thu detector người hoặc bộ train World Model.** Giữ LiDAR + depth theo lựa chọn người dùng, không thêm RGB. Không sửa robot, Nav2 hoặc mặc định ROS. Một lượt development kiểm wiring, sau đó bảy lượt chính thức với cấu hình nguồn đã đóng băng, không thu lại/chọn lượt đẹp.

## Kết quả thực tế

| Cảnh | Tập | Chuyển động | Frame xuất | Cụm người | Cụm môi trường |
| --- | --- | --- | ---: | ---: | ---: |
| p001 | train | tĩnh | 1 | 4 | 3 |
| p002 | train | tĩnh | 1 | 4 | 3 |
| p003 | train | động | 5 | 20 | 20 |
| p004 | validation | tĩnh | 1 | 2 | 3 |
| p005 | validation | động | 5 | 10 | 15 |
| p006 | test | tĩnh | 1 | 4 | 4 |
| p007 | test | động | 5 | 20 | 20 |
| Tổng | | | 19 | 64 | 68 |

Train có 54 cụm/7 frame, validation 30/6, test 48/6. Đây là 7 cảnh có tương quan theo 3 họ bố cục; không phải 132 người hay 132 episode độc lập. Bốn cảnh tĩnh chỉ xuất frame giữa (index2); ba cảnh động xuất 5 frame liên tiếp/cảnh. Không có cụm bị bỏ vì nhãn kém: cả 132 proposal baseline không lọc đều giữ lại; lần thu này không phát sinh nhãn ambiguous. Unit tests vẫn kiểm trường hợp mixed/unknown và không cho biến chúng thành negative.

Nguồn chính thức gồm **525 ảnh depth** (=7 cảnh ×15 camera đối chứng ×5 timestamp) và **140 LaserScan native**. Trong 525 ảnh chỉ 35 ảnh là full input camera gốc; 490 ảnh còn lại là control/counterfactual LABEL ONLY, không phải augmentation đầu vào. 19/35 full frame được chọn theo quy tắc đã chốt trước thu. Có một development preflight 75 ảnh/20 scan khác, không nhập corpus.

Cả 7 lượt: Ogre2 thực báo NVIDIA RTX4060 Laptop qua D3D12, full/control cùng timestamp khớp từng byte, 6 mesh được nhận diện đúng bởi render listener, callback cân bằng, pause/clock ổn định rồi thoát Gazebo mã0, không còn process của phiên. Physics, ROS transport, NumPy export/QA vẫn dùng CPU với số luồng giới hạn; không tuyên bố toàn bộ pipeline chạy CUDA. Không mở GUI mới trong lượt thu này.

Ba cảnh động thực dịch chuyển tối đa 0.39m trong cửa sổ quan sát; trace ECM actor/proxy khớp trong tolerance5mm. Đây là **chuyển động chẩn đoán được lập trình** và thay pha walk animation, không phải native social policy của SocNavGym/CrowdSimPlus/ComplexityNav. Robot đứng yên. Hộp/trụ/cầu và một số cặp cọc tạo clutter cong/che khuất; mesh người và protective cylinder của LiDAR được kiểm vị trí nhưng khác bề mặt.

## Nhãn và thời gian

Pixel người phải đồng thời thỏa drop-one lộ bề mặt phía sau và only-one khớp full trong1mm. Null không giải thích được/ownership mâu thuẫn sẽ là unknown−1. Tỷ lệ unknown đo được0, so với trần0.5% đã chốt. Có45 person-frame instance nhìn thấy ≥20pixel nhưng chỉ còn <90% silhouette in-view. Không dùng người ngoài FOV/ngoài range để tuyên bố fully-occluded, và không nghiệm thu detector/track dưới che khuất từ con số này.

Proposal person cần đúng1person ID, ≥5pixel điểm, purity≥90%, không unknown; environment cần tất cả điểm environment. Class−1/validfalse được dành cho trường hợp không rõ. [Schema](SCHEMA.md) mô tả kiểu dữ liệu, đơn vị và masking. Các ID1–6 chỉ là instance LABEL trong simulator, không phải re-ID. H1–H5 toàn bộ index−1/validfalse.

**Có thiếu LiDAR ở3/19frame, ứng với21/132proposal**: các frame đầu2s của ba cảnh động. Capture chỉ giữ scan từ2s; scan đồng timestamp tới sau thời điểm availability của depth nên loader không dùng. 111proposal còn lại có scan nhân quả hợp lệ (thường stamp depth−0.2s). 21sample thiếu giữ `scan_valid=False`, rangesNaN và stamp−1. Không chèn scan tương lai, không đổi timestamp, không loại frame, không nội suy. Nếu baseline sau dùng LiDAR bắt buộc phải mask các sample này hoặc chọn depth-only cho chúng theo protocol chốt trước huấn luyện. Availability là simulation-clock proxy, không phải latency wall-clock chính xác.

## Kiểm định và lỗi checker được giữ lại

9 unit tests đạt. Audit đối chiếu raw→portable từng frame, point/pixel/proposal, nhãn tính lại độc lập, native scan, nguồn SHA, chia toàn cảnh/họ và fingerprint frame lượng tử1mm không trùng giữa split. **Không chạy classifier hay preview test**, chỉ integrity/label QA tự động.

Auditor đầu `audit.py` (đã frozen) thất bại vì thêm điều kiện mọi sample đều phải có scan, dù loader và thiết kế trước thu đã cho phép mask missing. [Failure](audit01_failure.json) và source nguyên bản vẫn giữ; [diff auditor02](audit02_change.diff) bỏ đúng giả định dư này, đồng thời kiểm scan được chọn thực sự là mới nhất đủ điều kiện và missing chỉ khi không có scan nhân quả. [Auditor02](audit02.py) được bổ sung sau thu, không thay code capture/export/loader/label/protocol/frozen hoặc raw data. [Audit cuối](audit.json) PASS chỉ chứng minh integrity theo missing-scan policy; không chứng minh mọi sample đủ hai modality hay semantic accuracy. Auditor01 vẫn tái hiện FAIL nếu chạy; dùng command auditor02 bên dưới để kiểm định cuối.

Kiểm portable trên bản copy sang thư mục tạm:84sample train/validation đọc được bằng NumPy, không import ROS/Gazebo; xóa pixel/visibility/provenance LABEL và thay proposal labels bằng nhãn trái ngược không đổi input. Xóa mọi depth/scan tương lai ở13frame train/validation không đổi sample hiện tại;2scene test bị API chặn mặc định. [Portable audit](portable_audit.json). Dữ liệu gốc không đổi sau các can thiệp này.

Nguồn34file đóng băng tại `frozen.json` trước lượt chính thức, gồm mover binary, fixture, geometry/labels, loader/export và calibration. Giữ checkpoint HEAD `c67430e` và SHA nguồn từng lượt trong reports/captures. Preflight ở epoch nguồn trước comment-header cleanup, chỉ dùng development; không tính như một test độc lập. Presentation script `preview02.py` chỉ sửa độ rộng/tiêu đề hình, bản frozen `preview.py` giữ nguyên.

Runtime scene/world và trace ECM được ghi hash riêng trong `runtime_artifact_hashes.json`; mesh asset cũng khớp SHA checkpoint trước thu. Hai file `runtime/snapshot.json` và `runtime/fixture_snapshot.json` được helper cũ tạo ở bước dựng nền, vẫn chứa bố cục nền cũ: **không phải thiết kế cuối hoặc GT lượt này**. Chỉ dùng `scene_LABEL_ONLY.json`/`warehouse.sdf` cho thiết kế cuối, `motion_LABEL_ONLY.jsonl` cho vị trí ECM thực đo và pixel intervention để tạo nhãn. Giữ nguyên các file raw cũ này để truy vết, không sửa thành metadata khác sau thu.

## Dữ liệu và ảnh/GIF thật

Bundle đầy đủ: `generated/corpus01/`, 973,543 byte (≈0.97MB thập phân), đọc độc lập bằng Python+NumPy, không ROS/Gazebo. [Mẫu train p003](../examples/train_p003/manifest.json) giữ đủ5frame/40proposal, 251,297 byte, được lưu Git. Raw intervention/native log ở `generated/runs/p001`…`p007`. **Generated/raw/bundle đầy đủ bị gitignore, cần sao lưu riêng; commit không thay thế backup dữ liệu.** `bundle_hashes.json` giữ kiểm tổng portable. Git có công cụ, protocol, freeze, reports, native GPU logs, small train example và media.

[Ảnh](media/depth_labels.png), [GIF](media/depth_labels.gif) là phát lại5depth frame đã ghi của **train p003**, kèm màu instance LABEL và các point cloud chiếu từ depth. **Không dùng AI sinh ảnh/GIF.** Màu đỏ/xanh trên panel proposal là nhãn đối chứng, không phải dự đoán của một model. Panel phải là BEV của các điểm depth nhìn thấy, không phải bản đồ occupancy hoàn chỉnh; vùng không thấy không được suy diễn trống. `media_provenance.json` ghi scene/split/nguồn.

## Tương thích, giới hạn và bước sau

Đã build/run thực tế Ubuntu24.04, ROS2Jazzy, GazeboHarmonic8.15.0, Python3.12.3/NumPy1.26.4/SciPy1.11.4; không cài dependency mới trong mốc này. Offline loader không phụ thuộc backend Gazebo, nhưng scene/label/render plugin này riêng Gazebo; chuyển simulator vẫn phải làm adapter/capture/label validation.

Một nhà xưởng, một avatar `walk_blue.dae`, góc và pha bước chân khác nhau; chưa đa dạng mesh/robot motion/real-world/cross-simulator. 19frame quá nhỏ để tuyên bố model tổng quát. Native social behavior, thành công navigation, re-ID/association hay H1–H5 không được kiểm mới ở đây. Các navigation TIMEOUT/gateFAIL cũ không đổi; HuNavSim vẫn giữ cho kiểm ngoài sau này. Default ROS/Nav2, dataset cũ và12staged entry người dùng được bảo toàn; [preservation](preservation_before_commit.json) kiểm15,289file tracked hiện diện trước mốc này không đổi.

Bước tiếp theo: lập baseline person-vs-environment từ sensor-only bằng bộ train nhỏ này; chốt feature/model/metric trước fit, chọn cấu hình trên validation, rồi đóng băng trước mở test cho đánh giá một lần. Đây là kiểm pipeline perception; nếu không đạt phải thêm đa dạng data phù hợp. Sau semantic acceptance còn association/occlusion độc lập trước batch World Model. **Chưa bật training eligibility hoặc sửa ROS/Nav2 từ kết quả corpus này.**
