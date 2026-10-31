# Batch đầu dữ liệu robot chuyển động cho World Model

**Đã thực hiện 10 lượt: 8 episode hợp lệ, 2 lượt cách ly.** Có 5,106 cửa sổ, trong đó 5,094 đủ điều kiện target BEV quan sát tương lai. Bộ này dùng được để **train thử WM dự báo BEV quan sát**, theo đúng split và mask. Chưa huấn luyện WM hoặc RL trong lượt này.

Robot chạy bằng Nav2 thật trong Gazebo. Giữ nguyên cấu hình moderate đã kiểm: start (−28,0), goal (−20,0), budget 90 giây mô phỏng, bán kính 0.25 m, padding 0.01 m, inflation 0.8 m, tốc độ vx 0.30/−0.15 và vy 0.20 m/s. Không script đường robot, không GT người vào planner, không đổi mặc định ROS/Nav2 hoặc source framework cũ. Render cảm biến dùng NVIDIA RTX 4060/D3D12; physics, Nav2, worker và export/audit vẫn dùng CPU với BLAS/OMP một luồng.

## Toàn bộ lượt thu

| Episode | Tập | Nav2 | Thu dữ liệu | Cửa sổ | WM BEV hợp lệ |
| --- | --- | --- | --- | ---: | ---: |
| wm_standing_45 | train | TIMEOUT | Đạt | 851 | 851 |
| wm_walking_45 | train | SUCCEEDED | Đạt | 597 | 597 |
| wm_standing_46 | train | SUCCEEDED | Đạt | 259 | 259 |
| wm_walking_46 | train | SUCCEEDED | Đạt | 268 | 268 |
| wm_standing_47 | train | TIMEOUT | Cách ly: exit 255 | 0 | 0 |
| wm_walking_47 | train | TIMEOUT | Đạt | 800 | 788 |
| wm_standing_48 | validation | TIMEOUT | Đạt | 802 | 802 |
| wm_walking_48 | validation | TIMEOUT | Đạt | 850 | 850 |
| wm_standing_49 | test | SUCCEEDED | Đạt | 679 | 679 |
| wm_walking_49 | test | TIMEOUT | Cách ly: exit 255 | 0 | 0 |

Timeout được giữ nếu dữ liệu hợp lệ; không chạy lại để chọn kết quả tới đích. Hai lượt standing47 và walking49 đóng MCAP mã 0, không worker fault, nhưng Gazebo thoát mã 255. Chúng vẫn FAIL tiêu chí clean-exit và không có trong portable training corpus. Callback filter cân bằng lần lượt 1028/1028 và 1034/1034, cleanup rỗng; những bằng chứng này không thay thế clean-exit gate. **Lỗi shutdown còn tái diễn, chưa sửa dứt điểm.** Raw/log của cả hai lượt được giữ.

Pipeline ban đầu dừng ở lượt thứ năm. [Review01](quarantine_review01.json) cho phép chạy tiếp đúng năm lượt chưa thử bằng cùng frozen driver/settings, cách ly standing47, không retry hoặc thay thế. Lượt cuối walking49 cũng FAIL; [Review02](quarantine_review02.json) chỉ đóng sổ cuối với 8 accepted + 2 quarantine, không chạy thêm. Hai [diff accounting đầu](audit_quarantine_accounting.diff) và [cuối](audit_final_quarantine_accounting.diff) giữ rõ thay đổi số lượng; ngưỡng chấp nhận, capture/export/target math và raw cũ không đổi. Capture, export và continuation có source freeze riêng.

## Split và số lượng

| Tập | Episode hợp lệ | Cửa sổ xuất | WM BEV hợp lệ |
| --- | ---: | ---: | ---: |
| train | 5 | 2775 | 2763 |
| validation | 2 | 1652 | 1652 |
| test | 1 | 679 | 679 |

Chia trước thu theo toàn seed family: 45–47 train, 48 validation, 49 test. Các cửa sổ chồng lấn không bị chia sang tập khác. **Test chỉ còn cảnh standing49** vì walking49 bị cách ly, nên batch này không kiểm độc lập được walking trên test. Same warehouse/route/avatar vẫn xuất hiện ở các tập; đây là seed holdout, chưa phải holdout môi trường/người/simulator.

Đây là batch SocNavGym đầu tiên: sáu người, nhóm đứng hoặc đi ban đầu, native lifecycle và reactive projection hiện có. Nhóm có thể tan; không khẳng định cả sáu người trò chuyện suốt episode. CrowdSimPlus chưa thêm ở batch này, HuNavSim vẫn giữ riêng và chưa sử dụng. H1–H5 không được suy từ tên simulator hoặc tư thế.

## Dữ liệu đã có cho WM

Đã kiểm **6,696 ảnh depth khớp MCAP từng byte** và 3,349 native LaserScan trong 8 episode hợp lệ. MCAP giữ đủ 16 topic. Mỗi cửa sổ có lịch sử 2 giây/20 query ở 10 Hz: LiDAR, depth, IMU, odom, lệnh vx/vy/wz và goal từ AMCL có mask tuổi dữ liệu. Scan gốc 5 Hz được giữ theo mẫu đã khả dụng, không tạo scan giả ở 10 Hz. Camera thấp và kích thước robot giữ nguyên; RGB observer chỉ là camera debug bên ngoài, không đầu vào robot.

Normalizer xét 5,205 anchor, giữ 5,106, loại 99 do thiếu hỗ trợ history/future label/action theo contract. Raw và lý do loại được giữ. Trong cửa sổ đã xuất, thêm 12 cửa sổ thiếu đủ target BEV được mask, không bù bằng frame tương lai hay GT trong input.

Input BEV dùng depth và odometry nhân quả để warp về anchor ước lượng. Target ở **+0.5/+1/+3 giây** dùng depth tương lai thực đã ghi, warp về hệ trục robot thật tại anchor; GT ego pose chỉ nằm ở nhánh LABEL. Grid 64×64, cell 0.1 m; unknown −1, observed free 0, occupied 1. Loss chỉ dùng cell/frame hợp lệ. Đây là occupancy thực quan sát, không toàn cảnh, social cost hoặc dự đoán do model sinh; input ước lượng và target trục thật vẫn có sai số căn chỉnh.

Accessor `.action_condition(i)` đọc riêng 30 lệnh tương lai đã chạy ở t…t+2.9 s để làm điều kiện hành động cho WM. `.observation(i)` không chứa future sensor/GT. Dữ liệu theo behavior policy Nav2, không đo phản thực cho mọi action và không tự coi là expert policy.

Human future/group GT được giữ ở LABEL để truy vết; association, H1–H5 và RL reward vẫn mask false. Reward NaN không có nghĩa reward bằng 0. Success/truncation được ghi riêng; chưa có reward/contact truth hoặc Gym step/reset để train RL online. Không cần classifier hoàn hảo mới ghi chuỗi cảm biến cho nhiệm vụ WM observed BEV.

## Kiểm chứng và sử dụng

8 unit tests trước export freeze đạt. [Audit cuối](audit.json) kiểm source/inventory SHA, thời gian/availability nhân quả, raw future actions/human labels, toàn bộ target pose/time/mask và whole-family split. Đã replay 96 grid ở first/middle/last của mỗi episode bằng pose/projection độc lập và raster đã chốt; không có history depth lượng tử 1 mm + action/odom trùng giữa các split. 7 episode train/validation đọc ở thư mục khác vẫn giữ observation khi xóa LABEL trên bản copy. Test API mặc định bị chặn; chỉ integrity/label QA tự động, không WM fitting/tuning/scoring hoặc xem/công bố ảnh test.

Native driver/normalizer tự tạo debug media/BEV preview ở raw, nhưng test media không được xem/chọn và không có trong portable input. Các tag `pilot_only`/overall training false của legacy normalizer được giữ để truy vết; split và **task eligibility mới** lấy từ dataset.json, episode.json và task masks. WM mask true ở test không cho phép train trên test. Xem [schema và API](SCHEMA.md).

[Ảnh train](media/wm_sample.png), [GIF Gazebo thật 4×](media/native_observer_train_4x.gif) và [window mẫu](../examples/window01/metadata.json) lấy cố định index 50 của standing45. **Không AI sinh ảnh/GIF, không có WM prediction**: cột tương lai là LABEL từ depth đã ghi. Gray = unknown, trắng = observed free, đỏ = observed occupied; cone tương lai dịch do camera tương lai được warp về anchor.

Full portable bundle ở `generated/dataset01/`, **3,008,185,798 byte (khoảng 3.01 GB)**. MCAP của cả 10 lượt là 524,047,141 byte; paired arrays/runtime khác cần backup riêng nếu muốn trọn bằng chứng. **Git không chứa full raw/bundle**; Git giữ code, protocol/freeze, audit/reports, native logs và một window mẫu. Copy toàn bộ dataset01 để loader chạy bằng Python + NumPy, không ROS/Gazebo/Torch; giữ raw để kiểm lại MCAP. [Inventory](dataset_inventory.json) lưu SHA/size. Các file đã có và staging người dùng được kiểm bảo toàn trong [preservation](preservation_before_commit.json).

Mốc thu/chuẩn hóa batch đầu đã xong với 2 quarantine rõ ràng. Bước tiếp theo là train thử WM observed BEV bằng train, chọn trên validation rồi cố định trước test standing49. Đây vẫn là pilot nhỏ, tương quan, cùng route/avatar và một backend; chưa nghiệm thu các head social-navigation khác, RL hay tổng quát qua simulator.
