# Nguồn dữ liệu và nhãn hành vi

Quyết định cho bước xây dựng dữ liệu: dùng SocNavGym, CrowdSimPlus và ComplexityNav theo năng lực đã kiểm; HuNavSim hiện có dành cho kiểm thử cuối, không dùng chọn tham số. Chưa thu hàng loạt hoặc train World Model.

| Nguồn | Năng lực đã có bằng chứng trong project | Dùng cho dữ liệu | H1–H5 |
|---|---|---|---|
| SocNavGym lifecycle + response | Nhóm đi/đứng, membership/lifecycle; đứng v3 nhích tránh robot có giới hạn; 2 standing headless đạt goal thật | Nhóm, tương tác; ghi rõ variant/projection | Chưa xác thực, null + mask |
| CrowdSimPlus ORCA+ | Cá nhân né tương tác, native replay + Nav2; 1/2 lượt goal thật đạt | Baseline cá nhân; có thể mở thêm SFM sau kiểm định recorder | Không gán H1 chỉ vì ORCA |
| ComplexityNav geometry | Native intention + roadmap và ORCA projection; 2 Nav2 pilot đạt | Cảnh geometry/lối hẹp; giữ metadata hybrid | Không gọi trained RGL hay suy H1/H5 |
| HuNavSim đang có | Hệ thống test người dùng đã có; chưa audit đủ persona trong mốc này | Held-out test, cần exporter theo contract sau | Chỉ dùng nhãn đã kiểm mechanism |

H1 cooperative (α=.5), H2 unyielding (α=.10–.35), H3 robot-blind, H4 đứng/trò chuyện, H5 soft repulsion. Đây là cơ chế/nhãn nghiên cứu, không phải tên simulator. Với H3 phải chứng minh tất cả tầng guard/projection đều không phản ứng robot. Đứng v3 có phản ứng khác H4 tĩnh; chỉ ghi group_mode=standing và response_variant, không tự gắn H4.

Source cũ rvo2_human_system.cpp có code các persona, nhưng cần kiểm hành vi xuyên suốt trước khi coi là ground truth. Persona hiện tại của ba adapter mới chưa đạt điều kiện đó. M3 vì vậy có nhãn unknown/mask, chưa có supervision H1–H5 hợp lệ.

M1: tương lai quỹ đạo người 3s theo hành động robot. M2: BEV từ depth, không RGB; horizon BEV tương lai còn cần chốt. M3: xác suất H1–H5 từ lịch sử 2s. M4 là đánh giá robot thật. Pilot chỉ chuẩn bị dữ liệu; không triển khai các model.

Đa nguồn không tự đồng nghĩa đa cơ chế: các nhánh ORCA/projection có thể giống nhau. Khi chia train/validation, tách cả episode/seed/scenario family; không chia các cửa sổ chồng lấn của một episode sang hai tập. Metadata backend chỉ phục vụ provenance/stratification, không làm đầu vào model.

Gazebo/Isaac là tầng vật lý/cảm biến, độc lập với human backend. Contract SI/frame/sim-time giữ nguyên; khi chuyển Isaac phải viết exporter/calibration/kiểm clock và va chạm, rồi tái QA. Không cần bỏ toàn bộ dữ liệu/pipeline đã chuẩn hóa, cũng không phải thao tác copy là chạy ngay.

Bằng chứng: ../socnavgym_group_nav2/reports/GROUP_NAV2_REPORT.md; ../shared_nav2_pilot/reports/SHARED_NAV2_REPORT.md; docs/SOCIAL_NAV_TRAINING_PIPELINE.md trong repository.
