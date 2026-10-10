# Đặc Tả Đo Thực Tế Hiệu Năng Gazebo Harmonic (Gazebo Harmonic Performance Benchmark Spec)

**Mục đích**: Đo thực tế Gazebo Harmonic trên máy phát triển để quyết định **dùng Gazebo cho toàn bộ sinh dữ liệu** hay **tách (sinh quỹ đạo/nhãn bằng sim nhẹ, chỉ render depth trong Gazebo)**. Không chứa code; đặc tả để tự cài đặt.

---

## 1. Câu Hỏi Cần Trả Lời

1. **RTF (Real-Time Factor)** của Gazebo là bao nhiêu khi có depth + lidar + actor?
2. **Yếu tố nào tốn tài nguyên nhất**: số người, mesh actor, độ phân giải/tần số depth, hay GUI?
3. **Chạy song song nhiều instance** có tăng tổng thông lượng (throughput) không?
4. **Có phát lại (replay) được quỹ đạo** đã sinh trong Gazebo để render depth, lặp lại được theo seed, và nhanh hơn thời gian thực không?
5. **Chi phí lưu trữ** mỗi khung/episode là bao nhiêu?

---

## 2. Môi Trường Đo (Ghi lại trước khi chạy)

* **Phiên bản**: Gazebo Harmonic, ROS 2 Jazzy, driver NVIDIA, CUDA, kernel Linux.
* **Phần cứng**: CPU Ryzen 7, GPU RTX 4060 8GB VRAM, 24GB RAM; chế độ nguồn / CPU governor (đặt `performance`), nhiệt độ ban đầu.
* **Môi trường chạy**: Đóng mọi ứng dụng nặng khác; chạy headless (`-s`, headless rendering) trừ khi đo GUI.
* **Physics**: Ghi nhận `max_step_size` và `real_time_update_rate` trong SDF (giữ cố định giữa các lần đo).
* **Robot**: TurtleBot3 Burger, di chuyển bằng chuỗi `cmd_vel` cố định (cùng kịch bản cho mọi lần chạy).

---

## 3. Cấu Hình Cơ Sở (Baseline)

| Tham Số | Giá Trị Cơ Sở |
| :--- | :--- |
| **World** | `corridor_090.sdf` |
| **Số người $N$** | 5 (actor `walk_blue.dae` + trụ va chạm $r = 0.28\text{ m}$) |
| **Nguồn hành vi người** | Plugin RVO2 C++ (`librvo2_human_system.so`) |
| **LiDAR** | LDS-01, 360 tia, 5 Hz (`gpu_lidar`) |
| **Depth Camera** | $640 \times 480$ @ 30 Hz |
| **Chế độ** | Headless (`-s`) |
| **Seed** | Cố định, ghi lại |

---

## 4. Ma Trận Thí Nghiệm (Đổi một yếu tố mỗi lần, các yếu tố khác giữ cơ sở)

| Nhóm | Yếu Tố | Các Mức Đánh Giá |
| :---: | :--- | :--- |
| **A** | **Cảm biến** | Không cảm biến; chỉ LiDAR; chỉ Depth; LiDAR + Depth |
| **B** | **Số người $N$** | 0, 2, 5, 10, 24 |
| **C** | **Biểu diễn người** | Actor + trụ va chạm (hiện tại); Chỉ trụ va chạm (không mesh); Chỉ actor (nếu khả thi) |
| **D** | **Độ phân giải & tần số Depth** | 320x240@15; 320x240@30; 640x480@15; 640x480@30; 848x480@30 |
| **E** | **World** | `corridor_090.sdf`; `School Arena`; `Hospital Medium` |
| **F** | **Nguồn hành vi** | RVO2 plugin; HuNavSim qua cầu nối Docker |
| **G** | **Giao diện (GUI)** | Headless (`-s`); Có GUI |

*Ghi chú*:
* Nhóm **C** cho biết riêng chi phí render mesh của actor.
* Nhóm **F** cho biết chi phí cầu nối (đồng bộ đồng hồ, độ trễ truyền nhận pose).

---

## 5. Quy Trình Mỗi Lần Đo

1. Khởi động world, đợi ổn định 10 s thời gian thực (warm-up, bỏ qua không tính vào chỉ số).
2. Chạy 120 s thời gian thực với kịch bản robot di chuyển cố định.
3. Lấy mẫu mỗi 1 s: sim time và wall time để tính RTF theo cửa sổ trượt 1 s.
4. Lặp lại **3 lần** cho mỗi cấu hình; báo cáo giá trị trung bình $\pm$ độ lệch chuẩn.
5. Tắt hoàn toàn tiến trình (`gz sim`, ros bridge, node) rồi mới chạy cấu hình tiếp theo.

---

## 6. Chỉ Số Cần Ghi Nhận

| Chỉ Số | Cách Lấy | Ghi Chú |
| :--- | :--- | :--- |
| **RTF** | $\Delta\text{sim\_time} / \Delta\text{wall\_time}$ | Báo cáo Trung vị (median), p5 (phân vị 5% - trường hợp xấu nhất) và Trung bình |
| **Tần số thực của topic** | Tần số đo được của `/scan`, `/camera/depth_image` so với danh định | Rớt tần số = cảm biến bị nghẽn không kịp render |
| **CPU** | % tổng CPU và % theo tiến trình `gz sim` | Giám sát qua `top` / `psutil` |
| **GPU** | % sử dụng Compute, VRAM đang dùng (MB) | Lấy qua `nvidia-smi` mỗi giây |
| **RAM** | Dung lượng RAM hệ thống sử dụng (MB) | Lấy qua `free -m` / `psutil` |
| **Thời gian khởi động / reset** | Từ lệnh kích hoạt reset đến khung cảm biến hợp lệ đầu tiên | Quyết định thông lượng sinh dữ liệu theo episode |
| **Độ lặp lại** | Chạy 3 lần cùng seed, so sánh chuỗi pose người và robot | Báo cáo độ lệch pose lớn nhất ($\max \|\Delta \mathbf{p}\|$) |

*Với cấu hình dùng HuNavSim (nhóm F), ghi thêm*: độ lệch đồng hồ host–Docker và độ trễ từ lúc HuNav tính pose đến lúc áp vào Gazebo.

---

## 7. Thí Nghiệm Song Song (Parallel Scaling)

* Chạy đồng thời $1, 2, 3, 4$ instance cấu hình cơ sở; mỗi instance sử dụng Gazebo partition (`GZ_PARTITION`) và ROS domain (`ROS_DOMAIN_ID`) độc lập.
* Ghi nhận:
  * Tổng thông lượng: $\text{Throughput}_{\text{total}} = \sum \text{RTF}_i$.
  * RTF của từng instance.
* Điểm dừng: Dừng khi tổng thông lượng không tăng thêm, hoặc GPU VRAM / RAM chạm trần ($> 90\%$).

---

## 8. Thí Nghiệm Phát Lại (Replay) Để Render Depth

*Mục đích: Kiểm tra tính khả thi của phương án TÁCH (quỹ đạo từ sim nhẹ 2D, depth từ Gazebo).*

1. Tạo một file quỹ đạo mẫu (pose người và robot theo thời gian, 10 Hz, thời lượng 20 s).
2. Phát lại trong Gazebo bằng cách đặt pose từng bước, chụp depth và lidar sau mỗi bước.
3. Kiểm tra xem có chạy được ở chế độ bước (tạm dừng sim, tiến $n$ bước qua service) để chạy nhanh hơn thời gian thực hay không; ghi rõ nếu chức năng này không hỗ trợ.
4. Đo đạc:
   * Số khung hình render mỗi giây (FPS).
   * Thời gian tổng để hoàn tất 1 episode 20 s.
   * Độ lặp lại: Hai lần phát lại có cho ra depth map giống hệt nhau hay chỉ gần giống (báo cáo độ lệch pixel lớn nhất).
5. Kiểm tra tính đúng đắn hình học:
   * Người xuất hiện trong depth map đúng vị trí tương ứng.
   * Actor và trụ va chạm không bị lệch pose so với file quỹ đạo.

---

## 9. Chi Phí Lưu Trữ Dữ Liệu (Storage Budget)

* Với 1 episode mẫu, ghi nhận dung lượng khung depth theo từng định dạng:
  * 16-bit PNG (nén không mất dữ liệu).
  * `.npy` nén (`np.savez_compressed`).
  * `.npz` nén.
  * Vector LiDAR.
* Quy đổi ra dung lượng lưu trữ trên $1,000$ episodes.

---

## 10. Quy Tắc Quyết Định Kiến Trúc (Decision Matrix)

Điền trước mục tiêu bài toán:
* **Số khung depth cần có**: $T$ khung.
* **Ngân sách giờ sinh dữ liệu**: $B$ giờ (với $k$ instances chạy song song).

**Quy tắc**:
* Thông lượng Gazebo $\approx (\text{khung/giờ từ Mục 8 hoặc từ RTF}) \times k$.
* **Trường hợp 1**: Nếu $\text{Thông lượng} \times B \ge T$ và độ lặp lại đạt chuẩn $\rightarrow$ **Dùng Gazebo cho toàn bộ pipeline sinh dữ liệu**.
* **Trường hợp 2**: Nếu không đạt $\rightarrow$ **Dùng phương án TÁCH**:
  * Sim nhẹ 2D sinh quỹ đạo và nhãn hành vi H1–H5.
  * Gazebo chỉ render depth cho một tập con đại diện.
* **Trường hợp 3**: Nếu ngay cả tập con vẫn quá chậm $\rightarrow$ Thiết kế World Model nhận đầu vào đã trừu tượng hóa từ depth (ví dụ: BEV tọa độ người đã phát hiện), giảm số lượng khung depth raw cần thiết.

---

## 11. Bảng Kết Quả Thực Nghiệm (Điền sau khi chạy)

| Cấu Hình | RTF Trung Vị | RTF p5 | Hz LiDAR | Hz Depth | GPU % | VRAM (MB) | RAM (MB) | Ghi Chú |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Baseline (Cơ sở)** | | | | | | | | |
| **A1: Không cảm biến** | | | | | | | | |
| **A2: Chỉ LiDAR** | | | | | | | | |
| **A3: Chỉ Depth** | | | | | | | | |
| **B1: N = 0 người** | | | | | | | | |
| **B2: N = 2 người** | | | | | | | | |
| **B3: N = 10 người** | | | | | | | | |
| **B4: N = 24 người** | | | | | | | | |
| **C1: Chỉ trụ (không mesh)**| | | | | | | | |
| **D1: Depth 320x240@15** | | | | | | | | |
| **D2: Depth 320x240@30** | | | | | | | | |
| **D3: Depth 640x480@15** | | | | | | | | |
| **D4: Depth 848x480@30** | | | | | | | | |
| **E1: School Arena** | | | | | | | | |
| **E2: Hospital Medium** | | | | | | | | |
| **F1: HuNavSim Docker** | | | | | | | | |
| **G1: Có GUI** | | | | | | | | |

---

## 12. Điều Cần Xác Minh Khi Cài Đặt (Prerequisites Verification)

1. Gazebo Harmonic có chế độ bước đồng bộ (Step mode) ổn định với cảm biến và actor không.
2. Actor có được điều khiển theo pose phát lại mà hoạt ảnh chân vẫn hợp lý không.
3. Tên topic, tên service và cờ headless rendering chính xác trên máy phát triển (không giả định trước).
