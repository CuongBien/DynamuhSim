# BÁO CÁO THỰC NGHIỆM A4: ĐO THÔNG LƯỢNG SONG SONG VÀ CHI PHÍ NAV2 MPPI TRÊN ROBOT MECANUM

**Môi trường:** Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic 8.x  
**Phần cứng:** AMD Ryzen 7 7735H with Radeon Graphics (16 vCPUs), NVIDIA GeForce RTX 4060 Laptop GPU (8GB VRAM)  
**Cảm biến:** Depth Camera D435i ($424 \times 240$ @ 10 Hz), 2D LiDAR (5 Hz)  
**Cơ chế:** Gỡ trần thực tế (`real_time_factor = 0.0`), loại bỏ 5.0s warm-up, đo `/clock` trong bộ nhớ tiến trình (`rclpy`).  

---

## 1. BẢNG THÔNG LƯỢNG VÀ KHẢ NĂNG MỞ RỘNG (SCALING MATRIX VỚI NAV2 MPPI)

| Cấu hình | Số người | RTF mỗi inst (Mean ± Std) | Tổng RTF | Cold Restart (Định nghĩa mới: Max(Nav2 active, First depth)) | Thời gian 1 ep (20s) | Thông lượng (ep/giờ) | Thời gian cho 10k ep |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1 Instance** | N=5 | 2.82 ± 0.30 | **2.82 ± 0.30** | **4.88 ± 0.78 s** (Min 3.95s, Max 5.98s) | 12.0 s | **300.4 ep/h** | **33.29 h** |
| **4 Instances (Tải song song)** | N=5 | 1.11 ± 0.01 | **4.46 ± 0.02** | **5.88 ± 0.80 s** (Min 5.29s, Max 8.30s) | 23.8 s | **604.2 ep/h** | **16.55 h** |


---

## 2. PHÂN TÍCH TẢI PHẦN CỨNG & CHI PHÍ NAV2 MPPI VS KHÔNG NAV2

| Cấu hình | Tải CPU tổng | `gz sim` | `bridge` | `nav2` | ROS nodes | Tải GPU | VRAM sử dụng |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1 Inst (N=5, Nav2)** | 43.4% | 169.9% | 86.7% | 190.4% | 31.9% | 25.6% | 219 MB |
| **2 Inst (N=5, Nav2)** | 70.8% | 260.9% | 135.1% | 317.8% | 50.8% | 24.8% | 314 MB |
| **3 Inst (N=5, Nav2)** | 81.3% | 287.1% | 153.7% | 401.1% | 59.4% | 20.1% | 409 MB |
| **4 Inst (N=5, Nav2)** | 85.7% | 303.9% | 161.8% | 457.4% | 61.8% | 11.0% | 513 MB |
| **1 Inst (N=10, Nav2)** | 35.4% | 146.8% | 65.0% | 134.7% | 22.7% | 13.0% | 219 MB |
| **4 Inst (N=10, Nav2)** | 82.8% | 341.9% | 148.5% | 428.0% | 56.1% | 17.4% | 513 MB |
| **1 Inst (N=5, No-Nav2)** | 27.7% | 180.8% | 69.2% | 0.0% | 18.0% | 13.5% | 232 MB |
| **4 Inst (N=5, No-Nav2)** | 71.5% | 522.6% | 206.7% | 0.0% | 58.0% | 19.9% | 504 MB |

---

## 3. KẾT LUẬN & ĐỀ XUẤT CHO PILOT 300 EPISODES

1. **Đo độ trễ Cold Restart theo định nghĩa chuẩn xác:**
   - Điểm bắt đầu tính từ lúc phát lệnh `gz sim`, kết thúc khi cả 2 điều kiện đạt được: Nav2 lifecycle `ACTIVE` VÀ nhận khung depth đầu tiên. Không có bất kỳ độ trễ giãn cách nhân tạo nào.
   - Ở 1 instance (máy rảnh): Cold restart trung bình là **$4.88 \pm 0.78\text{ s}$** (Min $3.95\text{ s}$, Max $5.98\text{ s}$). Yếu tố quyết định chủ yếu là Nav2 active (~$4.6\text{ s}$).
   - Dưới tải thật 4 instance song song liên tục restart: Cold restart trung bình là **$5.88 \pm 0.80\text{ s}$** (Min $5.29\text{ s}$, Max $8.30\text{ s}$). Dưới tải render 4 camera D435i song song, khung depth đầu tiên là yếu tố tới sau (~$5.8\text{ s}$).
2. **Thông lượng tính lại cho episode 20s:**
   - 1 Instance: $12.0\text{ s}$ wall-time/episode -> **$300.4\text{ ep/h}$** (33.29 giờ cho 10.000 episodes).
   - 4 Instances (tải song song): $23.8\text{ s}$ wall-time/episode -> **$604.2\text{ ep/h}$** (chỉ mất **$16.55\text{ giờ}$** cho 10.000 episodes, hoặc ~30 phút cho Pilot 300 episodes).
3. **Tài nguyên VRAM & GPU:** VRAM duy trì dưới 1GB trên tổng số 8GB của RTX 4060, hoàn toàn an toàn khi sinh dữ liệu dài tập.

