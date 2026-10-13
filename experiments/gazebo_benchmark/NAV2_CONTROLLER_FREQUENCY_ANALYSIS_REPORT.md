# BÁO CÁO CHẨN ĐOÁN VÀ ĐIỀU CHỈNH TẦN SỐ NAV2 MPPI CONTROLLER

**Môi trường thử nghiệm:** Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic 8.x  
**Phần cứng:** AMD Ryzen 7 7735H with Radeon Graphics (8 cores / 16 threads), NVIDIA GeForce RTX 4060 Laptop GPU (8GB VRAM)  
**Phiên bản Nav2:** Binary apt release `ros-jazzy-nav2-mppi-controller` (1.3.13-1noble, Build type: Release `-O3 -DNDEBUG`)  
**Script thực thi:** [`experiments/gazebo_benchmark/benchmark_nav2_controller_frequency.py`](file:///home/cuongbien/DynamuhSim/experiments/gazebo_benchmark/benchmark_nav2_controller_frequency.py)  
**File kết quả JSON thô:** [`experiments/gazebo_benchmark/nav2_controller_frequency_benchmark_results.json`](file:///home/cuongbien/DynamuhSim/experiments/gazebo_benchmark/nav2_controller_frequency_benchmark_results.json)  

---

## 1. ĐO CƠ SỞ (BASELINE MEASUREMENT - 1 INSTANCE)

| Chế độ mô phỏng | Real Time Factor (RTF) | Tần số Wall Clock | Tần số Sim-Time (`cmd_vel`) | Chu kỳ Sim-Time trung bình | CPU Luồng MPPI (`top -H`) | Số cảnh báo trễ nhịp ("missed rate") |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **RTF = 1.0 (Khóa trần chuẩn)** | **1.00** | **20.00 Hz** | **20.06 Hz** | **49.9 ms** | **3.6%** (của 1 core) | **0** |
| **RTF = 0.0 (Gỡ trần uncapped)** | **41.30** | **20.01 Hz** | **0.48 Hz** | **2063.6 ms** | **16.0%** (của 1 core) | **0** |

---

## 2. NGUYÊN NHÂN GỐC RỄ CỦA HIỆN TƯỢNG "4.5 HZ" & BÁC BỎ GIẢ THUYẾT CŨ

### 2.1. Mã nguồn thực tế của `nav2_controller::ControllerServer` (Upstream ROS 2 Jazzy)
Trong file mã nguồn `nav2_controller/src/controller_server.cpp` (hàm `computeControl()`):
```cpp
rclcpp::WallRate loop_rate(controller_frequency_); // <--- ĐIỀU ĐỘ THEO ĐỒNG HỒ THỰC TẾ (WALL CLOCK)!
while (rclcpp::ok()) {
  auto start_time = this->now();                    // Lấy mốc thời gian sim-time
  ...
  computeAndPublishVelocity();                      // MPPI tính toán và publish cmd_vel
  auto cycle_duration = this->now() - start_time;
  if (!loop_rate.sleep()) {                         // <--- Sleep theo std::chrono::steady_clock!
    RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
                controller_frequency_, 1 / cycle_duration.seconds());
    loop_rate.reset();
  }
}
```

### 2.2. Bản chất cơ chế định nhịp
- `ControllerServer` sử dụng `rclcpp::WallRate`, paced theo **thời gian thực tế của hệ điều hành** (Wall Clock, 50.0 ms mỗi chu kỳ).
- Khi chạy uncapped (`real_time_factor = 0.0`), Gazebo chạy nhanh gấp nhiều lần thời gian thực (ví dụ $RTF \approx 4.46$ trong benchmark pilot).
- Trong **1 giây Sim-Time**, chỉ có $1 / RTF \approx 0.224\text{ s}$ Wall-Time trôi qua.
- Bộ điều khiển phát nhịp ở 20 Hz Wall-Time chỉ kịp phát:
  $$\text{Frequency}_{\text{sim}} = \frac{\text{Frequency}_{\text{wall}}}{RTF} = \frac{20.0\text{ Hz}}{4.46} \approx 4.48\text{ Hz}$$
- Do đó, tần số `cmd_vel` theo sim-time đo được 4.3 – 4.5 Hz **hoàn toàn là do tỷ lệ nén thời gian của mô phỏng**, không phải do Nav2 bị nghẽn!

### 2.3. Bác bỏ giải thích cũ ("280 × 30 = 8.400 pose tốn 220 ms")
- Báo cáo cũ lấy nghịch đảo của tần số sim-time ($1 / 4.46\text{ Hz} \approx 0.224\text{ s} = 224\text{ ms}$) và vội vàng kết luận: *"MPPI tính toán 8.400 pose bị quá tải, mất tới 220 ms mỗi chu kỳ"*.
- **Điều này hoàn toàn sai vì:**
  1. MPPI mặc định của Nav2 xử lý $2.000\text{ trajectories} \times 56\text{ timesteps} = 112.000\text{ poses}$ chỉ mất $\sim 15\text{--}25\text{ ms}$ nhờ vector hóa SIMD (xsimd AVX2/FMA) và OpenMP đa luồng trên CPU x86.
  2. Cấu hình của chúng ta chỉ có $280 \times 30 = 8.400\text{ poses}$ (nhỏ hơn 13.3 lần so với mặc định).
  3. Đo trực tiếp bằng `top -H` / `ps -T`: Luồng tính toán MPPI chỉ chiếm **$3.4\% \text{--} 3.6\%$** của 1 lõi CPU ở 20 Hz. Thời gian tính toán thực tế mỗi chu kỳ:
     $$T_{\text{compute}} \approx 50\text{ ms} \times 3.6\% \approx 1.8\text{ ms}!$$
  4. Suốt toàn bộ các bài đo, biến `loop_warnings_count` bằng **0** (không có bất kỳ cảnh báo trễ nhịp nào từ Nav2).
  5. Thử nghiệm ép tải ở **50 Hz** (chu kỳ chỉ 20 ms), controller vẫn hoàn thành xuất sắc ở **$50.02\text{ Hz}$** với CPU luồng chỉ **$3.9\%$**.

---

## 3. KIỂM TRA HỆ THỐNG CẤU HÌNH & BUILD

1. **`visualize` của MPPI:** Đang tắt (`visualize: false`).
2. **`consider_footprint`:**
   - Trong `CostCritic`: `consider_footprint: false`. Nhưng local costmap lại cấu hình `footprint` dạng polygon, gây cảnh báo shape mismatch.
   - Khi chuyển sang bán kính tròn `robot_radius: 0.16`, cảnh báo biến mất, lookup chi phí giảm từ footprint polygon về point lookup.
3. **Kích thước & độ phân giải local costmap:**
   - Gốc: $6 \times 6\text{ m}$, resolution $0.05\text{ m}$ ($120 \times 120 = 14.400$ ô).
   - Đã thử nghiệm thu nhỏ $3 \times 3\text{ m}$ ($60 \times 60 = 3.600$ ô, giảm 75% dữ liệu bộ nhớ).
4. **Số critics, batch_size, time_steps:**
   - Gốc: 8 critics, `batch_size: 280`, `time_steps: 30`.
   - Tinh gọn: 5 critics cốt lõi (`ConstraintCritic`, `CostCritic`, `GoalCritic`, `PathAlignCritic`, `PathFollowCritic`), bỏ 3 critics góc dư thừa.
   - `batch_size: 150`, `time_steps: 30` (giữ tầm nhìn $30 \times 0.05\text{s} = 1.5\text{s}$, giảm không gian mẫu từ 8.400 xuống 4.500 poses).
5. **Nav2 build:** Cài từ apt binary `ros-jazzy-nav2-mppi-controller` (Release build, stripped ELF 64-bit).
6. **CPU luồng controller_server:** Tối đa 3.4% – 5.6% của 1 core, không hề chiếm hết 1 lõi (còn dư trên 94% công suất CPU).

---

## 4. BẢNG ABLATION TỪNG THÔNG SỐ (1 INSTANCE, RTF = 1.0)

| Cấu hình | Thay đổi so với Baseline | Thời gian tính toán ($T_{\text{calc}}$) | Wall Hz | Sim Hz | CPU Luồng MPPI (%) | Cảnh báo Missed Rate |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Baseline** | $280 \times 30$, 8 critics, $6\times 6\text{m}$ costmap | ~1.8 ms | 20.0 Hz | **20.06 Hz** | 3.6% | 0 |
| **Cfg 1** | `robot_radius: 0.16`, tắt footprint polygon | ~1.75 ms | 20.0 Hz | **20.00 Hz** | 3.5% | 0 |
| **Cfg 2** | Local costmap $3 \times 3\text{m}$, res $0.05\text{m}$ | ~1.70 ms | 20.0 Hz | **20.00 Hz** | 3.4% | 0 |
| **Cfg 3** | Tinh gọn còn 5 critics cốt lõi | ~1.70 ms | 20.0 Hz | **20.00 Hz** | 3.4% | 0 |
| **Cfg 4** | `batch_size: 150`, `time_steps: 30` (horizon 1.5s) | ~1.70 ms | 20.0 Hz | **20.00 Hz** | 3.4% | 0 |
| **Cfg 5 (Stress)**| Thử nghiệm ép tần số lên `50.0 Hz` | ~0.80 ms | 50.0 Hz | **50.02 Hz** | 3.9% | 0 |

---

## 5. KẾT QUẢ ĐO 4 INSTANCES SONG SONG (MỤC TIÊU $\ge 10\text{ HZ}$)

### 5.1. Chế độ chuẩn `real_time_factor = 1.0` (Khóa nhịp thời gian thực)
- **Baseline Params (gốc):**
  - Instance 50: **20.22 Hz** (RTF = 0.98, CPU thread = 4.9%)
  - Instance 51: **19.72 Hz** (RTF = 1.01, CPU thread = 5.0%)
  - Instance 52: **19.80 Hz** (RTF = 1.00, CPU thread = 5.0%)
  - Instance 53: **20.19 Hz** (RTF = 1.02, CPU thread = 5.6%)
  - **Trung bình 4 instances:** **$19.98\text{ Hz}$** (Đạt mục tiêu $\ge 10\text{ Hz}$, chạm mức tối đa 20.0 Hz).
- **Optimized Params (Cfg 4):**
  - Instance 50: **19.74 Hz** (RTF = 1.01, CPU thread = 4.9%)
  - Instance 51: **20.02 Hz** (RTF = 1.00, CPU thread = 4.9%)
  - Instance 52: **20.38 Hz** (RTF = 0.98, CPU thread = 4.6%)
  - Instance 53: **20.75 Hz** (RTF = 0.96, CPU thread = 4.8%)
  - **Trung bình 4 instances:** **$20.22\text{ Hz}$** (Đạt mục tiêu $\ge 10\text{ Hz}$).

### 5.2. Chế độ gỡ trần `real_time_factor = 0.0`
- **Baseline Params (controller 20 Hz):** Sim Hz trung bình = **$3.12\text{ Hz}$** (do RTF trung bình đạt 4.37x).
- **50 Hz Controller:** Sim Hz trung bình tăng lên **$7.04\text{ Hz}$** (ở instance RTF = 1.81 đạt tới **$14.23\text{ Hz}$**).

---

## 6. KẾT LUẬN & ĐỀ XUẤT CHÍNH THỨC

1. **Khóa trần thời gian thực trong SDF (`<real_time_factor>1.0</real_time_factor>`):**
   - Khi duy trì `real_time_factor = 1.0`, tần số điều khiển `cmd_vel` trong sim-time **đạt đúng 20.0 Hz** trên toàn bộ 4 instances song song.
2. **Cấu hình tối ưu khuyến nghị cho Pilot / Production (Cfg 4):**
   - Thay footprint polygon bằng `robot_radius: 0.16` và `CostCritic.consider_footprint: false`.
   - Giảm local costmap còn $3.0 \times 3.0\text{ m}$ (tiết kiệm 75% ô chi phí).
   - Tinh gọn 5 critics: `ConstraintCritic`, `CostCritic`, `GoalCritic`, `PathAlignCritic`, `PathFollowCritic`.
   - Sử dụng `batch_size: 150`, `time_steps: 30`, `model_dt: 0.05` (duy trì horizon 1.5s, tiết kiệm 46% trajectory checks).
