# Đặc Tả Kỹ Thuật: Chuẩn Hóa Mô Hình Hóa Con Người & Cơ Chế Đồng Bộ Hóa (Sim-to-Real)

Tài liệu này xác định kiến trúc chuẩn (**Unified Human Pipeline**) cho dự án **DynamuhSim**, giải quyết triệt để sự phân mảnh giữa các thế giới mô phỏng hiện tại, tối ưu hóa triệt để cho cấu hình phần cứng **RTX 4060 (8GB VRAM)**, và chốt kiến trúc 2 tầng (**Two-Tiered Architecture**): **RVO2/ORCA cho Huấn luyện** và **HuNavSim cho Kiểm thử**.

---

## 1. Kiến Trúc Chiến Lược Hai Tầng (Two-Tiered Architecture)

Nhằm vừa đảm bảo khả năng sinh dữ liệu quy mô lớn (hàng chục ngàn episode không bị nghẽn máy), vừa đạt độ tin cậy tuyệt đối khi đánh giá tính lịch sự xã hội (Social Navigation, Proxemics & Politeness), dự án áp dụng mô hình phân tách rõ ràng:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                      KIẾN TRÚC PHÂN TẦNG ĐỘNG CƠ MÔ PHỎNG CON NGƯỜI                    │
├───────────────────────────────────────────┬────────────────────────────────────────────┤
│  TẦNG 1: HUẤN LUYỆN (95% KHỐI LƯỢNG)      │  TẦNG 2: KIỂM THỬ / EVAL (5% KHỐI LƯỢNG)   │
│  "High-Throughput Training Engine"        │  "Out-of-Distribution Benchmark Engine"    │
├───────────────────────────────────────────┼────────────────────────────────────────────┤
│ • Động cơ: C++ RVO2/ORCA tích hợp thẳng   │ • Động cơ: HuNavSim (Social Force Model +  │
│   vào Gazebo Sim 8 (In-Engine Plugin).    │   Behavior Trees) chạy trong Docker Humble.│
│ • Cơ chế: Asymmetric Reciprocity          │ • Kịch bản: school_arena_hunav & demo_1.   │
│   (Robot nhường 80%, Người nhường 20%).   │ • Mục đích: Chạy 50–100 episodes kiểm thử  │
│ • Vùng an toàn: Anisotropic Proxemics     │   mù (Zero-shot) để đo Personal Space      │
│   (Elip cá nhân mở rộng về phía trước).   │   Intrusion (PSI), chứng minh robot không  │
│ • Tiêu thụ VRAM: < 1.5 GB VRAM.           │   bị học vẹt (overfitting) vào RVO2.       │
│ • Tốc độ: Chạy Headless nhanh 5x - 10x.   │ • Tốc độ: Chạy thời gian thực 1x.          │
└───────────────────────────────────────────┴────────────────────────────────────────────┘
```

---

## 2. Phân Bổ Tài Nguyên Tối Ưu Cho Phần Cứng (RTX 4060 8GB VRAM)

RTX 4060 sở hữu 8GB VRAM. Để tránh hiện tượng Out-Of-Memory (OOM) hoặc nghẽn CPU khi vừa mô phỏng vừa nạp mạng nơ-ron huấn luyện:

| Thành phần | Mức tiêu thụ VRAM | Tải CPU | Ghi chú vận hành |
|---|---|---|---|
| **Gazebo Sim 8 (Headless)** + **RVO2 C++ Plugin** | **~1.2 – 1.5 GB** | 15% – 25% (4 cores) | Chạy ngầm không mở GUI Ogre2, tắt đổ bóng động (`<shadows>false</shadows>`). |
| **VRAM Dành Riêng Cho Training (PyTorch / Nav2)** | **~5.5 – 6.5 GB** | 50% – 60% | Đủ bộ nhớ VRAM để nạp Policy Network (PPO / SAC / BC / Diffusion Policy), replay buffer và batch size 64–256. |
| **Khi Chạy Kiểm Thử (HuNavSim Eval Mode)** | **~3.5 – 4.5 GB** | 70% – 85% | Bật container Docker, chạy 50–100 episode tuần tự để ghi log và tính toán báo cáo đánh giá. |

---

## 3. Chuẩn Hóa Mô Hình Thực Thể Con Người (Unified Single Model)

### 3.1. Cắt bỏ mô hình 2 lớp rời rạc
* ❌ **Chính thức phế truất**: Việc tách riêng 1 `<actor>` visual (dùng DAE mesh) và 1 `<model>` collision (hình trụ tàng hình) như trong [`corridor_090.sdf`](src/custom_corridor/worlds/corridor_090.sdf) và [`arena_obstacle.sdf`](src/custom_corridor/worlds/arena_obstacle.sdf).
* ✅ **Chuẩn hóa**: Toàn bộ con người trong mọi thế giới mô phỏng được định nghĩa bằng **một thẻ `<model>` duy nhất**:

```xml
<model name="human_01">
  <static>false</static>
  <pose>x y z roll pitch yaw</pose>
  <link name="base_link">
    <gravity>false</gravity>
    <kinematic>true</kinematic>

    <!-- 1. Tầng hiển thị Camera RGB: Mesh 3D cử động -->
    <visual name="human_visual">
      <geometry>
        <mesh>
          <uri>model://human/meshes/walk.dae</uri>
          <scale>1.0 1.0 1.0</scale>
        </mesh>
      </geometry>
    </visual>

    <!-- 2. Tầng va chạm và phản xạ tia 2D LiDAR -->
    <collision name="human_collision">
      <pose>0 0 0.85 0 0 0</pose>
      <geometry>
        <cylinder>
          <radius>0.25</radius>
          <length>1.70</length>
        </cylinder>
      </geometry>
      <surface>
        <contact>
          <collide_without_contact>false</collide_without_contact>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

### 3.2. Chuẩn hóa hình học & cảm biến Sim-to-Real
1. **Camera RGB-D (RealSense D435i)**:
   * Sử dụng các mesh 3D có hoạt ảnh xương ([`walk.dae`](src/custom_corridor/models/human/meshes/walk.dae), [`stand.dae`](src/custom_corridor/models/human/meshes/stand.dae)).
   * **Domain Randomization**: Ngẫu nhiên hóa tỉ lệ cơ thể ($0.9\text{–}1.1$), màu sắc quần áo và phụ kiện (balo, xe đẩy [`push_trolley.dae`](src/custom_corridor/models/human/meshes/push_trolley.dae)).
   * **Depth Dropout**: Thêm nhiễu ngẫu nhiên vào kênh độ sâu (đặc biệt ở đường viền cơ thể) để mô phỏng sự khuếch tán hồng ngoại trên quần áo thực tế.
2. **2D LiDAR**:
   * Trụ va chạm có bán kính $r = 0.25\text{ m}$ (bao quát cả thân và bước chân).
   * Bổ sung nhiễu Gaussian ($\sigma = 0.02\text{–}0.03\text{ m}$) lên các điểm laser quét trúng người để tránh việc mạng nơ-ron học vẹt hình cung tròn hoàn hảo.

---

## 4. Tầng Động Cơ Huấn Luyện: C++ RVO2 / ORCA Plugin (Training Engine)

Để phục vụ sinh hàng chục ngàn episode với tốc độ cao, thuật toán **RVO2 (Optimal Reciprocal Collision Avoidance)** được nhúng trực tiếp thành một C++ World System Plugin trong [`src/custom_corridor_plugins`](src/custom_corridor_plugins/).

### 4.1. Tích hợp tính Lịch sự (Politeness) & Khoảng cách xã hội (Proxemics)

Mặc định RVO2 chia đều $50\% - 50\%$ trách nhiệm né tránh. Để robot học được phép lịch sự:

1. **Bất đối xứng trách nhiệm (Asymmetric Avoidance)**:
   * Gán trọng số né tránh cho người: $w_{\text{human}} = 0.2$ (người chỉ nhường nhẹ khi cần).
   * Gán trọng số né tránh cho robot: $w_{\text{robot}} = 0.8$ (robot chủ động chịu phần lớn trách nhiệm nhường đường).
2. **Không gian cá nhân Elip (Anisotropic Proxemic Ellipse)**:
   * Vùng an toàn của con người được mô hình hóa bất đối xứng theo góc nhìn:
     * Phía trước mặt: $1.2\text{–}1.5\text{ m}$ (vùng tôn trọng di chuyển).
     * Hai bên hông: $0.6\text{–}0.8\text{ m}$.
     * Phía sau lưng: $0.4\text{–}0.5\text{ m}$.
3. **Độ trễ phản xạ con người (Human Reaction Latency)**:
   * Vận tốc người trong RVO2 không được thay đổi tức thời mỗi bước 250 Hz, mà bị làm mượt qua bộ lọc thông thấp (Low-pass filter) và cập nhật ở tần số $2.5\text{–}4.0\text{ Hz}$, tương đương độ trễ phản xạ $250\text{–}400\text{ ms}$ của con người thật.

```
                       [Không gian cá nhân elip]
                                ▲
                                │ Hướng nhìn (+X)
                          ┌─────┴─────┐
                          │   1.5 m   │  (Vùng tôn trọng phía trước)
                   ───────┼─────●─────┼───────
                     0.8m │   Người   │ 0.8m
                          └─────┬─────┘
                                │ 0.5m
                                ▼
```

---

## 5. Tầng Động Cơ Đánh Giá: HuNavSim Benchmark (Evaluation Engine)

* **Giữ nguyên toàn bộ cấu hình HuNavSim**: Mọi script khởi chạy [`start_hunav_agents.sh`](src/custom_corridor/worlds/school_arena/demo_1/start_hunav_agents.sh), container Docker và world [`school_arena_hunav.sdf`](src/custom_corridor/worlds/school_arena/school_arena_hunav.sdf) được duy trì nguyên vẹn.
* **Quy trình đánh giá mù (Zero-Shot Generalization Test)**:
  1. Sau khi mô hình được huấn luyện hoàn tất từ tập dữ liệu RVO2, đóng băng trọng số (Freeze Policy weights).
  2. Nạp Policy vào môi trường [`school_arena_hunav`](src/custom_corridor/worlds/school_arena/).
  3. Cho robot chạy 50–100 episodes dưới sự điều khiển của HuNavSim.
  4. Trích xuất các chỉ số đo lường xã hội chuẩn từ [`evaluation/social_metrics.py`](evaluation/social_metrics.py):
     * **PSI (Personal Space Intrusion)**: Tỉ lệ thời gian và mức độ robot xâm phạm vùng an toàn của người.
     * **Proactive Yielding Rate**: Khả năng robot chủ động nhường đường tại các nút giao cắt hành lang hẹp.
     * **Navigation Success Rate & Time-to-Goal**.

---

## 6. Chuẩn Hóa Cơ Chế Đồng Bộ Hóa Dữ Liệu (Synchronization Pipeline)

```
           [Gazebo Physics Step (PreUpdate @ 250 Hz, dt = 0.004s)]
                                      │
              ┌───────────────────────┴───────────────────────┐
              ▼                                               ▼
   [Cập nhật vị trí Human]                        [Sensor Rendering (Camera / LiDAR)]
   RVO2 Plugin cập nhật pose link                  Tia quét & Điểm ảnh lấy chính xác
   trong cùng một ECM cycle                        tọa độ mới nhất của Human
              │                                               │
              └───────────────────────┬───────────────────────┘
                                      ▼
                      [Miền thời gian ROS 2 (/clock)]
                      header.stamp = Gazebo Simulation Clock
                                      │
                                      ▼
        [Bộ lọc đồng bộ ApproximateTimeSynchronizer (Tolerance <= 25 ms)]
                                      │
              ┌───────────────────────┼───────────────────────┐
              ▼                       ▼                       ▼
       [2D LiDAR Scan]         [RGB-D Images]         [Robot Odometry]
              │                       │                       │
              └───────────────────────┼───────────────────────┘
                                      ▼
                 [Observation Tuple: S_t @ 10 Hz (Fixed dt = 0.1s)]
                 [Ground-Truth Label: H_t = (x, y, vx, vy, theta, role)]
```

1. **Clock Authority**: Bắt buộc kích hoạt `use_sim_time: true` trên tất cả các node ROS 2. Toàn bộ timestamp đều lấy từ topic `/clock` của Gazebo Sim 8.
2. **ApproximateTimeSynchronizer**: Dữ liệu cảm biến (`/scan`, `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/odom`) được gom thành bộ quan sát hoàn chỉnh bằng bộ lọc đồng bộ với ngưỡng trôi dạt tối đa $\le 25\text{ ms}$.
3. **Tần số huấn luyện khóa cứng (Fixed Control Step)**: Cố định bước thời gian điều khiển và thu thập dữ liệu ở **$10\text{ Hz}$** ($\Delta t = 0.1\text{ s}$).
4. **Đồng bộ nhãn Ground-Truth trực tiếp**: Trạng thái thực tế của toàn bộ con người được serialize trực tiếp cùng frame dữ liệu cảm biến, loại bỏ hoàn toàn các script ghi CSV rời rạc bên ngoài.

---

## 7. Lộ Trình Triển Khai Kỹ Thuật (Action Plan)

| Giai đoạn | Nội dung công việc cụ thể | File / Module phụ trách |
|---|---|---|
| **Phase 1: Model Consolidation** | Chuyển đổi toàn bộ model con người trong các world sang cấu trúc 1 thẻ `<model>` duy nhất (tích hợp DAE mesh visual + cylinder collision). | [`src/custom_corridor/models/human/model.sdf`](src/custom_corridor/models/human/model.sdf), các file `.sdf` trong [`worlds/`](src/custom_corridor/worlds/) |
| **Phase 2: RVO2 C++ Integration** | Nhúng thư viện RVO2 C++ vào [`src/custom_corridor_plugins`](src/custom_corridor_plugins/), lập trình cơ chế Asymmetric Reciprocity (Politeness) và Anisotropic Proxemics chạy ở nhịp `PreUpdate()`. | [`src/custom_corridor_plugins/src/`](src/custom_corridor_plugins/src/), [`CMakeLists.txt`](src/custom_corridor_plugins/CMakeLists.txt) |
| **Phase 3: Production Pipeline** | Cập nhật script sinh dataset chạy headless với động cơ RVO2 mới ở tần số 10 Hz, tối ưu mức ăn VRAM $< 1.5\text{ GB}$ trên RTX 4060. | [`produce_dataset_v1.py`](src/custom_corridor/worlds/school_arena/demo_1/produce_dataset_v1.py), [`dataset_runner/`](src/custom_corridor/worlds/school_arena/demo_1/dataset_runner/) |
| **Phase 4: Evaluation Benchmark** | Đóng gói script kiểm thử tự động nạp Policy đã train vào [`school_arena_hunav`](src/custom_corridor/worlds/school_arena/school_arena_hunav.sdf) để chạy 50 episodes và tự động xuất báo cáo đánh giá PSI. | [`evaluation/social_metrics.py`](evaluation/social_metrics.py), [`evaluation/reporter.py`](evaluation/reporter.py) |
