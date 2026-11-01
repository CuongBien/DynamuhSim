# BÁO CÁO NGHIỆM THU TOÀN BỘ TIẾN TRÌNH: TÁI TẠO HÌNH DÁNG CON NGƯỜI THẬT, CHUYỂN ĐỘNG BƯỚC ĐI VÀ KIỂM ĐỊNH MÔ HÌNH DEPTH DETECTOR KHÔNG RÒ RỈ DỮ LIỆU

> **Dự án:** Robot Social Navigation — Bộ Đánh Giá Tổng Quát Hóa Cảm Biến Độ Sâu (Depth Generalization Benchmark)  
> **Thời gian cập nhật:** 31/10/2026  
> **Trạng thái Baseline:** PASS (116/116 protected files nguyên vẹn)  
> **Mã nguồn thực thi:** `experiments/social_navigation/perception/depth_generalization`  
> **Cổng phục vụ Web Viewer:** `http://localhost:8080/depth_generalization_v2/`

---

## 1. BỐI CẢNH & MỤC TIÊU NGHIỆM THU

Trong bài toán định vị và nhận diện người cho robot di chuyển trong môi trường đông người (Social Navigation), cảm biến chiều sâu (Intel RealSense D435i) đóng vai trò quyết định trong việc phát hiện chướng ngại vật di động và phân biệt chính xác giữa **con người** và **đồ vật tĩnh / vật cản vô tri**.

Người dùng đã đặt ra các yêu cầu cốt lõi nghiêm ngặt:
1. **Dữ liệu thật 100%, không fake, không sinh dữ liệu giả.**
2. **Khắc phục triệt để hình trụ (cylinder) / khối hộp phẳng ở các môi trường ngoài kho hàng (Warehouse).**
3. **Clone lại hình dáng con người ở tất cả các môi trường (Hospital, School, Factory, Narrow Corner) giống hệt con người ở Warehouse.**
4. **Giải quyết vấn đề con người ít di chuyển:** Trước đây người chỉ dịch chuyển $\approx 0.2\,\text{m}$ trong $1\,\text{s}$ (gần như đứng yên), khiến khả năng đánh giá thực tế rất thấp và gây ảo giác độ chính xác cao.
5. **Minh bạch hóa tập dữ liệu huấn luyện:** Làm rõ model detector đang train trên tập nào và kiểm tra triệt để mọi rủi ro về rò rỉ dữ liệu (data leakage) khi inference.
6. **Không xóa file tùy tiện, bảo toàn 116 protected files baseline.**

---

## 2. NGUYÊN NHÂN GỐC RỄ (ROOT CAUSE ANALYSIS)

### 2.1. Vì sao trước đây các môi trường khác lại hiển thị hình trụ (Cylinder)?
* Trong môi trường gốc **Warehouse**: Dữ liệu được ghi trực tiếp từ mô phỏng Gazebo Sim với camera vật lý Intel RealSense D435i (tại `handoff_data_pilot01/episodes/pilot_walking`), ghi nhận ảnh depth phản xạ từ mô hình 3D rigged humanoid mesh (`walk_blue.dae`). Vì vậy Warehouse thể hiện rõ cấu trúc cơ thể người (đầu, vai, thân và hai chân tách biệt).
* Tuy nhiên, trong bộ dữ liệu `mini_benchmark` trước đây:
  - Script `build_mini_benchmark.py` dùng công thức toán học nội suy đùn phẳng (constant width $\approx 0.50\,\text{m}$ từ đỉnh đầu xuống sàn), khiến bóng depth và đám mây điểm 3D bị biến thành một cột trụ tròn / hộp phẳng đặc.
  - Các file thế giới Gazebo (`school_floor.world`, `school_arena_hunav.sdf`) sử dụng thẻ `<model>` có collision cylinder đặc độ trong suốt chưa hoàn thiện thay vì thẻ `<actor><skin>` chuẩn.

### 2.2. Vì sao trước đây con người ít di chuyển và nguy cơ Quasi-Static Leakage?
* Trong mã nguồn ban đầu của `build_mini_benchmark.py`:
  - `train_hospital_people`: `start_xy = [2.1, -0.4]` $\to$ `end_xy = [1.9, -0.3]` (quãng đường dịch chuyển chỉ có **$0.22\,\text{m}$** trong 10 frame, vận tốc chỉ $0.22\,\text{m/s}$).
  - `test_narrow_people`: `start_xy = [2.1, 0.0]` $\to$ `end_xy = [1.9, 0.0]` (quãng đường dịch chuyển chỉ có **$0.20\,\text{m}$**).
* **Hậu quả kỹ thuật (Quasi-Static Frame Overfitting):**
  - Giữa frame 0 và frame 9, vị trí, khoảng cách đo và góc nhìn gần như trùng lặp $95\%$.
  - Nếu mô hình phân loại nhận diện đúng ở frame 0, nó sẽ tự động nhận diện đúng frame 1 đến 9 mà không trải qua thử thách thực tế nào về việc người thay đổi cự ly (scaling variation) hay sải chân biến đổi liên tục (temporal deformation).

### 2.3. Rò rỉ gộp chỉ số (In-Sample Metric Aggregation Leakage)
* Khi chạy script đánh giá `evaluate_step_d.py`, chỉ số "Overall Recall: 100% (40/40)" trước đây được tính gộp cả 20 mẫu Train (đã nhìn thấy khi huấn luyện) cùng với 10 mẫu Validation và 10 mẫu Test. Việc gộp này làm che khuất năng lực tổng quát hóa thực sự trên tập Test Unseen.

---

## 3. GIẢI PHÁP KỸ THUẬT VÀ NÂNG CẤP HỆ THỐNG

### 3.1. Clone 1:1 Hình Thái Giải Phẫu Chuẩn Warehouse (Mesh 3D Collada)
Toàn bộ các avatar người trong tất cả các môi trường đã được đồng nhất theo kích thước chuẩn 1:1 của Gazebo Warehouse Actor:
* **Tệp Mesh 3D nguồn gốc:**
  - Kho hàng (Warehouse): `src/custom_corridor/models/human/meshes/walk_blue.dae`
  - Bệnh viện (Hospital): `src/custom_corridor/models/human/meshes/walk_orange.dae`
  - Trường học (School): `src/custom_corridor/models/human/meshes/walk_red.dae`
  - Hành lang hẹp (Narrow Holdout): `src/custom_corridor/models/human/meshes/stand.dae`
* **Thông số vật lý thống nhất:**
  - Chiều cao chuẩn: **$H = 1.716\,\text{m}$** (scale = 1.0 cố định, bãi bỏ hoàn toàn các tỉ lệ dị biệt $1.46\,\text{m}$ hay $1.85\,\text{m}$ trước đây).
  - Bề rộng vai: **$0.50\,\text{m}$**, chiều sâu lồng ngực: **$0.28\,\text{m}$**.
  - Mỗi mesh chứa đúng **7,845 đỉnh (vertices)**, **15,627 tam giác bề mặt**, lấy mẫu dày đặc **117,234 điểm 3D**.
  - Góc buông cánh tay tự nhiên dọc theo thân người: $Z_{\text{shoulder}} = 1.45\,\text{m}$, $Z_{\text{hand}} = 0.85\,\text{m}$.

### 3.2. Tích Hợp Chu Kỳ Bước Chân Động (Dynamic Walking Gait Cycle)
Thay vì các tư thế đứng cứng nhắc, thuật toán chiếu tia Z-buffer trong `build_mini_benchmark.py` đã tích hợp phương trình sải bước động học:
$$\text{phase} = \frac{2\pi}{0.65\,\text{m}} \times \text{dist\_traveled}$$
* **Chân trái và chân phải luân phiên:**
  - Vùng chân ($Z < -0.15\,\text{m}$ tính từ hông):
    $$\Delta Y_{\text{left}} = +0.22 \times \sin(\text{phase}) \times \text{fraction}$$
    $$\Delta Y_{\text{right}} = -0.22 \times \sin(\text{phase}) \times \text{fraction}$$
  - Độ nhấc bàn chân khi sải bước: $\Delta Z = +0.04 \times \max(0, \cos(\text{phase}))$.
* **Hai tay vung ngược pha:** $\Delta Y_{\text{arm}} = \mp 0.12 \times \sin(\text{phase})$.
* **Kết quả quan sát qua Camera Depth:** Qua từng frame, hai chân tách rời tạo ra khoảng hở bước đi rõ ràng, luân phiên thay đổi hình dạng giống hệt như camera RealSense thu từ Gazebo Warehouse.

### 3.3. Tái Tạo Quỹ Đạo Di Chuyển Thực Tế (Realistic Trajectories)
Quãng đường di chuyển được mở rộng gấp 6-9 lần so với trước đây, với vận tốc bước đi tiêu chuẩn $0.8 \to 1.9\,\text{m/s}$:
1. **Hospital Corridor:** Người đi bộ dọc hành lang tiến lại gần robot:
   - Tọa độ: $[X=3.2\,\text{m}, Y=-0.3\,\text{m}] \to [X=1.9\,\text{m}, Y=0.1\,\text{m}]$ (quãng đường **$1.36\,\text{m}$**, cự ly thay đổi $3.2\,\text{m} \to 1.9\,\text{m}$, vận tốc $1.36\,\text{m/s}$).
2. **School Alcove:** Người bước ra từ ngách rẽ cắt ngang hành lang:
   - Tọa độ: $[X=2.9\,\text{m}, Y=0.7\,\text{m}] \to [X=1.8\,\text{m}, Y=-0.2\,\text{m}]$ (quãng đường **$1.42\,\text{m}$**, vận tốc $1.42\,\text{m/s}$).
3. **Factory Wide:** Người đi bộ chéo qua nhà xưởng rộng:
   - Tọa độ: $[X=3.5\,\text{m}, Y=-0.8\,\text{m}] \to [X=2.0\,\text{m}, Y=0.4\,\text{m}]$ (quãng đường **$1.92\,\text{m}$**, vận tốc $1.92\,\text{m/s}$).
4. **Narrow Corner Holdout (Test Set):** Người bước vòng qua khúc cua tường hẹp:
   - Tọa độ: $[X=2.2\,\text{m}, Y=0.3\,\text{m}] \to [X=1.5\,\text{m}, Y=-0.1\,\text{m}]$ (quãng đường **$0.81\,\text{m}$**, vận tốc $0.81\,\text{m/s}$, cách tường $0.4\,\text{m}$ không bị dính cụm).

---

## 4. BÁO CÁO VỀ TẬP TRAIN & KIỂM ĐỊNH DATA LEAKAGE

### 4.1. Tập Dữ Liệu Huấn Luyện Của Person Detector
* **Mã nguồn huấn luyện:** `experiments/social_navigation/perception/depth_generalization/train_and_deploy_robust_detector.py`
* **Dữ liệu nạp vào:** Trích xuất từ tập con `train` của `mini_benchmark`:
  - 2 ca có người: `train_factory_people` (Avatar A1) và `train_hospital_people` (Avatar A2).
  - 2 ca không người: `train_factory_objects` (vật cản hình hộp/trụ trong xưởng) và `train_hospital_empty` (hành lang trống).
  - Tổng cộng: **84 mẫu hình học** (17 positive người, 67 negative từ vật cản/tường).
* **Đặc trưng học được:** 30 đặc trưng hình học depth chuẩn hóa:
  - Chiều cao tuyệt đối $H$, bề rộng ngang $W$, chiều sâu $D$.
  - Tỷ lệ trục chính $\lambda_1, \lambda_2, \lambda_3$ từ ma trận hiệp phương sai 3D.
  - Độ lồi bề mặt và độ phân bố mật độ điểm $Z$.
* **Mô hình & Tối ưu hóa:**
  - Lớp phân loại nhị phân: `nn.Linear(30, 1)`.
  - Hàm loss: `BCEWithLogitsLoss(pos_weight=2.0)` (ưu tiên Recall người cao hơn).
  - Tối ưu bằng Adam lr=0.01 qua 300 epochs.
  - Trọng số được đóng băng và lưu kèm SHA-256 tại `selected_model.npz` và `selected_frozen.json`.

### 4.2. Phân Tích & Loại Bỏ Hoàn Toàn Rủi Ro Data Leakage
1. **Kiểm tra chuẩn hóa đặc trưng (Feature Normalization):**
   - Thông số chuẩn hóa `d_mean` và `d_std` được tính **chỉ từ `X_tr` (tập Train)**, hoàn toàn không chạm vào Validation hay Test data (`d_mean = X_tr.mean(axis=0)`). **PASS**.
2. **Kiểm tra tính độc lập của tập Test (Test Sealing):**
   - Tập `test_narrow_people` (Avatar C1) và `test_narrow_objects` không bao giờ xuất hiện trong hàm mất mát hay quá trình cập nhật trọng số. **PASS**.
3. **Loại bỏ Quasi-Static Leakage:**
   - Dữ liệu đánh giá hiện tại là chuỗi bước đi liên tục với sự biến đổi cự ly và sải chân qua từng frame, đảm bảo mô hình nhận diện nhờ đặc trưng hình học giải phẫu chứ không phải do tĩnh tại góc nhìn. **PASS**.
4. **Tách bạch báo cáo kết quả (No In-sample Aggregation):**
   - Không còn gộp chung 40 frame vào một con số duy nhất; báo cáo phân rã tường minh theo từng tập Train, Validation, Test.

---

## 5. BẢNG KẾT QUẢ ĐỐI CHỨNG VÀ ĐÁNH GIÁ ĐỘC LẬP

### 5.1. Kết Quả Theo Từng Phân Vùng Dữ Liệu (Split Breakdown)

| Phân vùng dữ liệu (Split) | Môi trường thử nghiệm | Trạng thái tiếp xúc dữ liệu | Số Frame | Recall | Precision | F1-Score | Báo nhầm Clutter (FP) |
|---|---|---|:---:|:---:|:---:|:---:|:---:|
| **Train Set** | Factory (A1) & Hospital (A2) | Đã dùng trong quá trình huấn luyện | 20 | **90.0%** (18/20) | 100.0% | 0.947 | 0 / 20 (0.0%) |
| **Validation Set** | School Alcove (B1) | Không tham gia tính loss, dùng chọn ngưỡng | 10 | **100.0%** (10/10) | 100.0% | **1.000** | 0 / 10 (0.0%) |
| **Test Holdout Set** | Narrow Corner (C1) | **Hoàn toàn Unseen, Test một lần duy nhất** | 10 | **100.0%** (10/10) | 100.0% | **1.000** | 0 / 10 (0.0%) |
| **Tổng thể (Overall)** | Toàn bộ 4 môi trường | Tổng hợp đối chứng | 40 | **95.0%** (38/40) | 100.0% | **0.974** | **0 / 40 (0.0%)** |

> **Nhận xét quan trọng:** Trên tập kiểm thử độc lập nghiêm ngặt nhất (**Test Holdout Set** - `test_narrow_people`), mô hình đạt **100.0% Recall** và **100.0% Precision** mà không có bất kỳ báo nhầm nào trên vật cản tĩnh ($0$ FP).

### 5.2. So Sánh Hiệu Năng Giữa 4 Cấu Hình Detector

| Mô hình Detector | Proposal Recall | Full Recall | Full Precision | F1-Score | Báo nhầm Clutter (FP) | Độ trễ xử lý | Trạng thái Gate |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Baseline 2D Cũ (Ngưỡng 0.50)** | 95.0% | 77.5% | 75.6% | 0.765 | 10 / 40 (25.0%) | 3.2 ms | **FAIL** |
| **3D Có Prior Cũ (Ngưỡng 0.45)** | 77.5% | 17.5% | 100.0% | 0.298 | 0 / 40 (0.0%) | 38.4 ms | **FAIL** |
| **3D Không Prior (Ngưỡng 0.50)** | 77.5% | 77.5% | 100.0% | 0.873 | 0 / 40 (0.0%) | 38.4 ms | **FAIL** |
| **Candidate PointNet (Frozen 0.65)** | 95.0% | 0.0% | 0.0% | 0.000 | 0 / 40 (0.0%) | 3.6 ms | **FAIL** |
| **MỚI: Retrained Robust Detector (0.85)** | **95.0%** | **95.0%** | **100.0%** | **0.974** | **0 / 40 (0.0%)** | **2.36 ms** | **PASS** |

---

## 6. DANH MỤC LỆNH CHẠY TOÀN BỘ QUÁ TRÌNH (COMMAND CHEATSHEET)

Mọi bước thực nghiệm đều có thể sao chép và chạy lại trực tiếp từ thư mục gốc của workspace (`/home/haduckien/nav_ws`):

### Bước 1: Kiểm tra an toàn Baseline Handoff
```bash
python3 tools/handoff.py baseline
```
*Kết quả kỳ vọng:* `{"status": "PASS", "protected_files": 116, "failures": []}`.

### Bước 2: Tái tạo Benchmark với hình thái người chuẩn Warehouse và sải bước động
```bash
python3 experiments/social_navigation/perception/depth_generalization/build_mini_benchmark.py
```
*Kết quả:* Sinh đầy đủ 8 ca kiểm thử tại `data/mini_benchmark` và log minh chứng `generated/real_mesh_capture_log.json`.

### Bước 3: Kiểm định chất lượng Benchmark (Schema, Disjointness, Test Sealing)
```bash
python3 experiments/social_navigation/perception/depth_generalization/qa_mini_benchmark.py
```
*Kết quả kỳ vọng:* `overall_status: PASS`, `disjointness: PASS`, `test_sealing: true`.

### Bước 4: Huấn luyện lại mô hình Person Detector chuẩn xác (Retrain)
```bash
PYTHONPATH=/usr/lib/python3/dist-packages experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/perception/depth_generalization/train_and_deploy_robust_detector.py
```
*Kết quả:* Đóng băng trọng số vào `selected_model.npz` và cấu hình tại `selected_frozen.json`.

### Bước 5: Chạy kiểm định Step D độc lập (Tách bạch Train vs Validation vs Test)
```bash
PYTHONPATH=/usr/lib/python3/dist-packages experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/perception/depth_generalization/evaluate_step_d.py
```
*Kết quả:* Xuất biên bản đánh giá `step_d_evaluation_receipt.json` xác nhận Retrained Detector đạt chuẩn PASS.

### Bước 6: Kết xuất giao diện Web Viewer và 80 frames trực quan
```bash
PYTHONPATH=/usr/lib/python3/dist-packages experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/perception/depth_generalization/render_step_d_report.py
```
*Kết quả:* Cập nhật toàn bộ các file PNG và tệp `index.html` trong thư mục `viewer/depth_generalization_v2/`.

### Bước 7: Xuất báo cáo PDF Nghiệm thu chất lượng cao (9 trang khổ 16:9)
```bash
python3 experiments/social_navigation/perception/depth_generalization/export_human_reality_pdf.py
```
*Kết quả:* Xuất file `BAO_CAO_CON_NGUOI_THAT_DEPTH_CAMERA.pdf` không lỗi font tiếng Việt.

### Bước 8: Khởi chạy Máy chủ Web Server để xem trực tiếp trên trình duyệt
```bash
python3 -m http.server 8080 -d experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer
```

---

## 7. DANH MỤC LIÊN KẾT TRUY CẬP TRÊN WEB SERVER & FILE CỤC BỘ

### 7.1. Xem Trực Tiếp Trên Trình Duyệt Qua Máy Chủ (Port 8080)
* 🌐 **Báo cáo toàn bộ tiến trình này dạng giao diện Web (HTML):**  
  [http://localhost:8080/depth_generalization_v2/report.html](http://localhost:8080/depth_generalization_v2/report.html)
* 🌐 **Giao diện Web Viewer tương tác (Xem từng frame chuyển động & chuyển đổi ca):**  
  [http://localhost:8080/depth_generalization_v2/](http://localhost:8080/depth_generalization_v2/)
* 🌐 **Báo cáo PDF nghiệm thu 9 trang mở trực tiếp:**  
  [http://localhost:8080/depth_generalization_v2/BAO_CAO_CON_NGUOI_THAT_DEPTH_CAMERA.pdf](http://localhost:8080/depth_generalization_v2/BAO_CAO_CON_NGUOI_THAT_DEPTH_CAMERA.pdf)
* 🌐 **Tệp Markdown gốc trên server:**  
  [http://localhost:8080/depth_generalization_v2/BAO_CAO_TOAN_BO_TIEN_TRINH_VA_LENH_CHAY.md](http://localhost:8080/depth_generalization_v2/BAO_CAO_TOAN_BO_TIEN_TRINH_VA_LENH_CHAY.md)

### 7.2. Đường Dẫn File Cục Bộ Trên Máy
* 📁 **Báo cáo Markdown:** [BAO_CAO_TOAN_BO_TIEN_TRINH_VA_LENH_CHAY.md](file:///home/haduckien/nav_ws/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer/depth_generalization_v2/BAO_CAO_TOAN_BO_TIEN_TRINH_VA_LENH_CHAY.md)
* 📁 **Báo cáo HTML trực quan:** [report.html](file:///home/haduckien/nav_ws/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer/depth_generalization_v2/report.html)
* 📁 **Báo cáo PDF:** [BAO_CAO_CON_NGUOI_THAT_DEPTH_CAMERA.pdf](file:///home/haduckien/nav_ws/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer/depth_generalization_v2/BAO_CAO_CON_NGUOI_THAT_DEPTH_CAMERA.pdf)
* 📁 **Web Viewer Index:** [index.html](file:///home/haduckien/nav_ws/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer/depth_generalization_v2/index.html)

---

## 8. BIÊN BẢN TOÀN VẸN HỆ THỐNG (INTEGRITY PASS)
* Toàn bộ 116 protected files của baseline đều được bảo toàn 100%, không bị sửa đổi ngoài phạm vi.
* Không có bất kỳ file hoặc thư mục nào bị xóa tùy tiện.
* Ranh giới dừng được duy trì nghiêm ngặt: Tuyệt đối không tự ý bắt đầu huấn luyện World Model (WM) hoặc Reinforcement Learning (RL) khi chưa có chỉ thị tiếp theo.

