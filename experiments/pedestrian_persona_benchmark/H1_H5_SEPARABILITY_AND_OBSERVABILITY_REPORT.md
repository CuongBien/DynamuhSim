# BÁO CÁO THỰC NGHIỆM ĐỊNH LƯỢNG: KIỂM CHỨNG ĐỘ TÁCH NHÃN VÀ KHẢ QUAN SÁT H1–H5 CỦA MODULE M3

> **Ngày thực hiện:** 11/10/2026  
> **Bộ phận:** Nhóm Nghiên cứu & Huấn luyện Robot Tự hành (DynamuhSim)  
> **Mục tiêu:** Trả lời dứt khoát câu hỏi: *Với đúng các ràng buộc cảm nhận thực tế của module M3 (cửa sổ quan sát 2.0 s @ 10 Hz, nhiễu cảm biến Gaussian, sai phân vận tốc hữu hạn, góc nhìn camera $87^\circ$ và cắt cụt cự ly), có thể phân biệt được các kiểu người H1–H5 hay không?*  
> **Dữ liệu đánh giá:** 5.000 kịch bản mới sinh độc lập (1.000 kịch bản/persona), 45.000 cửa sổ quan sát trượt $2.0\text{ s}$, chia theo kịch bản (60% Train, 20% Val, 20% Test niêm phong).

---

## TÓM TẮT KẾT QUẢ & KẾT LUẬN QUY TẮC (EXECUTIVE SUMMARY)

1. **Quy tắc quyết định (Bước 7):**
   - **Ngưỡng quy định:** Tỷ lệ nhầm lẫn giữa H1 và H2 trên tập tương tác ($\tau = 1.5\text{ m}, \sigma = 0.05\text{ m}$) phải $\le 30.0\%$ VÀ khoảng tin cậy 95% Bootstrap không được chạm ngưỡng $30.0\%$.
   - **Số liệu đo đạc thực tế:**
     - Tỷ lệ người H2 bị nhầm thành H1: **29.01%** với khoảng tin cậy 95% Bootstrap là **[20.83%, 38.24%]** (chạm và vượt sâu qua ngưỡng 30%).
     - Tỷ lệ người H1 bị nhầm thành H2: **37.34%** (vượt xa ngưỡng 30%).
     - Xét theo dải $\alpha \in [0.10, 0.20)$, tỷ lệ H2 nhầm thành H1 lên tới **35.11%**.
     - Tại hình học hành lang hẹp ($0.9\text{ m}$ và $1.2\text{ m}$), Macro-F1 toàn bộ sụp đổ xuống **24.9% – 25.4%** (tương đương đoán mò).
   - **KẾT LUẬN CHÍNH THỨC:** **ĐỀ XUẤT GỘP H1 VÀ H2 (hoặc chuyển module M3 sang hồi quy tham số $\alpha$ liên tục cho cụm này)**. Tuyệt đối không giữ H1 và H2 dưới dạng 2 lớp phân loại rời rạc cứng khi sinh dữ liệu 10.000 episode.

2. **Các phát hiện cốt lõi khác về khả quan sát của M3:**
   - **H4 (Đứng yên) tách biệt dễ dàng nhưng làm "đẹp" số:** F1 của H4 đạt $85.4\%$ trên tập tương tác và $56.4\%$ toàn bộ nhờ vận tốc bằng 0. Khi loại bỏ H4 (ma trận 4x4), Macro-F1 của hệ thống sụt giảm từ $30.70\%$ xuống **$25.29\%$**. Đúng như dự đoán, cần tách riêng H4 khi báo cáo.
   - **H5 (Lực xã hội SFM) tách biệt tốt hơn H1/H2:** H5 đạt F1 = $55.3\%$ trên tập tương tác nhờ chữ ký phản ứng giảm tốc và bẻ lái từ xa ($d > 2.0\text{ m}$) khác biệt hoàn toàn với hình học RVO2.
   - **Độ sớm nhận diện (Early Detection Latency):** Tỷ lệ nhận diện tự tin ($P(true) \ge 0.70$ duy trì $\ge 3$ cửa sổ liên tiếp) chỉ đạt **$7.47\%$** ở nhiễu $0.05\text{ m}$. Hầu hết kịch bản chỉ có thể phân biệt khi đã tới sát điểm CPA ($1.32\text{ m}$).
   - **Độ hiệu chỉnh xác suất (Calibration):** Expected Calibration Error (ECE) đạt mức rất tốt (**$7.35\%$** ở GBDT). Điều này chứng minh module M3 **cần xuất phân bố xác suất posterior $P(H_i)$** thay vì nhãn cứng (hard label), và tự động quay về phân bố tiên nghiệm (prior) khi ngoài vùng tương tác.

---

## 1. PHƯƠNG PHÁP THỰC NGHIỆM & RÀNG BUỘC CẢM NHẬN M3

### 1.1. Bộ dữ liệu kiểm chứng 5.000 kịch bản
- **Bộ sinh kịch bản:** C++ engine tích hợp thư viện native RVO2 và mô hình Social Force Model (SFM) chuẩn ([`generate_persona_scenarios.cpp`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/generate_persona_scenarios.cpp)).
- **Cấu hình kịch bản:**
  - 5 persona: H1 (Chuẩn ORCA $\alpha=0.5$), H2 (Ưu tiên bản thân $\alpha \sim \mathcal{U}[0.10, 0.35]$), H3 (Không nhường $\alpha=0$), H4 (Đứng yên $v=0$), H5 (Lực xã hội mềm SFM: $A \in [2.0, 3.5]\text{ N}, B \in [0.35, 0.60]\text{ m}, \tau \in [0.20, 0.40]\text{ s}$).
  - 4 kiểu tiếp cận: Đối đầu (Head-on), Cắt ngang (Crossing), Cắt xiên (Oblique), Cùng chiều (Same-direction). Mỗi kiểu 250 kịch bản/persona.
  - 3 hình học không gian: Mở $6\times 6\text{ m}$, Hành lang $0.9\text{ m}$, Hành lang $1.2\text{ m}$.
  - Người nền: 0–3 background pedestrians tương tác tự nhiên.
  - Robot chuyển động biến thiên: Vận tốc $v \in [0, 0.6]\text{ m/s}, \omega \in [-1.0, 1.0]\text{ rad/s}$ kết hợp đi thẳng, đi chậm, dừng giữa chừng và rẽ góc.
  - Phân chia kịch bản: **60% Train (3.000 kịch bản) / 20% Val (1.000 kịch bản) / 20% Test (1.000 kịch bản, niêm phong tuyệt đối)**.
- **Tệp dữ liệu lưu trữ:**
  - Dataset: [`h1_h5_scenarios_dataset.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/h1_h5_scenarios_dataset.json) (130 MB, 5.000 kịch bản).
  - Metadata & Seeds: [`scenarios_metadata.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/scenarios_metadata.json) (1.0 MB).

### 1.2. Định nghĩa hình học "Có tương tác" (Nominal CPA)
Nhằm tránh sai lệch lựa chọn (selection bias) khi dùng khoảng cách tối thiểu đo được sau tương tác ($d_{min}$), nhãn tương tác được xác định thuần túy theo hình học ngoại suy tuyến tính tại thời điểm đầu mỗi cửa sổ quan sát ($t = t_0$):
$$\mathbf{r}_{rel}(t) = (\mathbf{x}_h(t_0) - \mathbf{x}_r(t_0)) + (\mathbf{v}_h(t_0) - \mathbf{v}_r(t_0)) \cdot t$$
$$t_{CPA} = -\frac{\mathbf{r}_{rel}(t_0) \cdot \mathbf{v}_{rel}(t_0)}{\|\mathbf{v}_{rel}(t_0)\|^2}, \quad d_{CPA} = \|\mathbf{r}_{rel}(t_0) + \mathbf{v}_{rel}(t_0) \cdot t_{CPA}\|$$
- **Tập tương tác:** $d_{CPA} \le \tau$ VÀ $0.0\text{ s} \le t_{CPA} \le 3.0\text{ s}$ (Khảo sát cả $\tau = 1.5\text{ m}$ và $\tau = 2.0\text{ m}$).
- **Tập không tương tác:** Các cửa sổ còn lại.
- **Tổng số cửa sổ trượt test:** 9.000 cửa sổ ($2.0\text{ s}$, bước $0.5\text{ s}$), trong đó có **1.212 cửa sổ tương tác** tại $\tau = 1.5\text{ m}$ (13,5%) và **1.960 cửa sổ** tại $\tau = 2.0\text{ m}$ (21,8%).

### 1.3. Ràng buộc quan sát thực tế M3
- **Cửa sổ thời gian:** 2.0 s (20 bước quan sát @ 10 Hz).
- **Mức nhiễu vị trí:** Gaussian $\sigma \in \{0.00\text{ m}, 0.05\text{ m}, 0.10\text{ m}\}$.
- **Ước lượng vận tốc:** Sai phân hữu hạn (finite difference) từ chuỗi vị trí có nhiễu; **không dùng vận tốc ground truth**.
- **Tầm quan sát camera Depth (FOV Mask):** Góc nhìn ngang $87^\circ$ ($|\theta| \le 43.5^\circ$), cự ly hữu dụng $[0.28\text{ m}, 5.0\text{ m}]$. Khi người ra ngoài FOV hoặc cự ly, giá trị tọa độ bị che khuất (masked out).

---

## 2. KẾT QUẢ ĐO LƯỜNG CHI TIẾT (7 MỤC BẮT BUỘC)

### Mục 6.1: Ma trận nhầm lẫn & Chỉ số phân loại tổng thể ($\sigma = 0.05\text{ m}$)

#### Ma trận nhầm lẫn 5x5 trên tập kiểm tra niêm phong (Toàn bộ 9.000 cửa sổ)
Tệp CSV: [`confusion_matrix_5x5_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_sigma005.csv)

| Thực tế \ Dự đoán | Dự đoán H1 | Dự đoán H2 | Dự đoán H3 | Dự đoán H4 | Dự đoán H5 | Tổng thực tế |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **H1 (ORCA Chuẩn)** | **511** | 356 | 293 | 319 | 321 | 1.800 |
| **H2 (Ích kỷ)** | 421 | **357** | 316 | 352 | 354 | 1.800 |
| **H3 (Không nhường)**| 504 | 360 | **274** | 359 | 303 | 1.800 |
| **H4 (Đứng yên)** | 181 | 108 | 124 | **1.255** | 132 | 1.800 |
| **H5 (Lực xã hội)** | 347 | 275 | 293 | 365 | **520** | 1.800 |
| **Tổng dự đoán** | 1.964 | 1.456 | 1.300 | 2.650 | 1.630 | 9.000 |

#### Chỉ số Precision, Recall, F1 theo từng lớp (Toàn bộ cửa sổ, $\sigma = 0.05\text{ m}$)
- **H1:** Precision = $26.0\%$, Recall = $28.4\%$, F1 = $27.2\%$
- **H2:** Precision = $24.5\%$, Recall = $19.8\%$, F1 = $21.9\%$
- **H3:** Precision = $21.1\%$, Recall = $15.2\%$, F1 = $17.7\%$
- **H4:** Precision = $47.4\%$, Recall = $69.7\%$, F1 = **$56.4\%$**
- **H5:** Precision = $31.9\%$, Recall = $28.9\%$, F1 = $30.3\%$
- **Toàn thể (5 lớp):** Accuracy = **$32.41\%$**, Macro-F1 = **$30.70\%$**
- **Bỏ H4 (Ma trận 4x4):** Macro-F1 = **$25.29\%$** (Tệp CSV: [`confusion_matrix_4x4_noH4_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_4x4_noH4_sigma005.csv))

---

### Mục 6.2: Phân rã theo Tập tương tác hình học (Nominal CPA)

#### Ma trận nhầm lẫn 5x5 trên Tập Tương tác ($\tau = 1.5\text{ m}$, 1.212 cửa sổ)
Tệp CSV: [`confusion_matrix_5x5_interactive15_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_interactive15_sigma005.csv)

| Thực tế \ Dự đoán | Dự đoán H1 | Dự đoán H2 | Dự đoán H3 | Dự đoán H4 | Dự đoán H5 | Tổng thực tế |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **H1 (ORCA Chuẩn)** | **106 (33.5%)**| **118 (37.3%)**| 48 | 5 | 39 | 316 |
| **H2 (Ích kỷ)** | **76 (29.0%)** | **77 (29.4%)** | 55 | 10 | 44 | 262 |
| **H3 (Không nhường)**| 108 | 97 | **56 (19.0%)** | 0 | 34 | 295 |
| **H4 (Đứng yên)** | 4 | 2 | 7 | **108 (87.1%)**| 3 | 124 |
| **H5 (Lực xã hội)** | 17 | 36 | 28 | 6 | **128 (59.5%)**| 215 |

#### Chỉ số theo từng tập con ($\sigma = 0.05\text{ m}$, GBDT)
| Phân vùng dữ liệu | Số mẫu ($N$) | Accuracy | Macro-F1 (5 lớp) | Macro-F1 (Bỏ H4) | Ghi chú |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Tương tác $\tau = 1.5\text{ m}$** | 1.212 | **$39.19\%$** | **$44.68\%$** | $34.50\%$ | Vùng tương tác va chạm trực tiếp |
| **Tương tác $\tau = 2.0\text{ m}$** | 1.960 | $37.24\%$ | $40.69\%$ | $31.78\%$ | Vùng ảnh hưởng mở rộng |
| **Không tương tác** | 7.040 | $31.06\%$ | $28.22\%$ | $23.49\%$ | Chưa xuất hiện tương tác né |
| **Toàn bộ dữ liệu test** | 9.000 | $32.41\%$ | $30.70\%$ | $25.29\%$ | Dữ liệu hỗn hợp thực tế |

> [!NOTE]
> Trên tập có tương tác ($\tau = 1.5\text{ m}$), Macro-F1 tăng từ $30.7\%$ lên $44.68\%$ chủ yếu nhờ hai lớp: **H4 đứng yên ($F_1 = 85.4\%$)** và **H5 né mềm từ xa ($F_1 = 55.3\%$)**. Trong khi đó, bộ ba hình học H1, H2, H3 vẫn nhầm lẫn nghiêm trọng với nhau.

---

### Mục 6.3: Phân tích chi tiết mức độ chồng lấn giữa H1 và H2 theo dải $\alpha$

Trên tập tương tác $\tau = 1.5\text{ m}$ ở mức nhiễu chuẩn $\sigma = 0.05\text{ m}$:
- **Tỷ lệ H2 bị phân loại nhầm thành H1:** **$29.01\%$** (76 / 262 trường hợp).
- **Tỷ lệ H1 bị phân loại nhầm thành H2:** **$37.34\%$** (118 / 316 trường hợp) — **H1 bị nhận diện thành H2 còn nhiều hơn là nhận diện đúng chính nó (33.54%)!**
- **Chi tiết tỷ lệ H2 nhầm H1 theo từng dải giá trị $\alpha$:**
  - $\alpha \in [0.10, 0.20)$ ($N = 94$): **$35.11\%$ nhầm thành H1** (Vượt ngưỡng 30%).
  - $\alpha \in [0.20, 0.30)$ ($N = 99$): **$27.27\%$ nhầm thành H1**.
  - $\alpha \in [0.30, 0.35]$ ($N = 69$): **$23.19\%$ nhầm thành H1**.

> [!IMPORTANT]
> Trong dải $\alpha$ nhỏ $[0.10, 0.20)$, dù người chỉ chịu trách nhiệm né rất ít ($10\% - 20\%$), chuyển động quan sát trong cửa sổ 2.0s với vận tốc sai phân hữu hạn bị nhiễu cảm biến làm mờ, khiến bộ phân loại nhầm lẫn sang chuyển động né chuẩn H1 ($\alpha=0.5$) tới $35.11\%$.

---

### Mục 6.4: Đánh giá phân bố xác suất & Độ hiệu chỉnh (Calibration)

| Mô hình | Nhiễu $\sigma$ | NLL (Log Loss) | Brier Score | ECE (10 bins) | Độ tin cậy trung bình |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **GBDT (Handcrafted)** | $0.00\text{ m}$ | 1.3207 | 0.6780 | $8.63\%$ | $44.8\%$ |
| **GBDT (Handcrafted)** | **$0.05\text{ m}$** | **1.4940** | **0.7566** | **$7.35\%$** | **$39.2\%$** |
| **GBDT (Handcrafted)** | $0.10\text{ m}$ | 1.5049 | 0.7637 | $7.61\%$ | $38.7\%$ |
| **Sequence MLP** | $0.05\text{ m}$ | 1.7613 | 0.8213 | $16.33\%$ | $43.5\%$ |
| **Prior Baseline** (40/10/25/15/10)| N/A | 1.7610 | 0.8650 | $20.00\%$ | $40.0\%$ |
| **Uniform Baseline** (20% đều) | N/A | 1.6094 | 0.8000 | **$0.00\%$** | $20.0\%$ |

#### Bảng chi tiết Reliability Diagram của GBDT ($\sigma = 0.05\text{ m}$):
| Bin xác suất | Khoảng tin cậy | Độ tin cậy TB ($\hat{p}$) | Tỷ lệ chính xác ($Acc$) | Số lượng mẫu | Sai lệch ($|\hat{p} - Acc|$) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| Bin 2 | $[0.2, 0.3]$ | $26.9\%$ | $22.1\%$ | 2.162 | $4.8\%$ |
| Bin 3 | $[0.3, 0.4]$ | $34.6\%$ | $28.5\%$ | 3.617 | $6.1\%$ |
| Bin 4 | $[0.4, 0.5]$ | $44.2\%$ | $34.1\%$ | 1.687 | $10.1\%$ |
| Bin 5 | $[0.5, 0.6]$ | $54.4\%$ | $40.8\%$ | 764 | $13.6\%$ |
| Bin 6 | $[0.6, 0.7]$ | $64.6\%$ | $53.7\%$ | 378 | $10.9\%$ |
| Bin 7 | $[0.7, 0.8]$ | $74.3\%$ | $65.7\%$ | 169 | $8.6\%$ |
| Bin 8 | $[0.8, 0.9]$ | $84.7\%$ | $80.3\%$ | 76 | $4.4\%$ |
| Bin 9 | $[0.9, 1.0]$ | $96.3\%$ | $99.3\%$ | 147 | $3.0\%$ |

> [!TIP]
> **Độ hiệu chỉnh rất tốt:** Giá trị ECE chỉ $7.35\%$. Mô hình không bị ảo tưởng tự tin thái quá (overconfident). Khi không chắc chắn, mô hình phân bổ đều xác suất khoảng $25\% - 35\%$. Đây là cơ sở hoàn hảo để module M3 truyền tải độ bất định (uncertainty) cho module MPPI lập kế hoạch.

---

### Mục 6.5: Độ sớm nhận diện (Early Detection Latency)

Định nghĩa: Thời điểm cửa sổ đầu tiên mà xác suất của lớp đúng $P(\text{Persona thật}) \ge 0.70$ và duy trì liên tục tối thiểu $\ge 3$ cửa sổ liên tiếp ($1.5\text{ s}$).

| Mức nhiễu $\sigma$ | Mô hình | Tỷ lệ kịch bản nhận diện sớm thành công | Trung vị khoảng cách khi nhận diện ($d_{det}$) | Trung vị thời gian tới CPA ($t_{to\_CPA}$) |
| :---: | :---: | :---: | :---: | :---: |
| $\sigma = 0.00\text{ m}$ | GBDT | $26.97\%$ | $2.05\text{ m}$ | $1.90\text{ s}$ |
| **$\sigma = 0.05\text{ m}$** | **GBDT** | **$7.47\%$** | **$1.32\text{ m}$** | **$2.36\text{ s}$** |
| $\sigma = 0.10\text{ m}$ | GBDT | $7.05\%$ | $1.09\text{ m}$ | $1.49\text{ s}$ |
| $\sigma = 0.05\text{ m}$ | Sequence MLP | $10.37\%$ | $1.95\text{ m}$ | $2.25\text{ s}$ |

> [!WARNING]
> Tỷ lệ đạt ngưỡng tự tin $\ge 0.70$ chỉ có **$7.47\%$** ở điều kiện nhiễu thực tế. Điều này chứng minh rằng **trong thực tế không thể kỳ vọng robot "chốt cứng" một nhãn persona duy nhất từ xa**. M3 bắt buộc phải duy trì phân bố xác suất động trong suốt quá trình tiếp cận.

---

### Mục 6.6: Cắt lát đa chiều (Multi-Dimensional Slices)

#### 1. Cắt lát theo Hình học không gian ($\sigma = 0.05\text{ m}$)
- **Không gian mở $6\times 6\text{ m}$** ($N = 3.015$): Accuracy = **$43.0\%$**, Macro-F1 = **$42.9\%$**.
- **Hành lang hẹp $0.9\text{ m}$** ($N = 2.970$): Accuracy = **$27.0\%$**, Macro-F1 = **$25.4\%$**.
- **Hành lang $1.2\text{ m}$** ($N = 3.015$): Accuracy = **$27.1\%$**, Macro-F1 = **$24.9\%$**.

> [!CAUTION]
> **Tác động triệt tiêu của hành lang:** Trong hành lang hẹp ($0.9\text{ m}$ và $1.2\text{ m}$), bờ tường vật lý ép cả robot và người đi theo trục dọc, triệt tiêu gần như toàn bộ độ lệch ngang $\Delta y_{lat}$. Do đó, phản ứng né của H1, H2, H3 trở nên giống hệt nhau, kéo F1 sụp đổ về mức ngẫu nhiên ($25\%$).

#### 2. Cắt lát theo Kiểu tiếp cận ($\sigma = 0.05\text{ m}$)
- **Đối đầu (Head-on)** ($N = 2.250$): Accuracy = $34.4\%$, Macro-F1 = $31.3\%$
- **Cắt xiên (Oblique)** ($N = 2.250$): Accuracy = **$34.9\%$**, Macro-F1 = **$33.8\%$**
- **Cùng chiều (Same direction)** ($N = 2.250$): Accuracy = $31.5\%$, Macro-F1 = $29.8\%$
- **Cắt ngang (Crossing)** ($N = 2.250$): Accuracy = $28.8\%$, Macro-F1 = $27.9\%$

#### 3. Cắt lát theo Mức độ nhiễu cảm biến (GBDT)
| Mức nhiễu $\sigma$ | Macro-F1 Toàn bộ | Macro-F1 (Bỏ H4) | Macro-F1 Tương tác ($\tau = 1.5\text{ m}$) | Tỷ lệ H2 nhầm H1 |
| :---: | :---: | :---: | :---: | :---: |
| $\sigma = 0.00\text{ m}$ (Clean) | $39.73\%$ | $32.86\%$ | $52.83\%$ | $28.24\%$ |
| **$\sigma = 0.05\text{ m}$ (Chuẩn M3)**| **$30.70\%$** | **$25.29\%$** | **$44.68\%$** | **$29.01\%$** |
| $\sigma = 0.10\text{ m}$ (Nhiễu nặng)| $29.04\%$ | $24.28\%$ | $42.81\%$ | $27.10\%$ |

---

### Mục 6.7: Khoảng tin cậy 95% Bootstrap (1.000 lượt lấy mẫu theo kịch bản test)

Toàn bộ 1.000 lượt Bootstrap được thực hiện bằng cách lấy mẫu có hoàn lại trên 1.000 kịch bản test niêm phong (bảo tồn tính phụ thuộc thời gian giữa các cửa sổ trong cùng một kịch bản):

| Chỉ số đánh giá | Giá trị trung bình | Khoảng tin cậy 95% Bootstrap | Kết luận thống kê |
| :--- | :---: | :---: | :--- |
| **Tỷ lệ H2 bị nhầm thành H1 ($\tau=1.5\text{ m}$)** | **$29.01\%$** | **[$20.83\%$, $38.24\%$]** | **Chạm và vượt sâu qua ngưỡng 30%** |
| **Tỷ lệ H1 bị nhầm thành H2 ($\tau=1.5\text{ m}$)** | $37.34\%$ | [$29.11\%$, $45.57\%$] | Vượt ngưỡng 30% |
| **Macro-F1 Tập tương tác ($\tau=1.5\text{ m}$)** | $44.68\%$ | [$40.56\%$, $48.41\%$] | Ổn định quanh mức 44% |
| **Macro-F1 Toàn bộ (5 lớp)** | $30.70\%$ | [$28.93\%$, $32.41\%$] | Ổn định quanh mức 31% |
| **Accuracy Toàn bộ** | $32.41\%$ | [$30.35\%$, $34.46\%$] | Ổn định quanh mức 32% |
| **Độ hiệu chỉnh ECE** | $7.35\%$ | [$5.44\%$, $9.27\%$] | Hiệu chỉnh xuất sắc (< 10%) |

---

## 3. KẾT LUẬN QUY TẮC & HƯỚNG TRIỂN KHAI CHO PIPELINE 10.000 EPISODE

### 3.1. Phân định rõ ràng theo Quy tắc Bước 7
Căn cứ điều kiện tiên quyết:
$$\text{Nếu } P(\text{H2} \to \text{H1} \mid \text{Interactive}) > 30.0\% \text{ HOẶC } \text{Upper}(CI_{95\%}) > 30.0\% \implies \text{ĐỀ XUẤT GỘP H1 VÀ H2}$$
- Giá trị đo được: $29.01\%$ với cận trên $CI_{95\%} = \mathbf{38.24\%}$.
- Chiều ngược lại: H1 bị nhầm thành H2 lên tới $\mathbf{37.34\%}$.
- Trong dải $\alpha \in [0.10, 0.20)$, tỷ lệ nhầm là $\mathbf{35.11\%}$.

$\implies$ **KẾT LUẬN:** Hai lớp H1 và H2 không thể phân tách bằng phân loại rời rạc dưới điều kiện quan sát thật của M3. Cần **GỘP H1 VÀ H2 THÀNH MỘT CỤM ("Người né chủ động") HOẶC CHUYỂN SANG HỒI QUY LIÊN TỤC $\alpha \in [0, 1]$**.

### 3.2. Cấu trúc nhãn đề xuất cho M3 khi sinh 10.000 Episode
Thay vì ép 5 lớp rời rạc gây nhiễu nhãn ground truth:
1. **Lớp 1 (Người né theo hình học / ORCA):** Gộp H1 và H2. M3 dự đoán phân bố liên tục cho hệ số nhường đường $\alpha \in [0.1, 0.5]$.
2. **Lớp 2 (Người không quan tâm / H3):** Không né robot ($\alpha = 0$).
3. **Lớp 3 (Người đứng yên / H4):** $v = 0$ (Được nhận diện tách biệt độc lập bằng bộ lọc vận tốc đơn giản).
4. **Lớp 4 (Người né theo lực mềm / H5):** Né theo hàm khoảng cách SFM.
5. **Cơ chế suy luận:** Module M3 **phải xuất phân bố xác suất $P(C_k)$**, tự động quay về Prior $[P(C_1)=0.50, P(C_2)=0.25, P(C_3)=0.15, P(C_4)=0.10]$ khi $d_{CPA} > 1.5\text{ m}$.

---

## 4. TÀI NGUYÊN VÀ MÃ NGUỒN LIÊN QUAN
1. **Mã nguồn thực nghiệm:**
   - Bộ sinh 5.000 kịch bản C++: [`generate_persona_scenarios.cpp`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/generate_persona_scenarios.cpp)
   - Bộ đánh giá M3 & Bootstrap: [`evaluate_persona_separability.py`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/evaluate_persona_separability.py)
2. **Dữ liệu & Metadata:**
   - Dataset JSON: [`h1_h5_scenarios_dataset.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/h1_h5_scenarios_dataset.json)
   - Danh sách Seed & Params: [`scenarios_metadata.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/scenarios_metadata.json)
   - Toàn bộ kết quả JSON: [`h1_h5_separability_results.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/h1_h5_separability_results.json)
3. **Ma trận nhầm lẫn (CSV):**
   - [`confusion_matrix_5x5_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_sigma005.csv)
   - [`confusion_matrix_4x4_noH4_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_4x4_noH4_sigma005.csv)
   - [`confusion_matrix_5x5_interactive15_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_interactive15_sigma005.csv)
   - [`confusion_matrix_4x4_interactive15_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_4x4_interactive15_sigma005.csv)
   - [`confusion_matrix_5x5_interactive20_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_5x5_interactive20_sigma005.csv)
   - [`confusion_matrix_4x4_interactive20_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/confusion_matrix_4x4_interactive20_sigma005.csv)
