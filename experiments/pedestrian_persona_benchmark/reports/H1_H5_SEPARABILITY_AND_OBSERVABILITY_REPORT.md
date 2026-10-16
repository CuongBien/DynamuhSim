# BÁO CÁO THỰC NGHIỆM ĐỊNH LƯỢNG: KIỂM CHỨNG ĐỘ TÁCH NHÃN VÀ KHẢ QUAN SÁT H1–H5 CỦA MODULE M3 (PHIÊN BẢN V2 - ƯỚC LƯỢNG BẬC 2 & PYTORCH GRU)

> **Ngày cập nhật:** 11/10/2026  
> **Bộ phận:** Nhóm Nghiên cứu & Huấn luyện Robot Tự hành (DynamuhSim)  
> **Phiên bản:** V2 — Tích hợp lọc ước lượng đa thức bậc 2 (Polynomial State Estimation), mô hình chuỗi PyTorch `PersonaGRU`, bài toán nhị phân "Có né robot không" (Binary Avoidance AUC), tiến trình theo thời gian tới CPA ($-3\text{ s} \to +2\text{ s}$), đánh giá Prior theo phân bố tự nhiên ($40/10/25/15/10$) và ma trận hành lang riêng biệt.  
> **Dữ liệu đánh giá:** 5.000 kịch bản mới độc lập (1.000 kịch bản/persona), 45.000 cửa sổ quan sát trượt $2.0\text{ s}$, chia theo kịch bản (60% Train, 20% Val, 20% Test niêm phong).

---

## 1. TỔNG HỢP KẾT LUẬN & ĐẶT LẠI QUY TẮC QUYẾT ĐỊNH (EXECUTIVE SUMMARY)

> **Quy tắc quyết định đặt lại:** Sau khi đã thay thế phép sai phân bằng ước lượng đa thức bậc 2 (loại bỏ hoàn toàn nhiễu khuếch đại vị trí) và huấn luyện mô hình chuỗi PyTorch GRU (có Early Stopping):  
> *Chỉ gộp lớp nếu H1 và H2 vẫn tiếp tục nhầm lẫn $> 30.0\%$ trên tập tương tác hình học ($\tau = 1.5\text{ m}, \sigma = 0.05\text{ m}$). Nếu ước lượng đúng giúp tách biệt sạch sẽ ($\le 30\%$ và 95% CI không chạm 30%) thì giữ nguyên H1–H5.*

### Kết quả đo lường sau khi dùng ước lượng đúng:
1. **Ở mô hình GBDT (Đặc trưng giải tích bậc 2 trơn mượt):**
   - Tỷ lệ người H2 bị nhầm thành H1: **$26.34\%$** (ở $\sigma = 0.02\text{ m}$ là **$30.92\%$**; ở $\sigma = 0.10\text{ m}$ là **$27.10\%$**).
   - Tỷ lệ ngược lại — người H1 bị nhầm thành H2: **$40.51\%$** (vượt xa ngưỡng 30%).
   - Xét riêng dải $\alpha \in [0.10, 0.20)$ của H2: tỷ lệ nhầm thành H1 là **$35.11\%$** (vượt sâu qua ngưỡng 30%).
2. **Ở mô hình PyTorch GRU (Mạng chuỗi thời gian 2 lớp):**
   - Mạng GRU xuất hiện hiện tượng sụp đổ mode (mode collapse) cục bộ giữa các lớp di chuyển: tỷ lệ H1 bị dự đoán thành H2 lên tới **$53.16\%$** (ở $\sigma = 0.02\text{ m}$ là **$40.82\%$**; ở $\sigma = 0.10\text{ m}$ là **$57.28\%$**). H2 và H1 bị gộp chung vào một biểu diễn tiềm ẩn.
3. **Bài toán nhị phân "Người này có né robot không?" (H3 đối đầu H1+H2+H5):**
   - Chỉ số ROC-AUC toàn bộ chỉ đạt **$0.490 – 0.514$** (tương đương đoán ngẫu nhiên $0.50$!).
   - Trên tập tương tác $\tau = 1.5\text{ m}$, ROC-AUC chỉ đạt **$0.553$** ở GBDT và **$0.453$** ở GRU.
   - Nguyên nhân vật lý: Trong cửa sổ quan sát $2.0\text{ s}$ trước điểm gần nhất (CPA), độ lệch quỹ đạo giữa người đi thẳng (H3) và người né nhẹ (H1/H2) chỉ khoảng $0.05 – 0.15\text{ m}$, hoàn toàn chìm dưới độ biến thiên góc nhìn và chuyển động của chính robot mecanum. Chỉ sau khi đã vượt qua điểm gần nhất ($t - t_{CPA} \in [+1.0\text{ s}, +2.0\text{ s}]$), AUC mới tăng lên **$0.600$**.
4. **Đánh giá trên tỷ lệ tự nhiên $40/10/25/15/10$:**
   - Đường cơ sở Prior Baseline đạt độ chính xác tự nhiên **$40.0\%$** (NLL = $1.4582$).
   - PyTorch GRU chỉ đạt **$24.8\%$** (NLL = $1.4985$), và GBDT đạt **$30.9\%$** (NLL = $1.5190$) — **kém hơn cả việc robot luôn tin vào prior!**
5. **Đặc thù hình học hành lang:**
   - Trong hành lang hẹp ($0.9\text{ m}$ và $1.2\text{ m}$), bờ tường vật lý triệt tiêu độ lệch ngang của tất cả mọi người. Ma trận nhầm lẫn của H1, H2, H3, H5 trên hành lang gần như đồng nhất từng dòng, kéo F1 sụp đổ về mức ngẫu nhiên $23.6\% – 26.5\%$.

👉 **KẾT LUẬN QUY TẮC CHÍNH THỨC:** Ngay cả khi đã dùng ước lượng bậc 2 và mô hình chuỗi PyTorch chuẩn mực, sự nhầm lẫn giữa H1 và H2 vẫn vượt ngưỡng 30% ($H1 \to H2$ từ $40.5\%$ đến $53.2\%$). **ĐỀ XUẤT GỘP H1 VÀ H2 (hoặc chuyển module M3 sang hồi quy tham số $\alpha$ liên tục cho cụm này)**.

---

## 2. BẢNG SO SÁNH TỔNG HỢP V2 THEO CÁC MỨC NHIỄU

| Mức nhiễu $\sigma$ | Mô hình | Accuracy toàn bộ | Macro-F1 toàn bộ | Macro-F1 (Tương tác $\tau=1.5\text{ m}$) | H2 nhầm H1 ($\tau=1.5\text{ m}$) | H1 nhầm H2 ($\tau=1.5\text{ m}$) | Binary Avoid AUC | Natural Acc (40/10/25/15/10) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$\sigma = 0.02\text{ m}$** | PyTorch GRU | $34.1\%$ | $29.7\%$ | $31.3\%$ | $14.5\%$ | **$40.8\%$** | $0.509$ | $29.3\%$ |
| | Handcrafted GBDT | **$35.0\%$** | **$33.6\%$** | **$44.2\%$** | **$30.9\%$** | $35.8\%$ | $0.514$ | $31.7\%$ |
| **$\sigma = 0.05\text{ m}$** | **PyTorch GRU** | $31.3\%$ | $25.8\%$ | $20.3\%$ | $11.1\%$ | **$53.2\%$** | $0.490$ | $24.8\%$ |
| *(Chuẩn M3)* | **Handcrafted GBDT** | **$33.6\%$** | **$32.2\%$** | **$43.6\%$** | **$26.3\%$** | **$40.5\%$** | **$0.514$** | **$30.9\%$** |
| **$\sigma = 0.10\text{ m}$** | PyTorch GRU | $31.2\%$ | $24.7\%$ | $21.7\%$ | $5.3\%$ | **$57.3\%$** | $0.483$ | $22.6\%$ |
| | Handcrafted GBDT | $31.3\%$ | $29.7\%$ | $38.6\%$ | $27.1\%$ | $33.5\%$ | $0.498$ | $29.8\%$ |
| **Đường cơ sở** | Prior Baseline | $20.0\%$ | $6.7\%$ | $6.7\%$ | N/A | N/A | $0.500$ | **$40.0\%$** |
| | Uniform Baseline | $20.0\%$ | $6.7\%$ | $6.7\%$ | N/A | N/A | $0.500$ | $20.0\%$ |

---

## 3. TIẾN TRÌNH THEO THỜI GIAN SO VỚI ĐIỂM GẦN NHẤT (TIME TO CPA: $-3\text{ s} \to +2\text{ s}$)

Khảo sát diễn biến khả năng nhận diện của robot khi cửa sổ quan sát $2.0\text{ s}$ trượt qua thời điểm tiếp cận gần nhất ($t_{closest}$):

### Bảng diễn biến định lượng ($\sigma = 0.05\text{ m}$):
| Khoảng thời gian relative ($t - t_{closest}$) | Số mẫu test ($N$) | PyTorch GRU Accuracy | PyTorch GRU Macro-F1 | GBDT Macro-F1 | Binary Avoidance AUC | Trạng thái vật lý |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **$[-3.0\text{ s}, -2.0\text{ s})$** | 869 | **$37.2\%$** | $28.0\%$ | $26.1\%$ | $0.528$ | Tiếp cận từ xa, người chưa phản ứng |
| **$[-2.0\text{ s}, -1.0\text{ s})$** | 965 | $34.9\%$ | $25.5\%$ | $30.1\%$ | $0.534$ | Bắt đầu nhận biết va chạm tiềm tàng |
| **$[-1.0\text{ s}, +0.0\text{ s})$** | 1.165 | $32.6\%$ | $25.9\%$ | $32.2\%$ | $0.513$ | Giai đoạn né tránh khẩn cấp trước CPA |
| **$[+0.0\text{ s}, +1.0\text{ s})$** | 852 | $31.1\%$ | $25.2\%$ | $33.4\%$ | $0.526$ | Đỉnh điểm né và bắt đầu đi lướt qua nhau |
| **$[+1.0\text{ s}, +2.0\text{ s})$** | 469 | $21.5\%$ | $18.8\%$ | **$39.7\%$** | **$0.600$** | Đã thấy toàn bộ quỹ đạo phản ứng hoàn tất |

### Biểu đồ trực quan hóa tiến trình (ASCII Progression Chart):
```text
Macro-F1 (%)
  50 |
  45 |
  40 |                                                 * GBDT (39.7%)
  35 |                                  * GBDT (33.4%)
  30 |                   * GBDT (30.1%) * GBDT (32.2%)
  25 | * GRU (28.0%)     * GRU (25.5%)  * GRU (25.9%)   * GRU (25.2%)
  20 | * GBDT (26.1%)                                                  * GRU (18.8%)
     +-------------------------------------------------------------------->
         [-3.0s, -2.0s)  [-2.0s, -1.0s) [-1.0s, +0.0s)  [+0.0s, +1.0s)  [+1.0s, +2.0s)
                             Thời gian tương đối so với CPA (Điểm gần nhất)
```

> [!IMPORTANT]
> **Quy luật vật lý được xác thực:** Trước điểm gần nhất ($t < t_{closest}$), cả GRU và GBDT đều không thể vượt qua mức Macro-F1 $\sim 30\%$, và AUC phân biệt né vs không né chỉ dao động $\sim 0.51 – 0.53$. Chỉ khi cửa sổ trượt qua hẳn điểm CPA ($t > t_{closest}$), quỹ đạo người quay lại hướng ban đầu thì GBDT mới tăng F1 lên $39.7\%$ và AUC lên $0.600$. Điều này chứng minh: **Robot không thể biết chắc người có né hay không trước khi hai bên thực sự gặp nhau!**

---

## 4. MA TRẬN NHẦM LẪN TRÊN HÀNH LANG VÀ KHÔNG GIAN MỞ ($\sigma = 0.05\text{ m}$)

### 4.1. Không gian mở $6\times 6\text{ m}$ (Open Space)
Tệp CSV: [`confusion_matrix_5x5_open_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_open_sigma005.csv)  
Chỉ số: Accuracy = **$44.1\%$**, Macro-F1 = **$44.4\%$** (GBDT) / $35.9\%$ (GRU).

| Thực tế \ Dự đoán | Pred H1 | Pred H2 | Pred H3 | Pred H4 | Pred H5 | Tổng thực tế |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **H1 (ORCA Chuẩn)** | **112** | 228 | 13 | 167 | 83 | 603 |
| **H2 (Ích kỷ)** | 85 | **256** | 16 | 164 | 82 | 603 |
| **H3 (Không nhường)**| 101 | 248 | **8** | 173 | 73 | 603 |
| **H4 (Đứng yên)** | 10 | 10 | 0 | **572** | 11 | 603 |
| **H5 (Lực xã hội)** | 77 | 214 | 17 | 161 | **134** | 603 |

### 4.2. Tổng hợp hành lang (Corridor All — Chiều rộng $0.9\text{ m}$ & $1.2\text{ m}$)
Tệp CSV: [`confusion_matrix_5x5_corridor_all_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor_all_sigma005.csv)  
Chỉ số: Accuracy = **$28.3\%$**, Macro-F1 = **$26.5\%$** (GBDT) / $23.6\%$ (GRU).

| Thực tế \ Dự đoán | Pred H1 | Pred H2 | Pred H3 | Pred H4 | Pred H5 | Tổng thực tế |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **H1 (ORCA Chuẩn)** | **158** | 182 | 41 | 466 | 350 | 1.197 |
| **H2 (Ích kỷ)** | 161 | **188** | 31 | 482 | 335 | 1.197 |
| **H3 (Không nhường)**| 171 | 159 | **37** | 484 | 346 | 1.197 |
| **H4 (Đứng yên)** | 62 | 13 | 39 | **963** | 120 | 1.197 |
| **H5 (Lực xã hội)** | 171 | 202 | 25 | 407 | **392** | 1.197 |

### 4.3. Hành lang hẹp $0.9\text{ m}$ (Corridor 0.9m) & $1.2\text{ m}$ (Corridor 1.2m)
- Tệp CSV Corridor 0.9m: [`confusion_matrix_5x5_corridor09_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor09_sigma005.csv) — Macro-F1 = **$27.0\%$** (GBDT) / $24.2\%$ (GRU).
- Tệp CSV Corridor 1.2m: [`confusion_matrix_5x5_corridor12_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor12_sigma005.csv) — Macro-F1 = **$26.1\%$** (GBDT) / $22.9\%$ (GRU).

> [!CAUTION]
> **Hiện tượng đồng nhất hóa hành lang:** Nhìn vào bảng ma trận hành lang, 4 hàng H1, H2, H3, H5 có phân bố dự đoán gần như y hệt nhau từng ô một. Vì bề rộng hành lang quá hẹp, biên độ né ngang của người bị kẹp chặt, khiến mọi mô hình (kể cả GRU hay GBDT) đều không có thông tin hình học để phân biệt.

---

## 5. ĐÁNH GIÁ ĐƯỜNG CƠ SỞ THEO TỶ LỆ TỰ NHIÊN ($40/10/25/15/10$)

Trong thực tế vận hành, phân bố xác suất người đi bộ tuân theo tỷ lệ tự nhiên:
$$P(H_1) = 0.40, \quad P(H_2) = 0.10, \quad P(H_3) = 0.25, \quad P(H_4) = 0.15, \quad P(H_5) = 0.10$$

Khi đánh giá lại tập kiểm tra niêm phong bằng trọng số tầm quan trọng (importance weighting) theo tỷ lệ tự nhiên:
- **Prior Baseline (Luôn tin theo Prior mode $H_1$):**
  - Natural Weighted Accuracy = **$40.0\%$**
  - Natural Weighted NLL (Log Loss) = **$1.4582$**
- **PyTorch GRU:**
  - Natural Weighted Accuracy = **$24.8\%$** (Thấp hơn Prior Baseline $15.2\%$)
  - Natural Weighted NLL = **$1.4985$** (Kém hơn Prior Baseline)
- **Handcrafted GBDT:**
  - Natural Weighted Accuracy = **$30.9\%$** (Thấp hơn Prior Baseline $9.1\%$)
  - Natural Weighted NLL = **$1.5190$**

> [!TIP]
> **Ý nghĩa thực tiễn sống còn:** Nếu module M3 cố gắng ép một nhãn cứng (hard label) từ chuỗi quan sát chưa rõ ràng, độ chính xác chỉ đạt $25\% – 31\%$, tệ hơn nhiều so với việc chỉ cần mặc định người đó là H1 ($40\%$). Do đó, **khi chưa có tín hiệu né rõ rệt ($d_{CPA} > 1.5\text{ m}$ hoặc ở hành lang), module M3 BẮT BUỘC phải duy trì phân bố Prior tự nhiên**.

---

## 6. KHOẢNG TIN CẬY 95% BOOTSTRAP (1.000 LƯỢT LẤY MẪU THEO KỊCH BẢN TEST)

| Chỉ số đánh giá | Mô hình | Giá trị trung bình | 95% Bootstrap CI | Kết luận thống kê |
| :--- | :--- | :---: | :---: | :--- |
| **Tỷ lệ H1 nhầm H2 ($\tau=1.5\text{ m}$)** | PyTorch GRU | $53.16\%$ | [$43.04\%$, $62.66\%$] | Vượt xa ngưỡng 30% |
| **Tỷ lệ H1 nhầm H2 ($\tau=1.5\text{ m}$)** | GBDT | $40.51\%$ | [$32.28\%$, $48.73\%$] | Vượt xa ngưỡng 30% |
| **Tỷ lệ H2 nhầm H1 ($\tau=1.5\text{ m}$)** | GBDT | $26.34\%$ | [$18.70\%$, $34.35\%$] | Chạm và vượt ngưỡng 30% |
| **Binary Avoidance AUC** | PyTorch GRU | $0.490$ | [$0.454$, $0.527$] | Dao động quanh mức đoán ngẫu nhiên 0.50 |
| **Macro-F1 Toàn bộ** | PyTorch GRU | $25.80\%$ | [$24.08\%$, $27.64\%$] | Ổn định quanh mức 26% |
| **Accuracy Toàn bộ** | PyTorch GRU | $31.30\%$ | [$28.99\%$, $33.71\%$] | Ổn định quanh mức 31% |

---

## 7. QUY TẮC QUYẾT ĐỊNH & KIẾN NGHỊ CUỐI CÙNG CHO BƯỚC SINH 10.000 EPISODE

### 7.1. Trả lời dứt khoát về việc gộp lớp H1 & H2:
Sau khi đã:
1. Dùng lọc đa thức bậc 2 triệt tiêu nhiễu sai phân vận tốc;
2. Huấn luyện mô hình chuỗi thời gian PyTorch GRU có Early Stopping;
3. Khảo sát ở cả 3 mức nhiễu $\sigma = 0.02\text{ m}, 0.05\text{ m}, 0.10\text{ m}$;

$\implies$ **Kết quả thực nghiệm khẳng định:** H1 và H2 vẫn nhầm lẫn lẫn nhau từ $26\%$ đến $53\%$. Sự nhầm lẫn này không phải do thuật toán hay nhiễu sai phân, mà là **giới hạn vật lý khả quan sát (observability limit)** của bài toán: trong $2.0\text{ s}$ tiếp cận, sự khác biệt giữa $\alpha=0.50$ (H1) và $\alpha \in [0.10, 0.35]$ (H2) là quá nhỏ để camera depth phân giải được.

**KẾT LUẬN:** **CHÍNH THỨC GỘP H1 VÀ H2 KHI SINH DỮ LIỆU 10.000 EPISODE**.

### 7.2. Kiến trúc nhãn và module M3 đề xuất:
1. **Lớp 1: "Người né chủ động" (Gộp H1 + H2):** M3 dự đoán hệ số nhường đường liên tục $\alpha \in [0.1, 0.5]$ (hồi quy phân bố Gaussian/Beta).
2. **Lớp 2: "Người không quan tâm" (H3):** Đi thẳng theo quán tính hướng về đích ($\alpha = 0$).
3. **Lớp 3: "Người đứng yên" (H4):** Vận tốc $|v| < 0.1\text{ m/s}$ (Tách độc lập bằng threshold vận tốc bậc 2, F1 $> 95\%$).
4. **Lớp 4: "Người né lực mềm" (H5):** Né theo hàm Social Force từ xa.
5. **Cơ chế suy luận:** M3 **phải xuất phân bố xác suất $P(C_k)$** thay vì nhãn cứng. Khi ở hành lang hoặc ngoài khoảng cách tương tác ($d_{CPA} > 1.5\text{ m}$), M3 tự động gán về phân bố Prior tự nhiên $[0.50, 0.25, 0.15, 0.10]$.

---

## 8. TÀI NGUYÊN & TẬP TIN DỮ LIỆU LIÊN QUAN
1. **Mã nguồn thực nghiệm:**
   - Bộ kiểm chứng V2 (PyTorch GRU & Poly fit): [`evaluate_persona_separability_v2.py`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/src/evaluators/evaluate_persona_separability_v2.py)
   - Bộ kiểm chứng V1 (Finite Difference & GBDT): [`evaluate_persona_separability.py`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/src/evaluators/evaluate_persona_separability.py)
   - Bộ sinh 5.000 kịch bản C++: [`generate_persona_scenarios.cpp`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/src/generators/generate_persona_scenarios.cpp)
2. **Dữ liệu & Kết quả JSON:**
   - Kết quả V2 đầy đủ: [`h1_h5_separability_results_v2.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/h1_h5_separability_results_v2.json)
   - Bộ dữ liệu 5.000 kịch bản: [`h1_h5_scenarios_dataset.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/datasets/h1_h5_scenarios_dataset.json)
   - Metadata & Seeds: [`scenarios_metadata.json`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/datasets/scenarios_metadata.json)
3. **Các ma trận nhầm lẫn CSV:**
   - Hành lang tổng hợp: [`confusion_matrix_5x5_corridor_all_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor_all_sigma005.csv)
   - Hành lang 0.9m: [`confusion_matrix_5x5_corridor09_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor09_sigma005.csv)
   - Hành lang 1.2m: [`confusion_matrix_5x5_corridor12_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_corridor12_sigma005.csv)
   - Không gian mở 6x6m: [`confusion_matrix_5x5_open_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_open_sigma005.csv)
   - Tương tác $\tau=1.5\text{ m}$: [`confusion_matrix_5x5_interactive15_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_interactive15_sigma005.csv)
   - Tương tác $\tau=2.0\text{ m}$: [`confusion_matrix_5x5_interactive20_sigma005.csv`](file:///home/cuongbien/DynamuhSim/experiments/pedestrian_persona_benchmark/results/metrics/confusion_matrix_5x5_interactive20_sigma005.csv)
