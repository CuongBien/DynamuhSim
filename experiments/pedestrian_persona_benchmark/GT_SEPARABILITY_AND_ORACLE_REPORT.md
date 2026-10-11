# Báo Cáo Kiểm Chứng Độ Phân Tách Nhãn H1–H5 & Giới Hạn Trên Ground Truth (Oracle Upper Bound)

> **Mục tiêu**: Đo lường thực nghiệm tính khả phân (kinematic separability) của 5 persona người (**H1–H5**) trên 2.500 kịch bản ngẫu nhiên; thiết lập **Giới hạn trên (Oracle Upper Bound)** bằng bộ phân loại trên quỹ đạo chân trị Ground Truth để trả lời dứt khoát câu hỏi: *Liệu có cần gộp lớp trước khi sinh dữ liệu lớn (10.000 episode) không?*

---

## 1. Thiết Kế Thí Nghiệm & Không Gian Kịch Bản

* **Số lượng mẫu**: 2.500 tương tác hoàn chỉnh (500 mẫu cho mỗi persona H1, H2, H3, H4, H5).
* **Không gian tương tác**:
  1. `head_on` (Đối đầu trực diện): Người đi ngược chiều robot với độ lệch trục ngang ban đầu $y_{\text{offset}} \in [-0.35, +0.35]\text{ m}$.
  2. `crossing` (Cắt ngang): Người đi vuông góc $90^\circ$ qua lộ trình robot.
  3. `oblique` (Cắt xiên): Người đi chéo góc $45^\circ / 135^\circ$.
  4. `same_direction` (Cùng chiều / Vượt): Robot đi nhanh hơn ($0.6\text{ m/s}$) vượt người đi chậm ($0.4\text{ m/s}$).
* **Phổ tham số động học**:
  * Robot: Vận tốc tịnh tiến $v_r \in [0.4, 0.7]\text{ m/s}$.
  * Người H1: ORCA chuẩn $\alpha = 0.50$, $v_h \in [0.65, 1.0]\text{ m/s}$.
  * Người H2: ORCA bất đối xứng, quét liên tục $\alpha \sim \mathcal{U}[0.10, 0.35]$.
  * Người H3: Lờ robot hoàn toàn (`ignoresRobot = true`), loại khỏi KdTree ORCA.
  * Người H4: Tĩnh đứng yên ($v_{\text{pref}} = 0$, $v_h = 0$).
  * Người H5: Lực đẩy xã hội mềm $F_{\text{rep}} = A \exp(-d_{\text{surf}}/B)$ ($A \in [2.0, 3.0]$, $B \in [0.35, 0.50]$) + bộ lọc thông thấp LPF ($\tau \in [0.25, 0.35]\text{ s}$).

---

## 2. Phân Bố Độ Lệch Ngang & Phân Tích Vùng Ranh Giới H1 vs H2 ($\alpha = 0.35$)

### 2.1. Thống kê độ lệch ngang cực đại $|Y|$ (Toàn bộ 2.500 kịch bản)

| Persona | Ý Nghĩa Động Học | Trung Bình (Mean) | Trung Vị (Median) | Khoảng Tứ Phân (IQR) | Cực Đại (Max) |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **H1** | Hợp tác đầy đủ ($\alpha = 0.50$) | $0.0977\text{ m}$ | $0.0786\text{ m}$ | $[0.0000, 0.1858]$ | $0.3083\text{ m}$ |
| **H2** | Ít nhường ($\alpha \in [0.10, 0.35]$) | $0.0554\text{ m}$ | $0.0326\text{ m}$ | $[0.0000, 0.0997]$ | $0.2369\text{ m}$ |
| **H3** | Lờ robot (`ignoresRobot`) | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | $[0.0000, 0.0000]$ | **$0.0000\text{ m}$** |
| **H4** | Đứng yên ($v=0$) | **$0.0000\text{ m}$** | **$0.0000\text{ m}$** | $[0.0000, 0.0000]$ | **$0.0000\text{ m}$** |
| **H5** | Lực đẩy mềm (SFM) | $0.1222\text{ m}$ | $0.0497\text{ m}$ | $[0.0134, 0.1811]$ | $0.9011\text{ m}$ |

*Ghi chú*: H3 có độ lệch ngang bằng 0 m tuyệt đối vì không chịu bất kỳ lực hay mặt phẳng cấm ORCA nào từ robot.

### 2.2. Chi tiết phân rã dải $\alpha$ của H2 vs H1 trong kịch bản đối đầu (Head-on)

Trong kịch bản đối đầu trực diện — nơi tương tác diễn ra mạnh mẽ và trực tiếp nhất:

```
H1 (alpha = 0.50)           : Mean = 0.1971 m | Median = 0.1952 m | Range = [0.1090, 0.3072] m
H2 (0.10 <= alpha < 0.20)   : Mean = 0.0863 m | Median = 0.0782 m | Range = [0.0441, 0.1577] m
H2 (0.20 <= alpha < 0.30)   : Mean = 0.1268 m | Median = 0.1216 m | Range = [0.0622, 0.2139] m
H2 (0.30 <= alpha <= 0.35)  : Mean = 0.1426 m | Median = 0.1381 m | Range = [0.0913, 0.2369] m
H3 (alpha = 0.00, blind)    : Mean = 0.0000 m | Median = 0.0000 m | Range = [0.0000, 0.0000] m
H5 (Social Force SFM)       : Mean = 0.1039 m | Median = 0.0436 m | Range = [0.0006, 0.7389] m
```

#### Nhận xét quan trọng về cận trên $\alpha = 0.35$:
1. **Khoảng cách phân biệt**: Ngay cả ở cận trên khó nhất ($\alpha \in [0.30, 0.35]$), trung vị lệch ngang của H2 ($0.1381\text{ m}$) vẫn thấp hơn rõ rệt so với H1 ($0.1952\text{ m}$), chênh lệch gần $6\text{ cm}$.
2. **Tại sao H5 không bị nhầm với H2?**:
   - Dù biên độ lệch ngang của H5 ($0.04 - 0.18\text{ m}$) nằm cùng thang đo với H2, **chữ ký không gian - thời gian (spatio-temporal signature)** của H5 hoàn toàn khác:
     * H5 bắt đầu phản ứng từ cự ly rất xa ($d \approx 2.5 - 3.0\text{ m}$) do trường lực hàm mũ lan xa, tạo độ cong quỹ đạo mượt mà và biến thiên gia tốc liên tục.
     * H2 đi thẳng tắp tới sát cự ly cảnh báo va chạm của ORCA ($d \approx 1.2 - 1.5\text{ m}$) rồi mới thực hiện bước bẻ góc vận tốc tức thời.
     * Vì vậy, bộ phân loại dựa trên đặc trưng chuỗi thời gian tách H5 đạt F1-score **0.97 – 1.00**, gần như không có sự nhầm lẫn giữa H5 và H2.

---

## 3. Kết Quả Huấn Luyện Bộ Phân Loại Oracle (Cửa Sổ Quan Sát 2.0 s & Đánh Giá Xác Suất)

Để phản ánh chính xác bài toán M3 thời gian thực mà **không bị rò rỉ dữ liệu tương lai (no future leakage)**, bộ phân loại Oracle chỉ được cung cấp chuỗi quan sát **$t \le 2.0\text{ s}$** (20 bước ở tần số $10\text{ Hz}$). Bộ phân loại được hiệu chuẩn xác suất (Calibrated via Isotonic/Sigmoid) để xuất ra phân bố $P(H_i \mid \mathbf{o}_{1:t})$.

### 3.1. Đánh giá trên Toàn Bộ Tập Kiểm Tra (Full Test Set, Cửa sổ $2.0\text{ s}$)
* **Overall Accuracy (H1–H5)**: **$83.04\%$** | **Overall Macro-F1**: **$83.02\%$**
* **Dynamic-Only Accuracy (H1, H2, H3, H5 - Tách riêng H4 Static)**: **$78.80\%$** | **Dynamic Macro-F1**: **$78.77\%$**
* **Chỉ số chất lượng phân bố xác suất**:
  * **Negative Log-Likelihood (NLL)**: **$0.3701$**
  * **Brier Score**: **$0.1988$**
  * **Expected Calibration Error (ECE)**: **$7.48\%$** (Độ tin cậy khớp sát với tỷ lệ chính xác thực tế, không bị overconfident).

### 3.2. Đánh giá Khách Quan trên Tập Xung Đột Hình Học Danh Định (Nominal CPA $\le 1.2\text{ m}$)
> [!IMPORTANT]
> **Loại bỏ thiên lệch lựa chọn (Selection Bias / Circularity)**: Trước đây việc lọc tập tương tác bằng $d_{\min}$ đo được sau chuyển động vô tình thiên vị H3 (vì H3 không né nên $d_{\min}$ luôn nhỏ), đẩy độ chính xác ảo lên $90\% - 97\%$.
> Phép đo chuẩn hóa sử dụng **Nominal CPA** tính tại $t=0$ khi hai bên chưa đổi hướng:
> $$d_{\text{CPA}}^{\text{nominal}} = \min_{t \ge 0} \|\mathbf{p}_h(0) + \mathbf{v}_h(0) t - (\mathbf{p}_r(0) + \mathbf{v}_r(0) t)\| \le 1.20\text{ m}$$

* **Số lượng mẫu xung đột hình học**: $511$ mẫu trong tập kiểm tra.
* **Accuracy (H1–H5)**: **$83.95\%$** | **Macro-F1**: **$85.52\%$**
* **Dynamic-Only Accuracy (không có H4)**: **$81.86\%$** | **Dynamic Macro-F1**: **$81.90\%$**
* **NLL**: **$0.3613$** | **Brier Score**: **$0.1933$**
* **Ma trận nhầm lẫn (Nominal CPA $\le 1.2\text{ m}$)**:
  * **H1 (Cooperative)**: $79$ đúng, $17$ nhầm H2, $17$ nhầm H3, $0$ nhầm H4, $0$ nhầm H5 (Recall = $69.9\%$).
  * **H2 (Unyielding)**: $87$ đúng, $12$ nhầm H1, $16$ nhầm H3, $0$ nhầm H4, $0$ nhầm H5 (Recall = $75.7\%$).
  * **H3 (Robot-Blind)**: $93$ đúng, $5$ nhầm H1, $14$ nhầm H2, $0$ nhầm H4, $0$ nhầm H5 (Recall = $83.0\%$).
  * **H4 (Static)**: $59/59$ đúng ($100.0\%$).
  * **H5 (Social Force)**: $111/112$ đúng ($99.1\%$).

### 3.3. Độ Trễ Phát Hiện Sớm (Early Detection Latency Curve)
Khảo sát sự tiến triển của độ chính xác và độ tự tin khi độ dài cửa sổ quan sát tăng dần từ khi bắt đầu gặp gỡ:

| Cửa Sổ Quan Sát | Accuracy | Macro-F1 | Negative Log-Likelihood (NLL) | Độ Tự Tin Trung Bình |
| :---: | :---: | :---: | :---: | :---: |
| **$0.5\text{ s}$** (5 bước) | $71.04\%$ | $74.18\%$ | $0.5851$ | $71.51\%$ |
| **$1.0\text{ s}$** (10 bước) | $76.91\%$ | $79.20\%$ | $0.4906$ | $77.09\%$ |
| **$1.5\text{ s}$** (15 bước) | $80.63\%$ | $82.62\%$ | $0.4113$ | $79.58\%$ |
| **$2.0\text{ s}$** (20 bước) | **$83.37\%$** | **$84.97\%$** | **$0.3620$** | **$81.33\%$** |

*Nhận xét*: Ngay từ $1.0\text{ s}$, mô hình đã đạt độ chính xác gần $77\%$ và sau $2.0\text{ s}$ đạt đỉnh ổn định $83.4\%$.

### 3.4. Thử nghiệm Hồi Quy Hệ Số Liên Tục $\alpha$ (H1 & H2)
* Khi hồi quy $\alpha \in [0.10, 0.50]$:
  * Sai số tuyệt đối trung bình **$\text{MAE} = 0.0531$** (tương đương sai số $\pm 0.05$ trên thang $0.50$).
  * Hệ số xác định $R^2 = 0.643$.
  * Phản ánh đúng thực tế: sự chồng lấn nhẹ giữa $\alpha \in [0.30, 0.35]$ và $\alpha = 0.50$ tạo ra dải bất định tự nhiên, phù hợp với cách tiếp cận xuất phân bố xác suất thay vì quyết định nhị phân cứng nhắc.

---

## 4. Bốn Kết Luận Khoa Học & Quyết Định Kỹ Thuật

1. **Khả năng phân tách H1–H5 đạt $79\% - 84\%$ trên cửa sổ thực tế 2.0 s — KHÔNG CẦN GỘP LỚP**:
   * Ngay cả với cửa sổ hạn chế $2.0\text{ s}$ và không có thông tin tương lai, độ chính xác động học đạt $78.8\% - 81.9\%$. Duy trì đầy đủ phổ 5 lớp là hoàn toàn khả thi và cần thiết để mô tả hành vi người phong phú.
2. **Khắc phục triệt để thiên lệch lựa chọn bằng Nominal CPA**:
   * Thay thế hoàn toàn tiêu chí hậu nghiệm $d_{\min}$ bằng Nominal CPA ($d_{\text{CPA}}^{\text{nominal}} \le 1.20\text{ m}$). Tiêu chí này bảo đảm tính khách quan hình học và không bị méo mó bởi phản ứng của người.
3. **Huấn luyện trên toàn bộ tập dữ liệu với Đầu ra Phân bố Xác suất (Probabilistic Formulation)**:
   * **Bỏ khuyến nghị cắt lọc tập huấn luyện**: Mô hình M3 phải được huấn luyện trên toàn bộ tập dữ liệu quan sát.
   * Khi không có xung đột hoặc ở cự ly xa, con người đi thẳng theo quán tính; mô hình M3 xuất phân bố xác suất $P(H_i \mid \mathbf{o}_{1:t})$ tiệm cận về phân bố tiên nghiệm $[0.40, 0.10, 0.25, 0.15, 0.10]$. ECE đạt $7.48\%$ bảo đảm các xác suất này có độ tin cậy thực tế cao.
4. **Lưu trữ tạo tác & Dữ liệu thực nghiệm**:
   * Biểu đồ hiệu chuẩn xác suất và đường cong trễ phát hiện: `experiments/pedestrian_persona_benchmark/probabilistic_calibration_latency.png`.
   * Số liệu định lượng chi tiết: `experiments/pedestrian_persona_benchmark/probabilistic_evaluation_results.json`.
