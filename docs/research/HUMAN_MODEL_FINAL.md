1. Môi trường: Mujoco ([https\://github.com/CuongBien/DynamuhSim/tree/mujoco-hummodel](https://github.com/CuongBien/DynamuhSim/tree/mujoco-hummodel))   
2. Chi tiết các thuật toán riêng … → chung (tính sau): 

### **Test lần lượt các thuật toán**  

- ORCA thuần   
- SFM (dùng từ JuPedSim)   
- AVM (dùng từ JuPedSim)   
- CSM (dùng từ JuPedSim)   
3. Chi tiết các episode 

### **Số episode đề xuất**

| Giai đoạn | Số episode | Mục đích |
| ----- | ----- | ----- |
| Vòng 0 | 48 | Kiểm tra thuật toán hoạt động đúng  $4thuậttoán\times 6kịchbản\times 2seed=48$ |
| Vòng 1 | 540 (lũy kế) | So sánh chuyển động và tương tác \- 360 episode đánh giá chuyển động và tương tác $4tt\times 6kbản\times 3cauhinh(/kban)\ x\ 5seed=360$ \- 180: so sánh tương tác với robot (Chọn ba tình huống có tương tác người–robot rõ ràng, ví dụ đối đầu trong hành lang, cửa hẹp và giao cắt. Với mỗi tình huống dùng một cấu hình hình học quan trọng nhất, chạy thêm ba hành động robot so với nhánh tiến đã có) |
| Vòng 2 | 900 (lũy kế) | Tăng seed, xác nhận kết quả (bổ sung 15 seed cho mỗi thuật toán và kịch bản)  |

4. Chi tiết các metrics và Chi tiết cách đánh giá theo từng metrics

Căn cứ hoàn toàn vào tài liệu chuẩn **`3700599.pdf`** (*"Principles and Guidelines for Evaluating Social Robot Navigation Algorithms"* của tác giả A. Francis và cộng sự, ACM THRI 2025), dưới đây là bản **tổng hợp đầy đủ, chính xác toàn bộ hệ thống metrics**, quy định cụ thể về **mức đạt chuẩn (ngưỡng "ổn") cho từng chỉ số** và **nguyên tắc xét một episode đạt hay không đạt**.

---

### **PHẦN 1: BẢNG TỔNG HỢP TOÀN BỘ CÁC METRICS THEO TÀI LIỆU `3700599.pdf`**

Tài liệu chuẩn hóa các chỉ số theo 4 nhóm chính:

1. **Chỉ số Thành công kỹ thuật & Thất bại (Success & Failure Metrics \- Table 1\)**  
2. **Chỉ số Chất lượng chuyển động & Xã hội (Quality and Social Metrics \- Table 1\)**  
3. **Chỉ số Thống kê quy mô & Độ phủ kịch bản (Scenario Scale & Coverage Metrics \- Sec 5.2, Table 4, 5, 6, Guidelines D1–D3, B4)**  
4. **Chỉ số Gán nhãn & Học từ dữ liệu (Learned & Surveyed Realism Metrics \- Sec 6.1, 6.4.2, 6.4.3, Guidelines M4–M6)**  
   ---

   #### **Nhóm I: Chỉ số An toàn & Thành công kỹ thuật (Success & Failure Metrics)**

| Tên Metric & Ký hiệu | Định nghĩa trong tài liệu | Ngưỡng / Mức thế nào được coi là "Ổn" theo bài báo |
| ----- | ----- | ----- |
| **Thành công (Success \- $S$, $SR$)** | Biến Boolean xác định robot có chạm đến đích hay không. Tính bình quân trên tập thử nghiệm là Success Rate ($SR$). | **Bắt buộc $S=1$** cho episode đạt. Trên toàn tập thử nghiệm, $SR$ phải tiệm cận **$1.0$ (100%)**, các benchmark chuẩn thường yêu cầu $\geq 90-95%$. |
| **Va chạm (Collision \- $C$, $CR$)** | Tổng số va chạm trong quỹ đạo. Bình quân là Collision Rate ($CR$). Phân tách thành: va chạm tường ($WC$), va chạm tác nhân khác ($AC$), và va chạm người ($HC$). | **Bắt buộc $C=WC=AC=HC=0$**. Nguyên tắc P1 (Safety) yêu cầu bảo vệ tuyệt đối con người, robot và môi trường xung quanh. |
| **Quá giờ (Timeout \- $TO$)** | Biến Boolean ghi nhận thất bại do hết thời gian quy định trước khi đến đích ($TO\in 0,1$). | **Bắt buộc $TO=0$**. Thời gian hoàn thành phải thỏa mãn $T<{T}_{max}$. |
| **Không tiến triển (Failure to progress \- $FP$)** | Số lần thất bại do không giảm được khoảng cách tới đích trong một khoảng thời gian quy định. | **$FP=0$**. Robot không được rơi vào tình trạng bế tắc vĩnh viễn hoặc đóng băng chuyển động ("freezing robot problem"). |
| **Thời gian dừng/kẹt (Stalled time \- $ST$)** | Thời lượng mà độ lớn vận tốc robot giảm xuống dưới một ngưỡng xác định. | **Càng nhỏ càng tốt (tiệm cận 0s)**. Tránh việc robot dừng lại thụ động quá lâu. |
| **Thời gian về đích (Time to reach goal \- $T$)** | Khoảng thời gian từ lúc nhận nhiệm vụ đến khi hoàn thành. | **Càng nhỏ càng tốt**. Đảm bảo tính kịp thời của nhiệm vụ. |
| **Độ dài đường đi (Path length \- $PL$)** | Tổng chiều dài thực tế của quỹ đạo di chuyển. | **Càng gần khoảng cách ngắn nhất càng tốt**. |
| **Thành công theo độ dài (SPL)** | Tỷ lệ thành công chuẩn hóa theo tỷ số giữa đường ngắn nhất $l$ và chiều dài thực tế $p$: $SPL=S\times \frac{l}{\max\limits_{}(p,l)}$. | **Càng tiệm cận 1.0 càng tốt**; mức chấp nhận được trong môi trường động có người thường từ **$\geq 0.7-0.8$**. |

   ---

   #### **Nhóm II: Chỉ số Chuyển động & Chất lượng Xã hội (Motion, Quality & Social Metrics)**

| Tên Metric & Ký hiệu | Định nghĩa trong tài liệu | Ngưỡng / Mức thế nào được coi là "Ổn" theo bài báo |
| ----- | ----- | ----- |
| **Đặc trưng vận tốc (${V}_{min},{V}_{avg},{V}_{max}$)** | Vận tốc tuyến tính nhỏ nhất, trung bình và lớn nhất trên quỹ đạo. | Nằm trong dải vận tốc vận hành an toàn của phần cứng, không có đột biến âm/dương bất thường. |
| **Gia tốc tuyến tính (${A}_{min},{A}_{avg},{A}_{max}$)** | Gia tốc tuyến tính nhỏ nhất, trung bình và lớn nhất. | Nằm trong giới hạn động học êm ái, tránh phanh gấp làm người xung quanh giật mình hoặc căng thẳng (P2: Comfort). |
| **Độ giật chuyển động (${J}_{min},{J}_{avg},{J}_{max}$)** | Đạo hàm bậc hai của tốc độ tuyến tính (độ mượt quỹ đạo / jerk). | **Càng nhỏ càng tốt**. Jerk cao thể hiện quỹ đạo giật cục (navigation jitter), làm suy giảm nghiêm trọng mức độ thoải mái của con người. |
| **Khoảng cách vật cản ($C{D}_{min},C{D}_{avg}$)** | Khoảng cách tối thiểu và trung bình tới các vật cản/tường trong suốt quỹ đạo. | **$C{D}_{min}>0$** (tuyệt đối không bằng 0 để tránh va chạm). Lề an toàn phụ thuộc kích thước hình học không gian. |
| **Tuân thủ không gian cá nhân (Space compliance / PSC)** | Tỷ lệ thời lượng quỹ đạo mà khoảng cách tới người nhỏ hơn ngưỡng quy định. Khi đặt ngưỡng 0.5m, gọi là Personal Space Compliance ($PSC$). | **Tỷ lệ vi phạm càng thấp càng tốt (lý tưởng \= 0\)**. Theo thuyết Proxemics trong bài: vùng thân mật là $0.45m$, vùng cá nhân là $1.2m$. Tuyệt đối không xâm phạm vùng $0.45m$ trừ khi điều kiện hình học bắt buộc. |
| **Khoảng cách tối thiểu tới người ($D{H}_{min}$)** | Khoảng cách ngắn nhất ghi nhận giữa robot và người trong suốt quỹ đạo. | **$D{H}_{min}\geq 0.5m$** (trong không gian bình thường là $\geq 1.2m$; ở điểm nghẽn cho phép co hẹp nhưng tuyệt đối không chạm vào người $D{H}_{min}>0$). |
| **Thời gian tối thiểu tới va chạm ($TTC$)** | Thời gian dự báo ngắn nhất tới va chạm nếu các tác nhân tiếp tục di chuyển thẳng đều. | **Càng lớn càng tốt**. Phải duy trì khoảng thời gian đủ cho phản xạ tránh né (thường khuyến nghị $TTC>1.5s-2.0s$, không để $TTC\rightarrow 0$). |
| **Thời gian tổng hợp ($AT$)** | Tổng thời gian để một nhóm tác nhân hợp tác cùng hoàn thành nhiệm vụ của mình. | **Càng tối ưu càng tốt**; thể hiện robot nhường đường/hợp tác hiệu quả, không cản trở hành trình của người khác. |

   ---

   #### **Nhóm III: Thống kê Quy mô & Độ phủ Kịch bản (Scenario Scale & Coverage Metrics)**

Nhóm chỉ số này dùng để đánh giá **tính tin cậy và độ đa dạng của tập dữ liệu mô phỏng** (Sec 5.2, Sec 7, Sec 9, Guidelines D1–D3, B4):

| Tên Metric theo `3700599.pdf` | Định nghĩa & Phạm vi đo lường | Mức thế nào được coi là "Ổn" theo chuẩn bài báo |
| ----- | ----- | ----- |
| **Tổng số lượng quỹ đạo (Trajectory Count)** | Tổng số quỹ đạo của con người và robot được ghi nhận trong tập dữ liệu. | Tùy bài toán, nhưng cần đạt từ vài trăm đến hàng nghìn/hàng chục nghìn quỹ đạo để đủ dữ liệu huấn luyện/kiểm thử (các dataset chuẩn như THOR có 600 quỹ đạo, TrajNet++ có \>200K quỹ đạo). |
| **Tổng thời lượng dữ liệu (Duration Traveled / Duration)** | Tổng thời gian mô phỏng hoặc tương tác được thu thập (phút hoặc giờ). | Phải đủ dài để bao quát đầy đủ các giai đoạn: tiếp cận, giải quyết xung đột và giải phóng lối đi (ví dụ SCAND có 522 phút, SACSoN có 75 giờ). |
| **Mật độ & Số lượng người (People Count and Density)** | Số lượng người xuất hiện và mật độ trên diện tích không gian. | Phải phân tầng rõ ràng giữa **Mật độ thấp (Low-density pedestrian: 1–2 người)** và **Đám đông (Crowd: từ 3 người trở lên)** theo Mục 7.2.1. Với hành lang hẹp, bắt buộc lấy mẫu ở mật độ 1–2 người để tránh kẹt vô lý trước. |
| **Số lượt chạm mặt (Pedestrian Encounter Count)** | Số lần robot thực sự bước vào vùng tương tác trực tiếp với con người. | Càng nhiều càng tốt; dữ liệu phải có tương tác thực chất, không lấy các đoạn robot chỉ chạy một mình trong map trống. |
| **Độ bao phủ kịch bản (Scenario Coverage \- Guideline N5, D1)** | Mức độ dữ liệu bao quát đầy đủ các tình huống xã hội thông dụng trong thực tế. | **Bắt buộc bao phủ đủ các kịch bản chuẩn trong Bảng 3 (Table 3\)**: Đi đối mặt (*Frontal Approach*), Người vượt robot (*Pedestrian Overtaking*), Robot vượt người (*Robot Overtaking*), Giao lộ (*Intersection*), Góc khuất (*Blind Corner*), Cửa hẹp (*Narrow Doorway*). |
| **Mật độ lấy mẫu kịch bản (Well-Sampled Coverage \- Guideline D3, B4)** | Số lượng phân cảnh (episodes/trials) được thu thập cho mỗi kịch bản cụ thể với các biến thể vị trí, vận tốc. | **Tối thiểu 30 đến 50 episode/kịch bản** (Guideline B4 nêu rõ quy tắc thực nghiệm: *"A good rule of thumb is at least 30 samples for trials"* để đảm bảo ý nghĩa thống kê). |
| **Bao phủ trường hợp biên (Corner / Edge Cases Coverage \- Guideline D1, B4)** | Tỷ lệ dữ liệu thu thập các tình huống khó, người đi bộ bất thường, đổi hướng đột ngột hoặc không hợp tác. | Benchmark phải chủ động đưa các ca biên này vào làm bài kiểm tra hồi quy (regression tests) để thuật toán không bị suy thoái. |
| **Tỷ lệ chế độ lỗi mô phỏng (Simulation Failure Modes)** | Thống kê số lượng/tỷ lệ các phân cảnh mô phỏng bị lỗi vật lý: xuyên tường, xuyên người, teleport. | **Bắt buộc \= 0%** đối với các lỗi phi vật lý. Các lỗi về logic xã hội phải được gắn nhãn tách bạch với lỗi thuật toán. |

---

#### **Nhóm IV: Chỉ số Gán nhãn & Học từ dữ liệu (Learned & Surveyed Realism Metrics)**

Nhóm này quy định cách đánh giá tính tự nhiên, độ thoải mái của con người (Sec 6.1, 6.4.2, 6.4.3, Guidelines M4–M6):

| Tên Metric theo `3700599.pdf` | Bản chất phân loại | Ngưỡng đánh giá thế nào là "Ổn" theo chuẩn |
| ----- | ----- | ----- |
| **Đánh giá chủ quan qua khảo sát (Surveyed Human Ratings \- Guideline M5)** | Sử dụng bảng câu hỏi chuẩn hóa (thang đo Likert 5 mức: từ *Rất không an toàn* đến *Rất an toàn*) để con người trực tiếp đánh giá độ an toàn (P1), thoải mái (P2: Comfort), độ dễ hiểu (P3: Legibility), và lịch sự (P4: Politeness). | Được xem là **thước đo chuẩn vàng (Gold Standard)**. Khảo sát phải đạt độ tin cậy nội bộ qua hệ số **Cronbach's alpha ($\alpha \geq 0.7$)** và kiểm định phân tích phương sai ANOVA để loại bỏ thiên kiến. |
| **Chỉ số học từ dữ liệu (Learned Metrics \- Guideline M4)** | Sử dụng các mô hình nơ-ron được huấn luyện trên dữ liệu tương tác người thật có gán nhãn (như SocNav1, SocNav2, AutoRL) để tự động xuất ra điểm số chấp nhận chuẩn mực xã hội (*Social Acceptability Score*). | Điểm số chấp nhận xã hội phải tiệm cận với phân phối điểm do con người đánh giá. Dùng làm bộ lọc nghiệm thu nhanh (**Acceptance Tests**) trước khi đưa vào thử nghiệm thực tế tốn kém. |
| **Độ dễ hiểu của chuyển động (Legibility & Visibility \- P3, Sec 6.4.3)** | Đo lường góc và khoảng cách mà robot xuất hiện trong trường nhìn của con người (*Visibility Indices*). | Robot phải tiến vào trường nhìn của con người từ khoảng cách đủ xa và giữ hướng đi nhất quán để người quan sát dễ dàng đoán được ý định di chuyển, không bị bất ngờ. |

---

### **PHẦN 2: MỘT EPISODE CẦN PASS QUA BAO NHIÊU METRICS MỚI ĐƯỢC TÍNH LÀ ĐẠT?**

Theo cấu trúc **Thẻ kịch bản (Scenario Card \- Mục 7.2, Table 2\)** và các nguyên tắc phân loại metric (Mục 6.1, 6.5), quy trình đánh giá một episode **KHÔNG** dùng cơ chế "tính tỷ lệ phần trăm các metrics đạt" (ví dụ: pass 7/10 metrics là đạt), mà phân định rạch ròi thành hai cấp độ: **Điều kiện Tiên quyết (Hard Constraints)** và **Thang điểm Đo lường Phẩm chất (Continuous Quality Scores)**.

#### **1\. Điều kiện Tiên quyết để một Episode được tính là "ĐẠT" (Pass Criteria)**

Một episode đơn lẻ được tính là **ĐẠT (Success \= 1\)** khi và chỉ khi **PASS TOÀN BỘ 100% CÁC TIÊU CHÍ AN TOÀN & NHIỆM VỤ CỐT LÕI (Failure Modes \= 0\)**:

1. **Không có bất kỳ va chạm nào:** $C=0$ (bao gồm cả va chạm tường $WC=0$, va chạm vật thể/robot khác $AC=0$, và va chạm người $HC=0$). Nếu có va chạm, episode bị tính là **Thất bại ngay lập tức (Terminated / Failure)**.  
2. **Chạm đến đích thành công:** Robot phải hoàn thành nhiệm vụ di chuyển đến vị trí mục tiêu ($S=1$).  
3. **Không bị hết giờ:** $TO=0$ (hoàn thành trước thời gian trần quy định $T<{T}_{max}$).  
4. **Không bị kẹt / không tiến triển:** $FP=0$ (không bị đóng băng đứng yên hoặc bế tắc vô hạn).

👉 **Kết luận:** Đối với các chỉ số an toàn và nhiệm vụ ($S,C,TO,FP$), robot **BẮT BUỘC PHẢI PASS TOÀN BỘ (Pass 100%)**. Chỉ cần vi phạm 1 tiêu chí (ví dụ: đâm trúng người hoặc hết giờ), episode đó bị tính là **FAIL (Thất bại)**.

#### **2\. Vai trò của các metrics còn lại trong một Episode**

Các metrics còn lại (như $SPL,PSC,D{H}_{min},TTC,Jerk$, hay điểm khảo sát) **không dùng làm điều kiện loại trừ nhị phân Pass/Fail cho từng episode**, mà dùng để **chấm điểm chất lượng (Quality Grading)** cho những episode đã pass an toàn:

* Sau khi episode đã đạt ($S=1,C=0$), các chỉ số này sẽ cho biết: Quỹ đạo về đích có mượt mà không hay bị giật cục ($Jerk$)? Quãng đường đi có tối ưu không ($SPL$)? Robot có tôn trọng khoảng cách xã hội và không gian riêng tư của con người hay không ($PSC,D{H}_{min},TTC$)?

  #### **3\. Quy tắc Báo cáo Thống kê Đạt chuẩn (Guideline M8 & B2)**

Theo Hướng dẫn **M8 (Report Results in Depth)** và **B2 (Include Quantitative Metrics)**, đánh giá một thuật toán không chỉ dựa vào một vài episode đơn lẻ, mà phải báo cáo **biểu đồ phân phối tần suất (Histograms) và các giá trị ngoại lệ (Outliers)** trên toàn bộ tập dữ liệu:

* **Tỷ lệ thành công tổng thể ($SR$):** Càng cao càng tốt (mục tiêu chuẩn là $\geq 90-95%$).  
* **Tỷ lệ va chạm tổng thể ($CR$):** Bắt buộc tiệm cận $0$.  
* **Phân phối của $PSC,D{H}_{min},TTC,Jerk$:** Giúp phát hiện các trường hợp ngoại lệ nguy hiểm (outliers), chứng minh rằng thuật toán điều hướng luôn duy trì chuyển động ổn định và tôn trọng chuẩn mực xã hội trong mọi kịch bản.

4\. Chi tiết các kịch bản 

Dựa trên chuẩn **Scenario Card** từ bài báo **Francis và cộng sự (2025) – `3700599.pdf`** (Mục 7.2, Bảng 2 và các nguyên tắc N1–N8), việc thiết kế Card cho một pipeline thực nghiệm có nhiều vòng (Vòng 0, 1, 2\) và nhiều thuật toán/giai đoạn cần được module hóa chuẩn xác:

Thay vì viết gộp chung một kịch bản chung chung, bạn nên chia thành:

1. **Scenario Base Card (Thẻ Kịch bản Cơ sở):** Định nghĩa chuẩn cho từng loại trong **6 mẫu tương tác**.  
2. **Phase Execution Card (Thẻ Điều phối Thực nghiệm theo Giai đoạn):** Định nghĩa cấu hình tham số, số episode và điều kiện lọc qua các **Vòng 0, 1, 2**.

Dưới đây là thiết kế chuẩn hoàn chỉnh, bám sát các thuật toán (**ORCA, SFM, AVM, CSM**) và các biến độc lập của bạn:

---

### **PHẦN 1: MẪU SCENARIO CARD CHUẨN CHO 6 KỊCH BẢN (BASE SCENARIO CARD)**

Dưới đây là bộ **6 Social Navigation Scenario Cards hoàn chỉnh (Cards 1 đến 6\)** được chuẩn hóa theo đúng cấu trúc của bài báo **Francis và cộng sự (ACM THRI 2025 – `3700599.pdf`, Mục 7.2, Bảng 2, Bảng 3\)** kết hợp với hệ thống **chỉ số đánh giá (Metrics)**.

Các Card này được thiết kế đồng nhất về mặt tham số, sẵn sàng để nạp trực tiếp vào pipeline thực nghiệm tách riêng 4 thuật toán (**ORCA, SFM, AVM, CSM**) và kiểm soát các biến độc lập (độ rộng $w$, mật độ, chính sách người, hành động robot).

### **🃏 CARD 1: ĐỐI ĐẦU TRONG HÀNH LANG (Frontal Approach)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `FA_Corridor_Passing` (Đi đối mặt / Tránh nhau đối đầu).  
  * **Scientific Purpose:** Đánh giá khả năng dự đoán ý định né tránh song phương (Reciprocal Collision Avoidance) trong không gian giới hạn và mức độ tuân thủ không gian cá nhân (Proxemics / Principle P2: Comfort, P5: Social Norms).  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Corridor` (hành lang thẳng, chiều dài $\geq 8m$).  
  * **Effective Width ($w$):**  
    * *Easy*: $2.1m$ (Đủ không gian né hai bên song song an toàn).  
    * *Critical*: $1.2m$ (Bắt buộc ép sát tường, xâm phạm ranh giới vùng không gian cá nhân).  
    * *Hard*: $0.9m$ (Điểm nghẽn sát nút $w<{b}_{r}+{b}_{h}+2{d}_{safe}$, đòi hỏi giảm tốc hoặc xoay thân).  
  * **Pedestrian Density:** 1 người đi đối mặt (Low-density) hoặc 2 người đi song song/so le.  
  * **Human Behavior Policy:** Hợp tác (biết lách phải), Trung tính, Ít hợp tác (đi thẳng, giữ tốc độ).  
  * **Robot Action Policy:** 4 nhánh đối chứng (`Advance`, `Stop`, `Yield/Turn`, `Reverse`).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Góc hướng mặt ban đầu $\Delta \theta \approx 18{0}^{\circ }\pm 1{5}^{\circ }$, khoảng cách ban đầu ${d}_{0}\geq 5m$.  
  * **Hard Constraints (Bắt buộc Pass 100% từng episode):**  
    * $C=0$ (Tuyệt đối không va chạm tường $WC=0$, không va chạm người $HC=0$).  
    * $S=1$ (Chạm đích thành công trong thời gian cho phép $T\leq {T}_{max}$).  
    * $FP=0$ & $TO=0$ (Không bị quá giờ, không bị kẹt/freezing quá 5s).  
  * **Quality & Social Metrics (Chấm điểm phân phối):**  
    * *An toàn & Xã hội:* $PSC$ (Tỷ lệ vi phạm vùng $0.5m$), $D{H}_{min}$ (Khoảng cách người tối thiểu), $TTC$ (Thời gian tối thiểu tới va chạm).  
    * *Chuyển động:* $Jer{k}_{max,avg}$ (Độ mượt, kiểm tra hiện tượng bẻ lái giật cục), ${A}_{max}$, ${V}_{avg}$.  
    * *Hiệu quả:* $SPL$ (Success weighted by Path Length), $PL$ (Độ dài quãng đường).  
  * **Failure Modes:** Va chạm người/tường, deadlock đối đầu trực diện (Head-on Deadlock), dao động lắc lư liên tục tại chỗ (Reciprocal Dance / Oscillation Jitter).

---

### **🃏 CARD 2: CỬA HẸP / ĐIỂM NGHẼN (Narrow Doorway / Bottleneck)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `ND_Doorway_Passing` (Cửa hẹp / Điểm nghẽn một lối).  
  * **Scientific Purpose:** Đánh giá cơ chế giải quyết xung đột không gian không thể lưu thông đồng thời (Single-Agent Bottleneck); kiểm tra hành vi lịch sự (P4: Politeness), sự chủ động (P7: Proactivity) và năng lực nhường đường (Yielding).  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Hospital` / `School` / `Warehouse` (Vách ngăn chia 2 phòng nối bằng 1 cửa).  
  * **Effective Width ($w$):**  
    * *Easy*: $1.2m$ (Có thể lách qua mép cửa nếu giảm tốc).  
    * *Critical*: $0.9m$ (Bắt buộc 1 bên đi qua trước, 1 bên dừng chờ ngoài cửa).  
    * *Hard*: $0.6m$ (Chỉ vừa lọt 1 đối tượng, điều kiện $w<{b}_{r}+{b}_{h}$).  
  * **Pedestrian Density:** 1 người (tiếp cận từ phòng đối diện) hoặc Nhóm 2 người nối đuôi.  
  * **Human Behavior Policy:** Hợp tác (dừng trước vạch cửa nhường), Trung tính, Ít hợp tác (lao qua trước không giảm tốc).  
  * **Robot Action Policy:** Thử nghiệm nhánh `Advance`, `Stop`, `Yield` (dịch mép vách), `Reverse` (lùi nhường).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Cả người và robot cách tâm cửa $\leq 3m$ tại cùng thời điểm, có quỹ đạo cắt qua cửa.  
  * **Hard Constraints (Bắt buộc Pass 100%):** $C=0$, $S=1$, $TO=0$, $FP=0$.  
  * **Quality & Social Metrics:**  
    * *Tương tác & Xã hội:* $TTC$ (Khi cùng tiến về tâm cửa), $ST$ (Stalled Time \- thời gian đứng chờ ngoài cửa có hợp lý không), $AT$ (Aggregated Time \- tổng thời gian để cả người và robot cùng thoát qua cửa an toàn).  
    * *Chuyển động:* ${V}_{min}$, ${A}_{min}$ (Kiểm tra phản xạ phanh hãm mượt mà từ xa khi thấy đối phương tới gần mép cửa).  
  * **Failure Modes:** Cùng chui vào cửa gây kẹt cứng (Bottleneck Deadlock), va chạm mép cửa (Wall Collision), robot lùi vô tận ra khỏi phòng.

---

### **🃏 CARD 3: GÓC KHUẤT CHỮ L (Blind Corner Encounter)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `BC_Corner_Turn` (Gặp nhau tại góc rẽ che khuất tầm nhìn).  
  * **Scientific Purpose:** Đánh giá khả năng phản xạ an toàn khi thông tin cảm biến bị che khuất đột ngột; kiểm tra tính dễ hiểu (P3: Legibility) và khoảng cách quét an toàn vật cản tĩnh (P1: Safety).  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Corridor` / `Hospital` (Giao lộ góc vuông chữ L, có tường chắn che khuất tầm nhìn LiDAR/Camera).  
  * **Effective Width ($w$):**  
    * *Easy*: $2.1m$ (Khúc cua mở rộng, có tầm quét góc).  
    * *Critical*: $1.2m$ (Hành lang hẹp, che khuất hoàn toàn tầm nhìn đến khi cách góc cua \< 1.5m).  
    * *Hard*: $0.9m$ (Góc cua hẹp, hai bên vừa bước qua góc rẽ là chạm mặt sát nút).  
  * **Pedestrian Density:** 1 người đi ngược chiều ôm sát góc cua.  
  * **Human Behavior Policy:** Hợp tác (ôm rộng góc cua/né ngay khi thấy robot), Trung tính, Ít hợp tác (ôm sát mép tường, đi nhanh).  
  * **Robot Action Policy:** `Advance` (ôm cua), `Stop` (phanh khi phát hiện), `Yield` (dạt rộng góc cua).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Hai tác nhân di chuyển trên 2 nhánh vuông góc của khúc cua, không có đường ngắm trực tiếp (Line of Sight) cho đến khi cả hai cách góc rẽ $\leq 2m$.  
  * **Hard Constraints (Bắt buộc Pass 100%):** $C=0$, $S=1$, $TO=0$, $FP=0$.  
  * **Quality & Social Metrics:**  
    * *An toàn & Xã hội:* $D{H}_{min}$ tại thời điểm vừa lộ diện sau góc khuất; $PSC$ (xâm phạm không gian khi giật mình tránh nhau).  
    * *Chuyển động & Động lực học:* ${A}_{min}$ (Gia tốc phanh khẩn cấp), $Jer{k}_{max}$ (Kiểm tra robot có bị phanh giật đột ngột gây mất an toàn không).  
    * *Khoảng cách vật cản tĩnh:* $C{D}_{min}$ (Khoảng cách tới góc tường để tránh cọ quẹt mép góc cua).  
  * **Failure Modes:** Va chạm người ngay tại điểm rẽ do không phanh kịp, va chạm quẹt tường ở góc cua, robot đóng băng cứng (freezing) ngay khúc rẽ.

---

### **🃏 CARD 4: GIAO CẮT VUÔNG GÓC (Perpendicular Intersection)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `IN_Crossing_Paths` (Cắt ngang / Ngã tư hành lang).  
  * **Scientific Purpose:** Đánh giá khả năng dự đoán quỹ đạo chéo, điều tiết vận tốc để nhường đường hoặc vượt qua giao lộ mà không cắt mặt người (P4: Politeness, P6: Understanding Other Agents).  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Corridor` / `Warehouse` / `Hospital` (Giao lộ chữ thập hình chữ X hoặc chữ T).  
  * **Effective Width ($w$):**  
    * *Easy*: $2.1m$ cho cả hai trục đường.  
    * *Critical*: $1.2m$ (Trục đường giao cắt hẹp, vùng xung đột nhỏ).  
    * *Hard*: $0.9m$ (Không gian hẹp, đường đi cắt chính xác vào cùng một tọa độ giao cắt).  
  * **Pedestrian Density:** 1 người (cắt ngang từ trái sang phải hoặc ngược lại) hoặc 2 người cắt ngang liên tiếp.  
  * **Human Behavior Policy:** Hợp tác (chủ động giảm tốc nhường đường), Trung tính (giữ nguyên tốc độ), Ít hợp tác (tăng tốc băng qua giao lộ).  
  * **Robot Action Policy:** `Advance` (đi đều), `Stop` (dừng trước ngã tư nhường), `Yield` (giảm tốc đi sau người).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Góc quỹ đạo tiếp cận $\Delta \theta \approx 9{0}^{\circ }\pm 2{0}^{\circ }$; hai bên dự kiến chạm tâm giao lộ trong khoảng thời gian chênh lệch $|\Delta t|\leq 2.0s$.  
  * **Hard Constraints (Bắt buộc Pass 100%):** $C=0$, $S=1$, $TO=0$, $FP=0$.  
  * **Quality & Social Metrics:**  
    * *Tương tác:* $TT{C}_{min}$ (Đo liên tục trước khi tới tâm giao lộ), $PSC$ (Tuân thủ không gian cá nhân khi cắt mặt).  
    * *Chất lượng lộ trình:* $SPL$ (Kiểm tra robot có phải lượn vòng quá xa để tránh người không), $PI$ (Độ bất thường/quanh co của đường đi).  
    * *Lịch sự:* Không cắt ngang sát trước mặt người (Crossing distance \> 1.2m).  
  * **Failure Modes:** Đâm nhau ở tâm giao lộ, cắt sát mặt người làm người phải dừng đột ngột, dừng kẹt vĩnh viễn ở ngã tư vì sợ va chạm (Passive Obstacle Deadlock).

---

### **🃏 CARD 5: VƯỢT / DI CHUYỂN CHẬM CÙNG CHIỀU (Overtaking)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `OT_Same_Direction` (Người vượt robot hoặc Robot vượt người).  
  * **Scientific Purpose:** Đánh giá hành vi điều hướng mượt mà khi cùng chiều; đo lường sự thoải mái (P2: Comfort) khi có tác nhân tiến sát từ phía sau ngoài tầm nhìn trực tiếp.  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Corridor` / `Warehouse` (Hành lang dài $\geq 10m$).  
  * **Effective Width ($w$):**  
    * *Easy*: $2.1m$ (Đủ làn để vượt song song cách xa người).  
    * *Critical*: $1.6m$ hoặc $1.2m$ (Khoảng hở bên sườn hẹp, bắt buộc lách sát hông).  
    * *Hard*: $0.9m$ (Không thể vượt song song; người/robot phía sau bắt buộc phải bám đuôi giảm tốc).  
  * **Pedestrian Density:** 1 người đi cùng chiều phía trước (tốc độ chậm $0.4m/s$) hoặc 1 người đi nhanh từ phía sau vượt lên ($1.1m/s$).  
  * **Human Behavior Policy:** Hợp tác (dạt sang mép nhường lối), Trung tính (đi đều), Ít hợp tác (đi loạng choạng giữa lối).  
  * **Robot Action Policy:** `Advance` (chủ động vượt khi có khoảng trống), `Follow` (bám đuôi giữ khoảng cách an toàn), `Yield` (dạt sang một bên cho người sau vượt qua).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Góc lệch hướng di chuyển ban đầu $\Delta \theta \leq 2{0}^{\circ }$; khoảng cách đuổi bắt đầu ${d}_{0}\leq 4m$.  
  * **Hard Constraints (Bắt buộc Pass 100%):** $C=0$, $S=1$, $TO=0$, $FP=0$.  
  * **Quality & Social Metrics:**  
    * *Khoảng cách & Không gian:* $D{H}_{min}$ khi lướt ngang hông đối phương, $PSC$ (vi phạm vùng thân mật $0.45m$ khi vượt).  
    * *Động lực học:* $Jer{k}_{max}$ (Độ mượt quỹ đạo khi chuyển làn vượt), chênh lệch vận tốc tương đối $\Delta V$.  
    * *Thời gian dừng:* $ST$ (Nếu không vượt được, robot giữ khoảng cách bám đuôi hay dừng hẳn lại).  
  * **Failure Modes:** Đâm vào lưng người từ phía sau, ép người vào tường khi đang vượt, vượt ẩu gây cọ quẹt hông.

---

### **🃏 CARD 6: NHÓM ĐÔNG / ĐA TÁC NHÂN (Multi-Agent & Small Group Encounter)**

* **1\. Scenario Metadata:**  
  * **Scenario Name:** `MG_Small_Group` (Gặp nhóm người / Băng qua đám đông nhỏ).  
  * **Scientific Purpose:** Đánh giá khả năng điều hướng trong mật độ đông; kiểm tra xem thuật toán có phá vỡ liên kết nhóm (Social Norms \- P5, ví dụ: tách đôi hai người đang đi cùng nhau) hoặc bị tê liệt chuyển động hay không (Freezing Robot Problem).  
  * **Algorithms Evaluated:** ORCA thuần, SFM (JuPedSim), AVM (JuPedSim), CSM (JuPedSim).  
* **2\. Geometric & Agent Setup:**  
  * **Environment:** `Arena` / `Warehouse` / `Hospital`.  
  * **Effective Width ($w$):** $1.6m-2.1m$ (Không gian đủ cho nhóm người lưu thông).  
  * **Pedestrian Density:**  
    * *Đông vừa*: Nhóm 3–4 người (2 người đi cạnh nhau trò chuyện \+ 1 người đi ngược lại).  
    * *Đông đúc / Hỗn loạn*: 5–9 người tạo dòng lưu thông ngược chiều hoặc phân tán.  
  * **Human Behavior Policy:** Nhóm có liên kết (duy trì khoảng cách nội bộ nhóm), người độc lập né tránh tự do.  
  * **Robot Action Policy:** `Advance` (tìm khoảng trống lách qua), `Yield` (chờ dòng người tản bớt), `Follow` (đi theo sau dòng người).  
* **3\. Execution & Metrics (Đo lường & Đánh giá):**  
  * **Labeling Criteria:** Có từ 3 người trở lên xuất hiện đồng thời trong bán kính quét $\leq 4m$ của robot.  
  * **Hard Constraints (Bắt buộc Pass 100%):** $C=0$ (Không đâm bất kỳ ai trong nhóm), $S=1$, $TO=0$, $FP=0$.  
  * **Quality & Social Metrics:**  
    * *Xã hội:* $PSC$ (Tuân thủ không gian cá nhân với toàn bộ các thành viên), Group Cohesion Violation (Không được cắt ngang giữa 2 người đang đi song song thành cặp).  
    * *An toàn & Kẹt:* Tỷ lệ đóng băng $ST$ (Stalled Time \- kiểm tra thuật toán có bị hiện tượng *Freezing Robot Problem* khi mật độ tăng không), $TT{C}_{min}$.  
    * *Động lực học:* $Jer{k}_{avg}$ (Mức độ dao động khi liên tục đổi hướng né nhiều người).  
  * **Failure Modes:** Tê liệt đứng im tại chỗ (Freezing Robot Problem), đâm người do các mô hình cục bộ triệt tiêu lẫn nhau, cắt đứt nhóm người đang đi chung.

---

### **TÓM TẮT QUY TẮC NGHIỆM THU ĐỒNG BỘ CHO CẢ 6 CARDS**

* **Quy tắc Episode Đạt (Pass):** Một episode chỉ được tính là thành công khi đạt **100% các tiêu chí Hard Constraints ($C=0,S=1,TO=0,FP=0$)**. Bất kỳ va chạm nào (kể cả va quẹt tường) đều bị loại bỏ ngay lập tức.  
* **Quy tắc Đánh giá Thuật toán (Benchmark Grade):** Khi episode đã Pass, các chỉ số **$PSC,Jerk,TTC,D{H}_{min},SPL$** sẽ được ghi log vào phân phối (Histogram) để so sánh xem giữa **ORCA, SFM, AVM và CSM**, thuật toán nào cho chuyển động tự nhiên nhất, ít giật cục nhất và ít xâm phạm không gian con người nhất.

---

### **PHẦN 2: THIẾT KẾ CARD THEO 3 GIAI ĐOẠN (PHASE EXECUTION CARDS)**

Đây là khung điều phối số episode và cách lọc dữ liệu từ Gazebo sang MuJoCo theo đúng lộ trình của bạn:

```
┌────────────────────────┐      ┌────────────────────────┐      ┌────────────────────────┐
│        VÒNG 0          │      │        VÒNG 1          │      │        VÒNG 2          │
│   (Sanity Check)       │ ───> │  (Benchmark So sánh)   │ ───> │ (Xác nhận & Khóa Data) │
│       48 eps           │      │        540 eps         │      │        900 eps         │
│  Lọc lỗi vật lý/code   │      │   Chấm điểm Metrics    │      │  Tăng Seed (Stat Sig)  │
└────────────────────────┘      └────────────────────────┘      └────────────────────────┘
```

#### **📌 CARD GIAI ĐOẠN 0: SANITY CHECK (48 Episodes)**

* **Mục tiêu:** Kiểm tra pipeline mô phỏng, đảm bảo code tích hợp của 4 thuật toán (ORCA, SFM, AVM, CSM từ JuPedSim) hoạt động ổn định, không bị crash code, không xuyên tường, không teleport.  
* **Công thức phân bổ:**  
* $4thuậttoán\times 6kịchbản\times 1cấuhìnhtiêuchuẩn(Easy,w=2.1m)\times 2seeds=48episodes$  
*   
* **Cấu hình Simulator:**  
  * Gazebo Headless (`gzserver` only, tắt GUI, tắt render LiDAR/Depth camera).  
  * Chỉ log **Ground Truth Odometry/Trajectory** ở tần số **10 Hz**.  
* **Tiêu chí ĐẠT để chuyển vòng:**  
  * $100%$ episodes hoàn thành mà **không crash ROS/Gazebo**.  
  * Tỷ lệ lỗi vật lý phi thực tế \= $0%$ (Người không bị rớt khỏi map, không bay lên trời).

---

#### **📌 CARD GIAI ĐOẠN 1: SO SÁNH CHUYỂN ĐỘNG & TƯƠNG TÁC (540 Episodes)**

* **Mục tiêu:** Đánh giá định lượng sâu toàn diện về động học, mức độ mượt mà, khả năng tương tác của 4 thuật toán trong các môi trường hẹp và so sánh phản ứng khi robot đổi hành vi.  
* **Công thức phân bổ (Tổng cộng 540 eps lũy kế):**  
  * **Nhánh A (Đánh giá chuyển động & tương tác tự nhiên):**  
  * $4thuậttoán\times 6kịchbản\times 3cấuhình(Easy/Critical/Hard)\times 5seeds=360episodes$  
  *   
  * **Nhánh B (Đối chứng hành vi Robot):**  
    * Chọn **3 kịch bản tương tác gắt nhất**: *Đối đầu, Cửa hẹp, Giao cắt*.  
    * Dùng cấu hình quan trọng nhất ($w=0.9m$ hoặc $1.2m$).  
    * Chạy thêm **3 hành động của robot** (`Stop`, `Yield/Turn`, `Reverse`) so với nhánh tiến (`Advance`) đã có ở Nhánh A:  
    * $4thuậttoán\times 3kịchbản\times 1cấuhình\times 3hànhđộngmới\times 5seeds=180episodes$  
    *   
* **Hệ thống Metrics kiểm định (Lấy từ `3700599.pdf`):**  
  * *An toàn & Thành công (Hard)*: Lập bảng tỷ lệ Thành công ($SR$), Tỷ lệ va chạm ($CR=HC+WC$), Tỷ lệ kẹt ($FP$).  
  * *Chuyển động (Motion Quality)*: Xuất biểu đồ phân phối (Histogram) của Vận tốc (${V}_{avg,max}$), Gia tốc (${A}_{max}$), và Độ giật ($Jer{k}_{max}$). *So sánh xem JuPedSim (SFM/CSM) có giảm hiện tượng dao động (jitter) tốt hơn ORCA không.*  
  * *Tương tác xã hội*: Tính $PSC$ (ngưỡng $0.5m$), khoảng cách gần nhất $D{H}_{min}$, và $TTC$.  
* **Đầu ra vòng 1:**  
  * Biểu đồ so sánh phân phối giữa 4 thuật toán.  
  * Lọc ra các kịch bản xuất hiện **Outliers** (những episode bị kẹt lâu, jerk vọt xà, hoặc suýt đâm nhau).

---

#### **📌 CARD GIAI ĐOẠN 2: XÁC NHẬN KẾT QUẢ & KHÓA DỮ LIỆU ĐỂ CHUYỂN SANG MUJOCO (900 Episodes)**

* **Mục tiêu:** Bổ sung seed để đạt ý nghĩa thống kê chuẩn (theo **Guideline B4** trong bài báo: tối thiểu 20–30 mẫu cho mỗi cấu hình trọng yếu), xác nhận các ca biên (edge cases) và chốt bộ dữ liệu chuẩn (Golden Dataset).  
* **Công thức phân bổ:**  
  * Bổ sung thêm **15 seeds** cho các cấu hình trọng yếu (đưa tổng số seed từ 5 lên 20):  
  * $4thuậttoán\times 6kịchbản\times CấuhìnhCritical\times 15seedsbổsung=360episodes(mới)$  
  *   
  * $Tổnglũykếtoànbộthựcnghiệm=540+360=900episodes$  
  *   
* **Tiêu chí ĐẠT của Bộ Dữ Liệu:**  
  * Đạt đủ độ phủ 6 kịch bản chuẩn (Guideline N5 & Table 3).  
  * Mẫu thống kê đủ tin cậy (Confidence Interval $\geq 95%$).  
  * Toàn bộ file log quỹ đạo (Trajectory formats) được chuẩn hóa sẵn sàng để đem sang làm ground truth đối chiếu khi chuyển môi trường sang **MuJoCo**.

---

### **PHẦN 3: FILE CẤU HÌNH GỢI Ý ĐỂ LOAD VÀO PIPELINE (CARD SCHEMA DẠNG YAML)**

Bạn có thể lưu cấu trúc Card này thành file `scenario_card_config.yaml` để nạp trực tiếp vào code quản lý episode của ROS/Gazebo:

```
card_version: "1.0-IEEE-THRI"
benchmark_name: "Social_Nav_Narrow_Space"

phase_control:
  current_phase: 1 # Chuyển 0, 1, 2
  sample_rate_hz: 10
  timeout_nominal_sec: 30

algorithms:
  - name: "ORCA_pure"
  - name: "SFM_JuPedSim"
  - name: "AVM_JuPedSim"
  - name: "CSM_JuPedSim"

scenarios:
  - id: "SC1_Frontal"
    env: "corridor"
    widths: [2.1, 1.2, 0.9] # Easy, Critical, Hard
    density_levels:
    human_policies: ["cooperative", "neutral", "non_cooperative"]
    robot_actions: ["advance", "stop", "yield", "reverse"]
  - id: "SC2_Doorway"
    env: "hospital"
    widths: [1.2, 0.9, 0.6]
    density_levels:
    human_policies: ["cooperative", "non_cooperative"]
    robot_actions: ["advance", "stop", "yield"]
  # (Tương tự cho Blind_Corner, Intersection, Overtaking, Group)

evaluation_gates:
  hard_constraints:
    max_collisions: 0
    min_success_rate: 1.0 # Bắt buộc trong từng ep
    max_progress_freeze_sec: 5.0
  quality_thresholds:
    personal_space_radius_m: 0.5
    min_ttc_sec: 1.5
    max_acceptable_jerk: 2.5 # m/s^3
```

Khung Card này đáp ứng đầy đủ yêu cầu: **khoa học theo chuẩn `3700599.pdf`**, **tinh gọn (chỉ 540 – 900 episode thay vì 64.800)**, và **tách bạch rõ ràng từng biến độc lập** để bạn xử lý dứt điểm tình trạng dữ liệu hỗn hợp hiện tại.