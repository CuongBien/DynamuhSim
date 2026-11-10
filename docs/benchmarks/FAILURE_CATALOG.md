# Danh mục lỗi benchmark Human Navigation

Mẫu số là 900 lượt hợp lệ về hạ tầng. Một lượt có thể thuộc nhiều nhóm. Bảng máy đọc ghi backend, Card, level, seed, đường dẫn attempt, thời điểm contact và overlap đầu tiên, tiến độ từng người, số substep guard và lý do trượt ND. JSON thống kê tần suất theo backend, Card/level và seed. [Danh mục từng lượt](../../runs/task_08f/failure_catalog.csv) · [Tần suất theo tầng](../../runs/task_08f/statistical_summary.json)

| Nhóm | Lượt / 900 | Bằng chứng và độ chắc chắn |
| --- | ---: | --- |
| Contact solver robot–người | 277 | Có 3.413 mẫu tại substep, gộp thành 280 đợt. Tất cả 277 lượt kết thúc vì contact. Sự kiện xác nhận tương tác vật lý, còn nguyên nhân phân bổ giữa hai actor chưa được cô lập. |
| Overlap hình học robot–người, không có solver contact | 80 | Khoảng hở đĩa quy ước âm ở trace 50 ms. Cả 80 robot tới đích và trượt an toàn. Hình học solver dùng geom MJCF. Độ lệch giữa hai phép đo giữ riêng. |
| Contact solver robot–tường | 0 | Kênh quan sát được. |
| Robot chưa đạt mục tiêu | 277 | Cả 277 lượt kết thúc theo contact. Timeout và FP robot đều bằng 0. |
| Ít nhất một mục tiêu người chưa được quan sát là đạt | 513 | Cửa sổ quan sát kết thúc cùng robot. 544/1.420 mục tiêu người được quan sát là đạt. |
| Người `NO_PROGRESS` theo phân loại benchmark | 107 | Người chưa tới đích và có quãng thời gian trên 5 s không cải thiện khoảng cách tốt nhất, trong lượt an toàn. Nếu đếm điều kiện người chưa đạt và ngưng cải thiện trên 5 s cả trong lượt trượt an toàn, có 134 lượt. |
| Nhãn Card trượt | 85 | Tất cả ND. Có 49 robot chưa qua, 30 người chưa qua và sáu cả hai chưa qua. Tất cả có dữ liệu để chấm nhãn. |
| Guard giữ root người | 235 | 134.561 substep. Đây là can thiệp thực thi quan sát được, có thể trùng contact hoặc `NO_PROGRESS`. |
| Lỗi đồng bộ runtime hoặc solver sample chưa phân loại | 0 | Không có lỗi được hợp đồng runtime ghi nhận. Kênh solver người–người/người–tường giữ null theo mask. |
| Deadlock người–người, deadlock hai chiều robot–người và reciprocal dance | Chưa phân loại | Giao thức có FP của robot và freeze tiến độ từng người. Thiếu tiêu chí và nhãn nhân quả để đếm riêng từng kiểu deadlock/dance. Các lượt thiếu tiến độ và quỹ đạo gốc vẫn có thể truy vết. |

## Ca đại diện

- **AVM ND Critical Advance, seed 1005, `e0909263dcb34fc591992c2141f2314a:0`.** Người qua cửa ở 3,182 s. Khoảng hở robot–người âm từ mẫu 4,35 s, solver contact ở 4,388 s và robot còn cách đích 3,26 m. Có 212 substep guard. Kết thúc vì contact làm robot chưa qua cửa. [Attempt gốc](../../runs/task_08d/benchmark_study/attempts/c6a8af7aa7e356e392fec58a5dd5797ca629a01f777e281740b382b9a74945aa/attempt-0001/)
- **ORCA ND Easy Advance, seed 1000, `9f3f85f4856546d79400a8b118a05bcd:0`.** Robot tới đích. Ở 6,20 s, khoảng hở nhỏ nhất −0,0595 mm và không có mẫu solver contact. Dấu overlap tái lập trong replay. Có thể xác nhận chênh giữa hai phép đo. Nguyên nhân ở đúng substep chưa quan sát được từ trace 50 ms. [Attempt gốc](../../runs/task_08d/benchmark_study/attempts/49a8cac62b4b95b74cbf57370449bf47a35809023315d695e313d73fc8f57e64/attempt-0001/)
- **CSM V2 ND Critical Advance, seed 1010, `dd35615e4e44446fb189b06c728be501:0`.** Người qua ở 3,169 s và robot qua ở 5,622 s. Robot đạt an toàn. Người di chuyển tổng 7,06 m, còn cách đích 2,46 m và có 6,20 s không cải thiện khoảng cách tốt nhất. Số đo xác nhận thiếu tiến độ dù vẫn có chuyển động. [Attempt gốc](../../runs/task_08d/benchmark_study/attempts/acbbbf95478a0cabb0ca0b7751f8f427fd81e5075b9292ebe5cd375f77a33ddb/attempt-0001/)
- **SFM ND Critical Advance, seed 1011, `ec2d115960ae4308b690cb54fb31d947:0`.** Robot qua cửa và đạt an toàn. Người chưa cắt khẩu độ và còn cách đích 6,45 m lúc kết thúc. Nhãn trượt phản ánh thiếu lượt qua của người trong thời gian quan sát. [Attempt gốc](../../runs/task_08d/benchmark_study/attempts/3615dec96106549e391d3eb03e55f6bac8031afd7504eb10d21867b8f8a7d4ac/attempt-0001/)
- **ORCA MG Critical Advance, seed 1000, `3378d0a81af445d7b6da570be5982b34:0`.** Có 343 substep guard, đỉnh jerk người 650,82 m/s³ ngay sau tick giữ pose làm tốc độ về 0. Bốn trong năm người tới đích. Robot tới đích nhưng khoảng hở robot–người thấp nhất −0,0340 m. Dấu vết cho thấy tác động trực tiếp của guard lên jerk đo được ở ca này. [Attempt gốc](../../runs/task_08d/benchmark_study/attempts/5fe7b196bb56914eae017d4918a60c10fed3eb41237cee3663a1b3bda0cd16d5/attempt-0001/)

Guard kinematic giữ root mocap, contact masks giới hạn các loại cặp solver và bridge 2D–3D dùng robot external đồng bộ tại nhịp người. Các giới hạn này là một phần của protocol. Với ND, thiết kế follow-up có thể thay độ lệch tiếp cận và thời gian quan sát sau khi robot tới đích trong một protocol riêng. [Giao thức QA](DATA_QA_PROTOCOL.md) · [Phạm vi quan sát và contact](../architecture/05_MUJOCO_RUNTIME.md#quan-sát-và-contact)
