# TASK 08F · Nghiệm thu benchmark Human Navigation

**`CLOSED_WITH_SCIENTIFIC_LIMITATIONS`.** Có thể kết thúc Human Models và chuyển sang TASK 09. Benchmark 900 lượt đạt cổng toàn vẹn và tái lập trên snapshot đã khóa. Các tầng ND thiếu đối chứng theo nhãn vẫn là hạn chế nghiên cứu. [Manifest phát hành](../../../runs/task_08f/release_manifest.json) · [Hợp đồng TASK 09](../../handoff/TASK_09_ENTRY_CONTRACT.md)

## Cổng dữ liệu

| Đại lượng | Kết quả |
| --- | ---: |
| Slot kế hoạch Vòng 0 / 1 / 2 lũy kế | 48 / 540 / 900 |
| Attempt / episode hợp lệ / execution key duy nhất | 900 / 900 / 900 |
| Giữ lại từ Vòng 0–1 / bổ sung ở Vòng 2 | 540 / 360 |
| Lỗi hạ tầng / gián đoạn / retry | 0 / 0 / 0 |
| Đạt nhiệm vụ an toàn / trượt an toàn | 543 / 357 |
| Nhãn đạt / trượt / không quan sát được | 815 / 85 / 0 |

Kiểm lại từng attempt từ raw, gồm mã định danh, config, MJCF, asset, seed stream, pose reset sau dựng scene, hạn chế lệnh và ba tầng requested/limited/executed. Có 97.855 mẫu GT 10 Hz, 194.317 tick điều khiển 50 ms, human callback 100 ms và đồng hồ 4.857.925 substep 2 ms. GT và các kênh lệnh khớp roster, frame và timestamp. Sai phân pose khớp executed, với sai số lớn nhất ở người bằng 0 m/s. Khoảng hở người–tường và người–người không âm ở các tick áp dụng. Đối chiếu log solver theo substep riêng với clearance trên trace. Robot chạy lịch vòng hở theo timestamp hiện tại. [QA 900 lượt](../../../runs/task_08e2/report/episode_audit.csv) · [Inventory và đường dẫn raw](../../../runs/task_08f/episode_inventory.csv) · [Audit đọc lại raw](../../../runs/task_08f/audit.log)

19 tệp báo cáo/QA được tính lại vào thư mục tạm và khớp từng byte với 08E-2. Kiểm tra chéo 107 tầng dữ liệu và KTC Wilson từ kết quả gốc. Hash của cả 9.004 tệp nghiên cứu gốc giữ nguyên trước và sau audit/replay. [Log kiểm tra thống kê](../../../runs/task_08f/statistical_audit.log) · [Hash gốc](../../../runs/task_08f/raw_before.sha256)

## Kết quả backend và thống kê

SR là robot tới đích. CR là có contact solver quan sát được. Đạt an toàn còn yêu cầu clearance và các hard constraints. Bảng dùng tập hợp lệ thô, giữ các lượt collision và trượt nhãn trong mẫu số. KTC là Wilson 95%.

| Backend | SR, số lượt và KTC % | CR, số lượt và KTC % | Đạt an toàn, số lượt và KTC % | Nhãn đạt |
| --- | --- | --- | --- | ---: |
| ORCA | 186/225 [77,2–87,1] | 39/225 [12,9–22,8] | 126/225 [49,5–62,3] | 218/225 |
| SFM | 165/225 [67,2–78,7] | 60/225 [21,3–32,8] | 158/225 [63,9–75,8] | 180/225 |
| AVM | 91/225 [34,2–47,0] | 134/225 [53,0–65,8] | 82/225 [30,4–42,9] | 197/225 |
| CSM V2 | 181/225 [74,8–85,1] | 44/225 [14,9–25,2] | 177/225 [72,9–83,5] | 220/225 |
| Chung | 623/900 [66,1–72,2] | 277/900 [27,8–33,9] | 543/900 [57,1–63,5] | 815/900 |

Trong tập 815 lượt đạt nhãn, SR 595/815, CR 220/815 và đạt an toàn 518/815. Tập ghép cặp dùng 1.151/1.350 cặp đủ hai nhãn. Có 532 cặp đủ thêm hard constraints để so chất lượng chuyển động, với mẫu số riêng theo metric. Loại 199 cặp theo nhãn và thêm 619 cặp theo hard constraints. KTC ghép cặp dùng 2.000 lượt bootstrap cụm Card/seed, RNG seed 812. [Tử số, mẫu số và KTC](../../../runs/task_08f/statistical_summary.json) · [Ghép cặp và lý do loại](../../../runs/task_08e2/report/paired_backend_audit.csv)

Trong Critical Advance, CSM V2 cao hơn ORCA **22,5 điểm phần trăm** về đạt an toàn trên 120 cặp (KTC +15,0 đến +30,0). Trên 72 cặp cùng đạt an toàn, đỉnh jerk người của CSM V2 cao hơn 29,33 m/s³ (KTC +10,55 đến +47,93). Chênh lệch đạt an toàn CSM V2–SFM là +2,0 điểm phần trăm (KTC −11,0 đến +14,0, 100 cặp). Khoảng này chứa các hiệu ứng theo cả hai hướng, và 20 cặp ND của so sánh này thiếu nhãn. [KTC theo phạm vi và metric](../../../runs/task_08e2/report/paired_backend_summary.csv)

Trong tập đạt nhãn và hard constraints, trung bình PSC theo ORCA, SFM, AVM, CSM V2 lần lượt là 3,88%, 1,78%, 0,11% và 1,24%, với mẫu số 126, 133, 82 và 177. Bảng chi tiết giữ mean/max tốc độ, gia tốc, jerk, stall time, path length, clearance, TTC, encounter, tiến độ và số đo thiếu. Các tập đủ điều kiện của từng backend có thành phần khác nhau. Histogram dùng cùng định nghĩa bin đã khóa. SPL thiếu chiều dài tham chiếu ở 900 lượt. TTC có 842 số đo hữu hạn và 58 giá trị rỗng với trạng thái `no_predicted_contact`. Cohesion và realism thiếu bằng chứng đánh giá độc lập. [Kết quả từng tập theo backend](../../../runs/task_08f/backend_comparison.csv) · [Phân phối](../../../runs/task_08e2/report/motion_distributions.csv)

## Độ phủ sáu Card

| Card | Kế hoạch / hợp lệ | Nhãn đạt / trượt / null | An toàn / trượt an toàn |
| --- | ---: | ---: | ---: |
| FA | 180 / 180 | 180 / 0 / 0 | 90 / 90 |
| ND | 180 / 180 | 95 / 85 / 0 | 95 / 85 |
| BC | 120 / 120 | 120 / 0 / 0 | 58 / 62 |
| IN | 180 / 180 | 180 / 0 / 0 | 152 / 28 |
| OT | 120 / 120 | 120 / 0 / 0 | 114 / 6 |
| MG | 120 / 120 | 120 / 0 / 0 | 34 / 86 |

Critical Advance có 20 seed mỗi backend × Card. Tỷ lệ an toàn mỗi seed là 13–16/24 slot. Theo Card, an toàn Critical Advance dao động từ MG 21/80 tới OT 80/80. Các tầng còn lại gồm Easy, Hard, action, mật độ và subtype, với OT Critical phân bổ 10/10 cho hai subtype ở từng backend. [Độ phủ theo tầng](../../../runs/task_08e2/report/coverage_breakdown.csv) · [Critical 20 seed](../../../runs/task_08e2/report/critical_seed_coverage.csv)

Raw xác nhận Vòng 1 có 56 nhãn ND trượt, Vòng 2 thêm 29 và ND Hard vẫn **0/20**. ND Critical Advance đạt **41/80**, gồm ORCA 20/20, CSM V2 20/20, AVM 1/20 và SFM 0/20. Cả 180 lượt ND có mẫu đồng thời trong vùng 3 m. Trong 85 lượt trượt, 49 robot chưa cắt cửa, 30 người chưa cắt và sáu lượt cả hai chưa cắt. Pose, route danh nghĩa, timeline và nhãn quan sát được qua kiểm định. ND Hard thiếu toàn bộ so sánh theo nhãn. ND Critical thiếu cặp có SFM, còn AVM chỉ có một cụm seed. [Dấu vết qua cửa theo lượt](../../../runs/task_08e2/report/nd_doorway_qa.csv) · [Phân tích hình học và follow-up ND](TASK_08E2_REPORT.md#kiểm-định-riêng-card-nd)

## Đối chứng hành động

180/180 cặp Branch B khớp Advance reference, common prefix và xác nhận lệnh thực thi đã đổi. Hai bảng hành động tính lại từ dữ liệu gốc khớp byte với Vòng 1. Có 150 cặp nhiệm vụ và 83 cặp chuyển động. Trong mỗi nhóm 60, 10 cặp bị loại theo nhãn, với năm trường hợp trượt cả hai vế. Sau lọc hard constraints còn 26 Stop, 31 Yield/Turn và 26 Reverse.

| Hành động so với Advance | Cặp nhiệm vụ / chuyển động | Chênh lệch CR, điểm % [KTC 95%] | Trung vị lệch quỹ đạo người |
| --- | ---: | ---: | ---: |
| Stop | 50 / 26 | −6,7 [−11,7 đến −1,7] | 0,002 m |
| Yield/Turn | 50 / 31 | −23,3 [−35,0 đến −13,3] | 0,030 m |
| Reverse | 50 / 26 | −6,7 [−11,7 đến −1,7] | 0,002 m |

Hiệu ứng CR lấy trung bình theo cụm Card/seed trên cặp có nhãn. Độ lệch quỹ đạo tính trong cửa sổ chung trên các cặp đạt an toàn. KTC cho chênh lệch đạt an toàn của cả ba hành động đều chứa 0. Cùng một Advance có thể xuất hiện trong nhiều đối chứng. [Từng cặp hành động](../../../runs/task_08e1/report/action_contrasts.csv) · [Hiệu ứng theo backend/Card](../../../runs/task_08e1/report/action_contrast_summary.csv)

## Kết luận và tệp bàn giao

AVM giữ `PARTIAL`. CSM V2 giữ lỗi hành vi, gồm 53 lượt người `NO_PROGRESS` dù robot đạt an toàn. ORCA và SFM cũng giữ các lượt contact, overlap hoặc chưa hoàn tất mục tiêu. Kết quả mô tả từng trục an toàn, chuyển động, tương tác và độ phủ trong phạm vi đã đo. [Danh mục lỗi và ca đại diện](../../benchmarks/FAILURE_CATALOG.md) · [Giới hạn máy đọc](../../../runs/task_08f/limitations.json)

Replay độc lập 9/9 ca, gồm bốn backend, sáu Card và cả ba phase gốc. Khôi phục source bundle và cài các binary lưu trữ trong venv mới cho dry-run 900/900. Suite chọn lọc trên snapshot có 46 pass, 2 skip stock/04S, exit 0. [Gói tái lập, lệnh và exit code](../../benchmarks/REPRODUCIBILITY_PACKAGE.md) · [Kết quả replay](../../../runs/task_08f/replay/replay_results.json)

Tệp mới trong repository gồm báo cáo này, danh mục lỗi, tài liệu tái lập, hợp đồng TASK 09 và script kiểm chứng. Các đầu ra máy đọc, archive, checksum và replay nằm dưới `runs/task_08f/`. Đề xuất tag `human-nav-benchmark-v1.0.0`. [Checksum](../../../runs/task_08f/artifact_checksums.sha256) · [Verifier](../../../scripts/benchmarks/verify_release.py) · [Manifest và chỉ mục artifact](../../../runs/task_08f/release_manifest.json)
