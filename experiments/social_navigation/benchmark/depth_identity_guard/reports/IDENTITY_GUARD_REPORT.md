# Causal identity guard — kết quả 23/10/2026

**Đã xử lý case đổi đối ứng GT đã biết, giữ coverage và vượt tiêu chí bảo vệ ID trên regression; seed44 mới không suy giảm. Chưa nghiệm thu semantic detector, bộ train hoặc navigation.** Code thử nằm riêng trong experiment, không thay tracker/ROS/Nav2 mặc định. [Acceptance](acceptance.json), [audit](audit01/audit.json), [code và lệnh chạy](../README.md).

## Vấn đề và bản sửa cuối

Ở seed43, cùng track5 đổi ghép hình học từ GT person5 tại 13.5 s sang GT person6 tại 13.9 s. Một cụm bề mặt mới xuất hiện sau vài frame thiếu quan sát có thể nhận ID cũ vì gate residual đã mở rộng theo dt. Hungarian assignment cũng có thể ép một track cũ nhận lựa chọn còn lại khi lựa chọn đúng đã có track khác cạnh tranh. Đây là diagnostic theo simulator center/gate, chưa phải identity-switch truth đã gán nhãn semantic/visibility thủ công.

Bản candidate05 giữ nguyên detector hình học/shape filter và dormant parameters của parent. Phần active association đổi như sau:

- Ghép các quan sát liên tục rõ ràng trước; không cho track mất quan sát tranh lại proposal đã được ghép chắc.
- Sau gap >150 ms, cắt link nếu **cả** dịch chuyển và descriptor shape không phù hợp. Một cue riêng lẻ có thể phản ánh chỉ nhìn thấy một phần cơ thể. Giữ các threshold đã ghi trong config, không sweep theo seed mới.
- Với các link gap còn lại, ghép mutual-nearest có margin theo từng vòng, loại các row/column đã ghép rõ trước khi xét tiếp. Lựa chọn đã được track khác nhận không còn là đối thủ giả làm đứt ID.
- Nếu các track đã ổn định thực sự tranh cùng một proposal, giữ proposal đo nhưng để ID chưa rõ. Không ép identity, không phát ghost/prediction như observation và không backfill bằng dữ liệu tương lai.

`track_id=-1` đi kèm proposal_index riêng, confirmed=false, identity_observed=false, velocity_valid=false; nhiều proposal không rõ ID không được gom thành một người. confirmed chỉ là track đủ hits, không chứng nhận đó là người. Các stream giữ semantic_class=unknown_geometry_candidate, không nạp GT/persona/backend vào tracker.

## Quá trình phát triển được giữ lại

| Candidate | Thay đổi chính | Kết quả regression |
|---|---|---|
| 01 | Guard motion/shape và mutual matching rộng | Chặn case43 nhưng tăng fragmentation hai lượt42 |
| 02 | Thu hẹp guard vào gap, giữ đường liên tục | Còn fragmentation tăng |
| 03 | Yêu cầu hai cue cùng bất thường; competitor phải thật sự tranh cùng proposal | repeat42 đạt; mesh42 còn tăng 1 |
| 04 | Ưu tiên continuous trước gap | repeat42 cải thiện; mesh42 vẫn tăng 1 |
| **05** | Giải quyết mutual pairs theo vòng, bỏ alternative đã được nhận | Ba regression mesh đạt |

Code/config/hash và mọi kết quả các bản chưa đạt ở [candidates](candidates/). Không xoá fail để chỉ giữ bản đẹp. Những sửa đổi này chỉ dùng seed42/43 đã biết; seed43 chính thức là development/regression. Sau khi chọn05, [final engine](final_engine/frozen.json) ghi hash trước khi thu seed44. Không đổi algorithm/config/gate sau khi xem seed44.

## Tiêu chí của lượt sửa ID

Mục tiêu lần này là tránh đổi identity khi thiếu/ghép mơ hồ mà không mất coverage. Gate identity-safety được khai báo trước các phép thử: paired rows ≥95% parent; zero short-gap GT-pair changes; fragmentation proxy không tăng so parent; không có known-wrong dormant reacquisition.

**Đây là tiêu chí khác với gate đầy đủ cũ**, vốn yêu cầu giảm fragmentation so baseline unfiltered. Gate cũ vẫn tính bằng evaluator byte-identical và báo riêng. Trên regression43 và seed44, gate đầy đủ vẫn FAIL vì fragmentation không giảm; không gọi chúng thành nghiệm thu đầy đủ. Semantic/person visibility và train eligibility luôn false.

## Kết quả trên bản cuối

| Recording / vai trò | Paired rows parent→guard | Short-gap pairing changes | Fragmentation proxy | Identity-safety |
|---|---:|---:|---:|---|
| Mesh42 / regression | 2579→2579 | 0→0 | 10→10 | PASS |
| Repeat42 / regression, navigation cũ TIMEOUT | 2801→2801 | 0→0 | 8→7 | PASS |
| Seed43 / regression | 1983→1983 | **1→0** | 9→9 | PASS |
| Seed44 / mới, code đã chốt | 2587→2587 | 0→0 | 7→7 | PASS không suy giảm |

Trên cả bốn lượt mesh, giữ 100% paired rows. [Trace case](regression_identity_trace.png) thể hiện ID5 cũ được để unobserved khi không đủ cơ sở tiếp tục, thay vì nhận sang GT6; các track khác vẫn giữ coverage. Không nội suy để lấp khoảng trống trong trace. [Dữ liệu LABEL ONLY](regression_case_LABEL_ONLY.json).

**Seed44 không chứng minh cải thiện hơn parent**: parent cũng không gặp pairing change ở đó, cả hai có coverage và fragmentation giống nhau. Lượt này kiểm việc bản sửa không làm xấu một seed chưa dùng, trong cùng scene/layout/backend. Chưa có bằng chứng cross-layout hoặc cross-simulator. Tỉ lệ ghép GT là phép đối chiếu hình học, không phải accuracy/precision/recall của person detector; fragmentation proxy không phải IDF1/MOTA có nhãn visibility.

Proxy stress cũng chạy lại: parent/guard đều paired603, fragmentation3, không đổi pairing. Tuy nhiên shape filter trước đó loại nhiều người hình cylinder; failure với dữ liệu proxy cũ **vẫn giữ nguyên**. Identity-only PASS so cùng filtered parent không chữa hoặc chứng nhận compatibility/person recall của profile đó. Không dùng HuNavSim.

## Lượt seed44 mới và tính toàn vẹn

Thu đúng một lần, giữ world, native motion, camera mesh OFF + ON diagnostic, robot và Nav2 moderate; chỉ đổi seed input thành44 và vị trí output runtime. [Driver/diff/manifest](seed44/driver/manifest.json) cùng preflight xác nhận engine đã chốt, native friction4/4, camera SHA14c154… và world bằng parent sau chuẩn hóa socket do phiên sở hữu.

[Capture audit](seed44/capture_audit.json) PASS: actual Ogre NVIDIA RTX4060 D3D12, callback balanced, recorder đóng sạch, pause/stop và cleanup đạt, không adapter fault. Physics/Nav2 vẫn dùng CPU cho phần việc của chúng; không tuyên bố toàn bộ pipeline chạy GPU. GUI không được mở; camera observer thực vẫn ghi media.

| Thu/chuẩn hóa seed44 | Kết quả |
|---|---|
| Raw depth | **1023/1023** ảnh float32 khớp MCAP bitwise |
| Cửa sổ | 801/850; 49 bị loại theo contract; chỉ 1 episode mới |
| Future labels/action audit | XY max error ~7.55e−15 m; executed action resampling exact |
| Navigation | **TIMEOUT**, 90.04 sim s, actual goal còn **4.012 m**, yaw1.062rad |
| Label tail | 3.345 sim s |
| Seed diversity | Quỹ đạo người khác seed43 từ native step39; coupled robot inputs cũng khác |

Data format PASS không chuyển TIMEOUT thành navigation success. Giữ persona−1/valid=false và mask association/train false. `raw_audit` ghi âm đồng hồ bảo thủ bằng max(header,receive-clock), không đo wall-clock camera latency. Dataset lớn là dữ liệu thử/validation local, chưa phải bộ train.

## Audit và giới hạn bằng chứng

[Audit độc lập](audit01/audit.json) PASS: kiểm42596 khoảng cách GT qua bốn variants trên năm recording; replay1000 frame prefix với odom tương lai bị loại, gồm cả counterexample13.x; parent streams tái lập chính xác các stage cũ. Xác nhận source hash của32 file engine archived, nguồn raw seed44 không đổi và code/gate đã chốt trước holdout.

Mọi proposal của detector được giữ, parent/guard có cùng số đo và vị trí từng proposal. **Không có proposal ID−1 trong năm stream bản cuối thực tế**; đường abstain được kiểm bằng synthetic behavior tests, chưa được exercised trên các recording này. Vì vậy không giải thích việc giảm pairing change bằng việc che mất detection. [14 behavior tests](behavior_tests.json) được thực thi trước freeze, bao gồm late/clock, ego motion, ambiguity, gap motion/shape, partial visibility và alternative đã bị claim. Các test không phải chứng nhận semantic.

Hash/source/QA/evaluation/media nhỏ được lưu Git; raw MCAP/NPY/JSONL lớn ở generated bị Gitignore, không được backup bằng Git. Các PNG/GIF từ dữ liệu ghi thật, không AI. Hình comparison dùng midpoint cố định: regression43 frame525 và seed44 frame511, không chọn theo chất lượng. GT chỉ nằm trong nhánh label/evaluation/plot. World Model chưa được train.

## Quyết định và bước tiếp

Đóng gói prototype05 cùng regression suite, giữ mặc định project cũ. Có thể dùng nó làm baseline thử nghiệm để phát triển dataset, nhưng chưa bật association supervision của M1 chỉ vì gần GT theo gate. Cần nhãn visibility/person instance, metric semantic và kiểm người/vật thể/occlusion trước batch; H1–H5 vẫn cần xác thực theo cơ chế hành vi riêng. Lỗi navigation TIMEOUT tiếp tục là hạng mục khác chưa được sửa trong lượt tracking này. HuNavSim giữ test cuối.
