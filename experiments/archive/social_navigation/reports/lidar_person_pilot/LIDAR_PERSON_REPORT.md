# Thử detector người pretrained trên LiDAR — 23/10/2026

**DR-SPAAM chạy được trên GPU nhưng chưa phù hợp để bật nhận diện người với profile mô phỏng hiện tại.** Hai gate có người/không người đều FAIL vì gán hỗ trợ người vào đồ vật. Giữ input LiDAR + depth theo lựa chọn của người dùng, không thêm RGB. [Ảnh full](full01/midpoint/semantic_review.png), [ảnh không người](null01/midpoint/semantic_review.png), [acceptance](acceptance.json), [audit](audit.json), [code/cách chạy](../README.md).

## Đã triển khai

Dùng [code gốc của nhóm nghiên cứu](https://github.com/RWTHVision/2D_lidar_person_detection) và checkpoint JRDB ann từ model-zoo do tác giả liên kết; không train/fine-tune model ở đây. Pin commit/checkpoint SHA ở upstream.json. Runtime Python3.12.3 riêng, PyTorch2.7.1+cu128, CUDA12.8, actual parameter_device=cuda:0 trên NVIDIA RTX4060. Không sửa environment/ROS/SocNavGym cũ. Upstream có ROS1 node; thí nghiệm này gọi API Python độc lập, không gọi là hỗ trợ Jazzy node đã tích hợp.

Chỉ sửa tương thích np.int→int và torch.load(weights_only=True,map_location=cpu) trong bản copy. Kiểm parity với nguồn gốc:88state tensors bằng nhau,60312giá trị cutout bitwise,3chuỗi synthetic có sai số output0. Smoke cho thấy CUDA chạy; không dùng synthetic để công bố detection quality.

Sau đó thu đúng hai điều kiện cùng fixture:15camera×5timestamp mỗi điều kiện, tổng150depth thật và40scan native (20mỗi điều kiện). Renderer actual Ogre NVIDIA; server exit0, pause/stable/cleanup đạt cả hai. Camera thấp/intrinsics/clip, pose người/đồ vật dựa fixture tĩnh trước. Control null chỉ bỏ người và proxy, giữ hộp/nhà xưởng/cảm biến. Không có Nav2 hoặc native human stepping trong lượt này.

## Contract input

Input model chỉ scan native, không mesh/pose/ID/label GT. Sensor original360rays/0–2pi inclusive/3.5m/5Hz/noise1cm được giữ. Bỏ tia cuối trùng0°/360°, reorder cyclic359tia sẵn có vào grid đối xứng; không nội suy/tăng mật độ hoặc sửa khoảng đo. Model dùng FOV358.9972° khớp grid thật; tọa độ từ cos/sin góc chuẩn, không fit hoặc flip y theo GT. Zero/NaN/Inf→29.99 theo node tác giả, giữ gap/mất thông tin do tầm3.5m. Panoramic=true, stride1, confidence0.5 chốt trước dữ liệu. Không mang trạng thái recurrent từ full sang null hoặc từ synthetic sang scene.

Nhánh cảm biến dùng LiDAR để corroborate cụm depth bằng mutual nearest duy nhất, gate0.6m và margin0.15m. Không nhận ra không đồng nghĩa vật thể: mọi proposal vẫn giữ và unknown nếu không đủ bằng chứng. Positive vẫn experimental và semantic_valid=false. Nhánh LABEL mới đọc15depthcounterfactual để tính owner người thật trong ảnh; không truyền nhãn đó vào inference.

Với mỗi depth, chỉ lấy scan stamp≤depth stamp, available≤depth available, tuổi≤250ms. Available=max(header,latest receive sim-clock) là contract log, chưa đo wall latency hoặc xác định callback order trong cùng tick. t2s thiếu scan hợp lệ vì warmup; t3/4/5/6s có scan. Không gọi cold start này là model bỏ sót, và vẫn báo gate tổng nguyên bản FAIL.

## Kết quả

| Thời điểm / điều kiện | Cụm người được hỗ trợ đúng | Cụm đồ vật được hỗ trợ sai |
|---|---:|---:|
|Full3s|0|2|
|Full4s|0|1|
|Full5s|0|1|
|Full6s|1|0|
|Null3s|0|0|
|Null4/5/6s, mỗi thời điểm|0|1|

Full có5supported proposal rows:1người+4đồ vật trong4frame khả dụng; null có3false supported rows. Đây là các quan sát lặp của một fixture, không phải5người độc lập hoặc precision/recall tổng quát. Bốn người thấy trong depth vẫn có proposal nhưng phần lớn unknown; không xoá chúng để đạt precision. Learned detections không ghép được cũng được giữ trong JSONL, không bị che khỏi bằng chứng. Gate trước thử đòi zero hỗ trợ đồ vật và≥3/4người/frame full; **cả full/null FAIL**, kể cả khi chỉ xét4frame khả dụng.

Median warm inference native gồm preprocess/network/postprocess khoảng7.44ms full/7.36ms null, bỏ2lượt đầu khỏi median; cold first khoảng254/260ms. Đây là latency offline40scan, không phải Gazebo FPS hoặc tốc độ pipeline thực tế. Tensor allocation CUDA peak54.47MB không phải toàn bộ VRAM. Rendering và neural inference thực sự dùng GPU; transport/physics/analysis vẫn có phần CPU.

Native LiDAR hiện nhìn proxy cylinder, camera nhìn mesh; sampling/tầm/height cũng khác dữ liệu pretrained. Đây là domain/profile mismatch đáng nghi, **chưa có ablation chứng minh nguyên nhân duy nhất là proxy**. Không thay proxy, nâng camera, sửa threshold/NMS hoặc geometry để làm đẹp kết quả trong lượt này.

## Audit, lỗi và lưu trữ

[Audit độc lập](audit.json):14360ray geometry checks,6scan prefix thực replay khớp,10lựa chọn thời gian kiểm lại; prediction/histogram/count/label đúng,15102file có sẵn giữ nguyên. Nhãn pixel và full/control depth khớp cùng timestamp; source/weight hashes đầy đủ. **Integrity PASS không phải semantic PASS.** 9contract tests chạy trước capture.

Setup venv đầu thiếu ensurepip: dùng venv --without-pip và pip --python từ environment có sẵn để chỉ cài CUDA vào venv mới, không apt/system changes. Shared system/user packages được đọc lại, runtime không phải hermetic container. Lần infer đầu đã tính nhưng lỗi serialize NumPy bool trước ghi metric; giữ failure. Chỉ sửa JSON scalar conversion sau capture/trước đánh giá chất lượng; archive engine01, frozen manifest và diff còn nguyên. Không tuyên bố mọi source byte-identical sau capture; inference math/model/threshold/fusion/gate không đổi. Bản02 chạy lại cùng raw, không thu thay data.

Git lưu mã gốc/copy/license, manifest/checkpoint URL+SHA, CUDA lock, native scan arrays, raw prediction JSONL, LABEL ONLY representative, báo cáo/ảnh thật. Raw full-depth/CUDAenv/weights lớn ở generated và không backup bằng Git. PNG là từ dữ liệu Gazebo đo thật, không AI; ảnh midpoint cố định frame2/t4s, không chọn theo chất lượng. Ảnh đầu t2s cũng giữ để thấy cold-start thiếu LiDAR.

## Quyết định

Chưa promote DR-SPAAM cho sensor profile này. Semantic/association/persona H1–H5/train masks vẫnfalse, chưa World Model train, không dùng HuNavSim hoặc đổi Nav2/robot; navigationTIMEOUT vẫn chưa sửa.

Theo lựa chọn giữ LiDAR+depth, bước tiếp là kiểm bằng chứng bề mặt depth để phân biệt cụm vật thể phẳng với cụm còn chưa rõ, kèm abstention và giữ raw proposals; thử lại clutter cũ rồi fixture mới. Đây là một tầng lọc bảo thủ cần kiểm chứng, chưa tự chuyển thành semantic person detector đã đạt hoặc batch dữ liệu train.
