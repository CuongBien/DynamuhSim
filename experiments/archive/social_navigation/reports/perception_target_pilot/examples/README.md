# Một mẫu thật để xem

Sample 411 là mẫu giữa cố định của 823 cửa sổ, anchor 53.4 s, depth cuối khả dụng 53.3 s. Không chọn theo chất lượng detector.

- `sensor_inputs.npz`: 20 bước x 16 slot; XY/vXY từ depth + odom, missing=zero/valid=false; track ID cục bộ, chưa xác nhận semantic người. BEV hiện tại đã warp bằng odom bảo thủ tới anchor ước lượng.
- `future_bev_labels.npz`: 3 grid tại +0.5/+1/+3 s, `[3,64,64]`, tọa độ anchor thật của robot. Pose simulator chỉ nằm trong LABEL; unknown=-1 được loại khỏi loss bằng cell_observed_valid. Đây là future depth thực quan sát, không phải bản đồ đầy đủ hoặc model prediction.
- `association_hints_LABEL_ONLY.npz`: ghép hình học với human GT để chẩn đoán, không đưa vào input. `association_accepted=false`.
- `sample.json`: schema/shape/dtype. Không bật train, không tự suy H1–H5.

Các NPZ chỉ cần NumPy, `allow_pickle=False`. Mẫu này chưa gồm lại toàn bộ depth history; depth/raw/K/TF nằm trong portable episode gốc. Đọc mask cho từng timestep/slot/cell, không coi missing hoặc unknown là nhãn âm.
