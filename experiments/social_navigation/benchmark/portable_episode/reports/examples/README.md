# Một sample ghi thật, chỉ để review

Sample0 của episode socnavgym_standing42_earlyavoid_01, anchor12.3s. Depth cuối trong history được đo12.2s, không phải ảnh đồng thời với anchor. BEV giữ frame local của depth12.2s; chưa motion-compensate. Nhãn tương lai dùng ego frame tại anchor.

- observation.npz: toàn bộ input fields một mẫu,20ảnh depth history, current BEV, K và optical→base_link; không human GT/persona/backend/future.
- labels.npz: human XY30bước×6ID×2, executed robot actions30×3, group IDs/modes và persona unknown. Đây là nhãn, không phải model prediction.
- task_masks.npz: GT trajectory available, association/M3/future-BEV/training eligibilityfalse.
- sample.json: shape/dtype, timestamp và vai trò.

Một sample không phải episode mới, không dùng làm tập train. File đầy đủ823sample nằm trong bundle local đã chỉ ởREADME cấp trên. Đọc NPZ bằng np.load(..., allow_pickle=False).
