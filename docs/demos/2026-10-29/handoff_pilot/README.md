# Pilot dữ liệu bàn giao — PASS

Đã hoàn thành bước nhỏ cuối phase: **hai episode từ raw đã nghiệm thu**, không chạy thêm Gazebo và chưa train WM/RL. Bộ mới tách hoàn toàn dataset01.

| Episode | Windows | Target hợp lệ | Depth raw khớp từng byte | BEV được tính lại |
| --- | ---: | ---: | ---: | ---: |
| pilot_conversation |264 |264 |464 |1056 |
| pilot_walking |253 |253 |446 |1012 |
| Tổng |517 |517 |910 |2068 |

Đã kiểm tra mọi window về header/availability/query/anchor,history coverage,lệnh tương lai ZOH,raw GT trajectory/group labels,calibration,BEV estimated-input/true-anchor-label và mask. Loader chạy Python isolated mode, không import ROS; observation đọc được trong bản portable không có thư mục LABEL và fingerprint vẫn giống hệt. Bốn test mới kiểm tra NaN payload,validity/availability và từ chối frame/TF sai.

## Gói bàn giao

- [Tài liệu MuJoCo](HANDOFF_MUJOCO.md): phần tái sử dụng,phần backend cần port và checklist nghiệm thu.
- [Audit](audit.json), [tóm tắt](summary.json), [ZIP receipt](package_receipt.json), [loader inspection](portable_inspection.json).
- [Mẫu window thật](examples/window01/metadata.json):NPZ SENSOR_ONLY/LABEL_ONLY/calibration/masks trong cùng thư mục.
- Full portable ZIP local:experiments/social_navigation/datasets/handoff_pilot/generated/package01/handoff_data_pilot01.zip;31.7MB compressed,about379MB unpacked. Full ZIP/data không ở Git; raw MCAP không trong ZIP.

Sau giải nén vào thư mục mới, chỉ cần Python+NumPy:

```bash
cd handoff_data_pilot01
python tools/inspect_pilot.py --dataset .
```

History20×240×424 depth,20×360LiDAR,20×10IMU,20×9odom,20×3action; condition future30×3; BEV future3×64×64 tại+0.5/+1/+3s. Future targets là dữ liệu depth ghi thật đã warp, **không phải dự đoán World Model**. Grayunknown/whitefree/orangeoccupied trong ảnh; FOV/occlusion khiến nhiều ô unknown. BEV không phải social costmap hoặc full-world map.

Target-valid517/517 chỉ nói đủ support dữ liệu. Toàn bộ split=pilot_only,training_authorized=false,fit_allowed=false,persona/association/reward invalid. Không dùng pilot này làm train/val/test; seed đã được xem trong kiểm chứng simulator. dataset01 metadata/inventory và nguồn cũ giữ nguyên. Original MCAP hashes,export-source freeze và audit-source freeze được kiểm chứng; mọi archive member đã qua CRC và SHA256.

**Đây là mốc có thể tạm dừng phase Gazebo và bàn giao backend MuJoCo.** Chưa chuyển backend,thu batch lớn,train WM/RL hoặc tạo reward/semantic H1–H5. Bước tiếp thuộc phase migration: tuân thủ contract và xuất hai cảnh tương đương từ backend mới để đối chiếu loader/time/frame/masks.
