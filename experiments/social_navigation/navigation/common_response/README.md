# Locked response inputs và worker đang dùng

Các nguồn và hợp đồng đang dùng:

- `inputs/`: sáu fixture/trace đã khóa, layout, cấu hình, native dt, model/image pins và input hashes.
- `model_worker.py`: backend Runner được IPC worker dùng.
- `preflight.py`: dựng dependency tại các stage `navigation/` hiện hành, kiểm hash và đúng thay đổi bookkeeping population của CrowdSimPlus; không thay motion methods.

Từ repository root, chuẩn bị dependency bằng môi trường Python hiện có:

```bash
python3 experiments/social_navigation/navigation/common_response/preflight.py
```

Cần vendor/venv và các image đã pin theo [shared Nav2 runtime](../../../../docs/SHARED_NAV2_RUNTIME.md). Preflight không tự tải vendor, tạo image hoặc thay locked inputs. Worker hiện hành vẫn mount geometry kernel/roadmap cùng native-contract dependency chỉ đọc.

`prepare.py`, cohort `run.py`, `analyze.py` và `plot.py` của pilot 36 lượt `ff79dbf` đã được gỡ khỏi HEAD. [Commit gốc](https://github.com/CuongBien/DynamuhSim/commit/ff79dbfe250837cc05eb23128756dfd4668c2f14) · [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/f3e54812c3149df050db339eeb8cdc490614b648/experiments/social_navigation/benchmark/common_response) · [Report lịch sử](../../../../docs/REPORT_STORAGE.md).

Kết quả offline cũ vẫn giữ: 18 cặp far/near, sáu model variants và các repeat prefix, không phải benchmark Gazebo/Nav2 hoặc xếp hạng raw speed. Tái lập việc tạo input/phân tích cũ cần source revision/manifest tương ứng; không ghi lại scene để vượt qua input hash gate.

IPC worker nạp `kernel`/`motion` từ `navigation/socnavgym_warehouse_lifecycle` trước khi khởi tạo Runner đã khóa; module hiện hành không bị source cache ở benchmark cũ lấn khi Runner thêm legacy path. File `model_worker.py`, Runner class và toàn bộ input hashes giữ nguyên; không tạo lại alias source tại đường dẫn benchmark cũ.
