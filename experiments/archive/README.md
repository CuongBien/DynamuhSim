# Archive thực nghiệm

`social_navigation/reports/<stage>/` chứa 23 cây report/control đã chốt, tổng
1.814 file tracked. Byte, model, nhãn, freeze, test seal và outcome giữ nguyên.
Media lịch sử vẫn hydrate theo policy hiện có; raw dataset/MCAP không được thêm
vào Git trong đợt chuyển này.

`social_navigation/fixtures/` giữ hai fixture của cohort đã nghỉ dùng.
`social_navigation/support/` giữ ba alias tới source chuẩn có byte đúng như manifest lịch sử ghim.
Runtime hiện hành dùng `python/dynamuhsim/runtime/gpu.py`, `shutdown.py` và
`native/gl_probe.cpp`; các alias support chỉ phục vụ freeze cũ, không tạo bản sao implementation.

Đường dẫn `benchmark/<stage>/reports` là symlink tương thích tới archive.
Hai tên `benchmark/gpu_runtime` và `benchmark/shutdown_fix` cũng là symlink cho
source freeze/hydration cũ. Không có bản sao report thứ hai ở benchmark.
Hydration chỉ chấp nhận đúng các alias nội bộ đã khai báo, vẫn chặn symlink khác
hoặc đích đi ra ngoài checkout. Model/test seal vẫn tồn tại qua đường dẫn cũ;
probe mặc định vẫn cấm truy cập model legacy và không chạy train/test.

Xem [cây benchmark đang dùng](../social_navigation/benchmark/README.md),
[cohort đã nghỉ dùng](../../docs/history/retired_benchmarks.md) và
[hydrate media](../../docs/REPORT_MEDIA.md).
