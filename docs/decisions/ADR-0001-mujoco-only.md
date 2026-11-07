# ADR-0001 · MuJoCo là engine mô phỏng duy nhất

**Trạng thái** · Chấp nhận trong TASK 00

## Bối cảnh

Dự án cần kiểm soát hình học không gian hẹp, đồng hồ, chuyển động, tiếp xúc và nguồn quan sát để các đối chứng đo được tác động của backend người hoặc robot policy. Một engine thống nhất giúp các điều kiện dùng chung cơ chế thực thi và quy trình kiểm định.

## Quyết định

- Dùng MuJoCo 4.0 trở lên cho các thí nghiệm mô phỏng. Lưu MJCF và tài nguyên tại `sim/mujoco/` và khóa phiên bản cụ thể trong `uv.lock`.
- Adapter tại `src/narrow_nav/simulation/mujoco/` giữ độc quyền với đối tượng native trong mã sản phẩm. Bộ điều phối và backend giao tiếp qua kiểu dữ liệu và cổng của `core`.
- Tạo hai đường dữ liệu rõ ràng. Quan sát cảm biến phục vụ robot policy và suy luận. Ground truth phục vụ context hành vi người, kiểm định và nhãn ngoại tuyến với quyền truy cập riêng.
- Tích hợp ROS 2 và Nav2/MPPI về sau qua biên chuyển đổi dữ liệu. Biên triển khai giữ nguyên lựa chọn engine. TASK 00 chỉ xác lập thiết kế.

## Hệ quả

Nghiệm thu về sau cần kiểm định độ đúng của cảm biến, động học và tiếp xúc theo profile thay vì xem đầu ra engine là bằng chứng trực tiếp cho robot thật. Ghi phiên bản MuJoCo và fingerprint của tài nguyên đã nạp để truy nguyên thay đổi số học hoặc hình học. Thay đổi engine, đường dữ liệu hoặc quyền truy cập native cần ADR mới và cập nhật hợp đồng. [Ranh giới module](../architecture/01_MODULE_BOUNDARIES.md), [Kiểm định mô phỏng](../protocols/SIMULATION_VALIDATION_PROTOCOL.md)
