# Đóng góp vào NarrowNav Research

1. Đọc [AGENTS.md](AGENTS.md) và [lộ trình](docs/ROADMAP.md). Ghi rõ mốc, điều kiện nghiệm thu và hợp đồng hoặc giao thức mà task thay đổi. Kiểm tra `git status --short` để tách thay đổi của mình khỏi nội dung có sẵn.
2. Xác định module sở hữu theo [ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md). Khi chỉnh interface, cập nhật schema, tài liệu và các test của bên cung cấp lẫn bên gọi. Với thay đổi kiến trúc, bổ sung ADR nêu quyết định, lý do, ảnh hưởng và hướng chuyển đổi.
3. Khi thực hiện thí nghiệm, cố định điều kiện so sánh trước khi chạy. Giữ YAML và định nghĩa thí nghiệm trong repo. Lưu manifest và output theo run, bảo toàn ID, seed, hash và thông tin phiên bản theo [schema episode](docs/schemas/EPISODE_SCHEMA.md).
4. Chạy các test liên quan và ghi lệnh, số pass/fail/skip hoặc lý do chưa chạy được. Với thay đổi chỉ gồm tài liệu, đối chiếu liên kết tương đối, sơ đồ và các bất biến giữa schema, protocol, ADR. Dùng [giao thức kiểm định](docs/protocols/SIMULATION_VALIDATION_PROTOCOL.md) cho các task có runtime.
5. Kiểm tra `git status --short`, `git diff --check`, diff đã stage nếu có và nội dung các tệp mới. Chỉ commit phần được yêu cầu. Giữ bí mật, dữ liệu nghiên cứu thô và các kết quả chạy ngoài commit.

Khi gửi bản thay đổi để review, nêu phạm vi, ADR hoặc schema bị ảnh hưởng, bằng chứng kiểm tra và điều kiện cần để lặp lại kết quả.
