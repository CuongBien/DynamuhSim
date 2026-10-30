# Báo cáo localhost

[Bàn giao công việc detector tiếp theo](DETECTOR_HANDOVER.md).

Trang chính: http://localhost:35315/ — có link tới mọi trang HTML hiện có, kể cả bản nháp/bản lưu được ghi rõ.

## Mở lại trên máy hiện tại

Trong terminal Ubuntu/WSL:

```bash
cd /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer
python3 -m http.server 35315 --bind 127.0.0.1
```

Giữ terminal mở; Ctrl+C dừng server. Nếu server đang mở, chỉ cần truy cập link. Đây là báo cáo tĩnh, không khởi chạy Gazebo hay tính lại benchmark.

HTML, PNG, GIF, video và JSON phục vụ trên web nằm trong thư mục `viewer` ở trên; mỗi báo cáo có thư mục riêng. Trang demo gốc đã được giữ nguyên ở `simulation_20261029.html`, cùng thư mục để các link ảnh/video không đổi. Link cũ tới footprint vẫn được chuyển tới trang gốc.

## Cập nhật mục lục sau khi thêm báo cáo

Từ checkout nhánh `feat/depth-detector-generalization`:

```bash
python3 tools/build_report_hub.py --root /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053/viewer
```

Công cụ tự tìm tất cả HTML, cập nhật `index.html` và `hub_catalog.json`, không sửa báo cáo con. Mục lục có tìm kiếm và phân nhóm; trang mới chưa có mô tả sẽ hiện ở “Các trang khác”. Không tự cập nhật khi chỉ refresh browser: cần chạy lại công cụ khi thêm báo cáo.

## Những gì có và không có trên GitHub

Media và recordings trong `generated` đang bị Git bỏ qua; clone repository không tự có các báo cáo localhost này. Muốn bàn giao toàn bộ trang web, sao chép nguyên thư mục `viewer`, không chỉ file `index.html`. Lệnh server chạy được ngay trong thư mục đã sao chép, đường dẫn lưu máy cũ trên trang chỉ dùng để tham khảo.

Code tạo báo cáo và receipts đã commit nằm trong `tools/`, `experiments/social_navigation/perception/` và `docs/verification/`. Chúng là bằng chứng/kịch bản tái tạo, khác với nguyên bộ media cục bộ. Trang tổng hợp không nâng trạng thái nghiệm thu detector, training hay MuJoCo.
