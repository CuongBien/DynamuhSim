# Sao lưu dataset01 lên Google Drive

Đã upload 14 ZIP độc lập, cùng parts_manifest.json và README_RESTORE.txt vào:
https://drive.google.com/drive/folders/1I3kfKbTm5X6Bj32RyPVck5kFSjmVgqnl

Tải toàn bộ ZIP rồi giải nén **chung một thư mục mới**, giữ đường dẫn con. Không nối các ZIP bằng cat/copy. Giải nén thử tại máy khôi phục đủ 1.022 entries, từng file gốc đúng SHA256, reader đọc được depth [20,240,424] và target BEV [3,64,64]. Đã đọc lại metadata Drive và đối chiếu đủ tên, dung lượng và parent của 14 ZIP. Connector không trả checksum cloud, nên chưa tuyên bố xác minh SHA256 bằng tải xuống từ Drive.

Bản đầy đủ tại generated/archives/wm_dataset01_backup_20261023/DynamuhSim_wm_dataset01_backup_20261023.zip có 881.723.956 byte. Nguồn gói dữ liệu tại commit 88924bf, cộng script backup.py mới, với index cũ của người dùng được giữ nguyên. Không có model WM/RL đã train trong gói này.

Gói gồm dataset01 hoàn chỉnh, code/report/example của stage, cả 10 MCAP (8 accepted + 2 quarantined), trace bổ trợ và cấu hình/log runtime. Không gồm bản normalized trùng lặp, diagnostic paired_depth on/off, Git, cài đặt dependency hoặc toàn bộ asset/repository; không phải ảnh môi trường ROS có thể khởi động độc lập. Dữ liệu portable chỉ cần Python + NumPy; dựng lại Gazebo cần repository và asset đã ghi đường dẫn.

Lần thử upload ZIP đầy đủ bị giới hạn 512 MiB tại client; ZIP 377 MB tiếp theo bị chặn ở cổng truyền 100 MiB. Không có file ZIP lỗi được tạo trên Drive. Bản sau cùng chia dưới 80 MiB theo nhóm entries, tất cả upload thành công. Quyền thư mục đọc lại chỉ có owner; không tạo public share.

Chi tiết: backup_drive01.json. Script backup.py tạo bản gốc; split_backup.py tạo 14 ZIP nhỏ; verify_restore.py giải nén và kiểm tra reader. Các script có chủ ý từ chối ghi đè output đã có.
