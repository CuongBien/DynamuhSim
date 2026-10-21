# Intermediate outputs

`summary.json`, `timeline.json`, `native_cluster_check.json` và hình đếm cluster
là output canonical của analyzer cũ, góc mỗi particle ép về ±π. Chúng có thể
tách giả cụm trong spin recovery. Dùng `*_circular_clusters.json` và
[summary gộp](../summary.json), [event windows](../events.json),
[báo cáo](../STABILITY_REPORT.md) để đọc clustering đã xử lý circular yaw.
Các goal status, pose Gazebo/AMCL, clearance và global paths không phụ thuộc
phép dựng lại góc này. Intermediate outputs giữ nguyên phục vụ audit.
