# Cohort Gazebo chung —6 người,3 cấu hình,3 cảnh

[Báo cáo và ảnh/GIF thật](../../../archive/social_navigation/reports/common_gazebo/COMMON_GAZEBO_REPORT.md): đã chạy9 lượt,8 hoàn tất10s và1 SocNavGym crossing lỗi terminal tại5.5s. Worker/poses/LiDAR/camera/GPU và cleanup được kiểm. **Chưa nghiệm thu SocNavGym crossing hoặc Nav2; robot còn trôi sau fault trong baseline đã đo.** Giữ nguyên cả lỗi và overlap khi teardown.

Bộ offline trước đã commit `ff79dbf`. Cohort này dùng nguyên Runner đã đóng băng,6 người radius0.36m, robot nominal radius0.25m,42 AABBs, cùng initial fixtures và world-frame velocity script. Worker nhận pose/velocity đo thực từ Gazebo; không phát lại human trajectory có sẵn.3 cấu hình: SocNavGym guarded individual, CrowdSimPlus ORCA+, ComplexityNav geometry mixed. Formation tắt; group/lifecycle được kiểm riêng ở vòng trước. Đây là pilot one-seed10s, chỉ near conditions; không phải xếp hạng native algorithms hay navigation success.

Chỉ build plugin của experiment, không build/sửa robot workspace:

```bash
bash experiments/social_navigation/navigation/common_gazebo/run.sh build
```

Ví dụ chạy bản ORCA+ đã hoàn tất, output phải MỚI, từ root repo trong Ubuntu/WSL:

```bash
bash experiments/social_navigation/navigation/common_gazebo/run.sh live \
  --model crowd_orca_plus --case crossing_near --gui \
  --report-dir experiments/social_navigation/navigation/common_gazebo/reports/my_demo
bash experiments/social_navigation/navigation/common_gazebo/run.sh analyze \
  --live experiments/social_navigation/navigation/common_gazebo/reports/my_demo
```

Cảnh chọn `crossing_near`, `opposed_near`, `blocked_narrow_near`; model chọn `soc_guarded_orca`, `crowd_orca_plus`, `complex_geometry_mixed`. Mặc định10s mô phỏng rồi tự đóng để lưu dữ liệu. GUI có thể Pause/Play để xem lâu hơn; cohort mới chưa có xác nhận thao tác tay riêng. Không gọi đây là demo tự chạy an toàn nếu dùng Soc crossing hiện tại. Không chạy đồng thời các phiên Gazebo/ROS khác dùng domain91; mỗi phiên có GZ partition UUID/PID, worker/container sở hữu riêng và lock domain nội bộ. Không reset world trong session. Robot tự nhận lệnh vận tốc thử nghiệm; Nav2 không bật.

Cần runtime đã có của project: ROS2 Jazzy trên Python3.12, SocNavGym venv3.12,2 Docker image Python3.8.13 ghim content ID trong common_response/inputs/images.json, generated deps common_response/complexitynav_geometry và install/robot hiện có. Nếu deps offline bị thiếu, chạy preflight của hai experiment common_response/complexitynav_gazebo trước; không thay system Python. NVIDIA D3D12 được preflight và Ogre log kiểm; không fallback software. Worker giới hạn2 CPU/4GB container và một BLAS thread; physics/native ORCA vẫn dùng CPU, camera/GPU LiDAR/rendering dùng GPU.

Producer transport của cohort đã nghỉ dùng; receipt so sánh prefix vẫn giữ nguyên. Tái lập phép đo cũ cần [source đã ghim](https://github.com/CuongBien/DynamuhSim/tree/68b2dded70b1ae41361f2e1aa8dad593ae2111f6/experiments/social_navigation/benchmark).

Transport01 đã khớp exact20 bước saved offline của cả3 cấu hình, rồi gửi clock sai và xác nhận worker đóng terminal. Một attempt đầu thiếu omega giữ trong reports/transport_attempt01; worker từ chối trước update0. CLI sau đo chỉ thêm lựa chọn tên report mới; worker/policy không sửa.5.5s error không phải test fault chủ động.

Script chạy cohort cố định đã được gỡ khỏi entrypoint hiện hành; bản gốc còn trong `reports/source_snapshot/cohort.py`. Dùng lệnh `run.sh live/analyze` ở trên cho output mới. Script tổng hợp bảng cohort cố định đã nghỉ dùng; bảng/đồ thị đã đo giữ nguyên trong report và [source trước khi dọn](https://github.com/CuongBien/DynamuhSim/tree/fbfa33a6c5ed8b062afe7523073f24b77477b852/experiments/social_navigation/benchmark). Analyzer kiểm snapshot SHA,42 model geometry,6 proxy radius, actual/interpolated pose, swept gaps và LiDAR. Audit geometry outcome tách khỏi hoàn tất execution: một prefix lỗi vẫn là lỗi. Khoảng hở đánh giá0..10s hoặc tới fault; toàn recording gồm teardown lưu riêng.

Mọi kết quả dùng source/input SHA và raw observations gzip byte-identical; ảnh/GIF là RGB Gazebo1frame/s,640×480,4×, không phải AI. Checkpoint preservation là dữ liệu local ignored; máy khác cần checkpoint audit tương đương hoặc cohort mới, không ghi đè report cũ.6/6 human IDs không được tuyên bố LiDAR thấy trong mọi scene: Soc narrow chỉ matched IDs3/4, các report khác có1..6.

Việc tiếp theo là sửa họ proposal tránh robot và dừng robot khi worker fault trong phiên bản mới, kiểm lại đúng ca lỗi, rồi mới Nav2. Giữ native/vendor, baseline/report này và kiến trúc robot nguyên trạng; không chuyển Isaac Sim hoặc World Model trong bước này.
