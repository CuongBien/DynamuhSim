# SocNavGym response v2 và terminal Pause interlock

[Báo cáo, ảnh/GIF thật và kiểm chứng](reports/RESPONSE_FIX_REPORT.md). Baseline lỗi giữ trong commit `9ab3c9f`. Bản v2 dùng một experiment riêng, không sửa robot/Nav2/vendor hoặc evidence cũ. Đã có6/6 offline,4 lượt Gazebo bình thường (2 giao cắt,đối đầu,lối hẹp) và2 test worker fault/socket loss với Pause đúng. Chưa bật Nav2 hoặc kiểm formation bằng projector v2.

Bản cũ chỉ scale chung một proposal ORCA; bản mới cho từng người chọn velocity riêng dưới support halfspaces/speed discs, target endpoint gap4cm, fallback2cm/2mm và swept validation floor2mm. Robot prediction giữ measured velocity, không giả định robot nhường. Bất khả thi/invalid input vẫn terminal. Plugin phát zero velocity và Pause world khi lỗi; đây là simulation interlock, không phải physical emergency braking. Sau fault cần session mới; Play bị interlock Pause lại sau một tick trong hai test đã đo.

Từ root repo trong Ubuntu/WSL, build CHỈ plugin thử nghiệm:

```bash
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh build
```

Chạy vào output MỚI, không ghi đè bằng chứng:

```bash
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh live \
  --model soc_guarded_orca --case crossing_near --gui \
  --report-dir experiments/social_navigation/benchmark/socnavgym_response_fix/reports/my_crossing
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh analyze \
  --live experiments/social_navigation/benchmark/socnavgym_response_fix/reports/my_crossing
```

Mặc định10s mô phỏng rồi tự đóng. Cảnh khác: `opposed_near`, `blocked_narrow_near`.6 người,radius0.36m,robot nominal0.25m,physics0.0025s,native planning0.1s. Robot tự nhận velocity script, không chạy Nav2; actual robot feedback được gửi vào worker. Cùng initial/script không có nghĩa robot traces giống hệt giữa mọi session. GPU NVIDIA phải được preflight/Ogre xác nhận; không fallback software. Worker2 CPU/one BLAS thread; native policy/physics vẫn dùng CPU.

Test fault có chủ ý, sẽ Pause world và tự kết thúc sau kiểm interlock, không coi là demo policy:

```bash
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh live \
  --model soc_guarded_orca --case crossing_near --fault-after-updates 55 \
  --report-dir experiments/social_navigation/benchmark/socnavgym_response_fix/reports/my_fault
```

Thêm `--fault-mode disconnect` để worker đóng socket thay vì trả lỗi. Runner thử Play một lần sau fault rồi xác nhận interlock Pause lại. Không reset world trong session. Dùng ROS domain91/GZ partition riêng và lock chung với common_gazebo; không chạy đồng thời phiên ROS/Gazebo khác dùng domain91. Dừng đúng phiên của experiment; không killall.

Cần ROS2 Jazzy, existing install/robot và SocNavGym Python3.12 venv của project. Không cài SciPy, system packages, Docker image mới hoặc đổi ROS Python. Dependencies numpy/RVO2 đã có; analyzer dùng Shapely/PIL/Matplotlib trong venv. Build/live dùng run.sh để source Jazzy/workspace; native/backend source và locked fixtures lấy từ common_response đã commit. Source hashes, copied snapshots và local preservation checkpoint được kiểm; máy khác cần checkpoint tương đương hoặc cohort mới.

IPC vào output mới:

```bash
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh transport --report-name my_transport
```

IPC01 đã so exact20 bước với output offline v2 và xác nhận đóng terminal khi clock sai. `diagnose.py`, `offline.py`, `test_projection.py` giữ tên output của lần đo và từ chối ghi đè; cần quy hoạch output mới trước khi chạy lại. Script cohort cố định đã gỡ khỏi entrypoint hiện hành; bản gốc còn trong `reports/source_snapshot/cohort.py`. Dùng `run.sh live/analyze` ở trên cho output mới. `summarize.py` chỉ vẽ dữ liệu đo.5 tests hình học kiểm bằng closest-approach/GEOS độc lập, gồm trường hợp bất khả thi phải từ chối.

Mọi raw pose/scan/odom/worker request, SDF và log có gzip byte-identical; camera1fps640×480,4×, không phải AI. Target4cm và6/6 one-seed không phải bảo đảm an toàn/tự nhiên tổng quát. Bằng chứng group/conversation vẫn thuộc adapter lifecycle trước; không gán kết quả individual v2 cho group. Việc kế tiếp: pilot Nav2 nhỏ có robot chủ động tránh/nhường, sau đó mới benchmark navigation. Không World Model/Isaac Sim hoặc đổi kiến trúc robot.
