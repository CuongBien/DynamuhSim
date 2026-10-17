# ComplexityNav độc lập — vòng 3

Vòng này khảo sát ComplexityNav chính thức, commit
`36acac12a940a20dceb7b4e1d36acc4c7a4e344a`. Không chỉnh source upstream,
Python ROS hoặc kiến trúc robot; chưa nối Gazebo/nhà xưởng.

## Chạy demo

Từ thư mục gốc DynamuhSim:

```bash
bash experiments/social_navigation/round3/run.sh build
bash experiments/social_navigation/round3/run.sh demo
```

Image đã có trên máy này. Build dùng lại image vòng 2 làm nền, tự build image
vòng 2 nếu chưa có. Runtime: Python 3.8.13, Torch 1.13.1 CPU, Debian 11 trong
Docker trên host Ubuntu 24.04. Tác giả dùng Python 3.8.10; đây là khác biệt bản
patch được ghi rõ, không phải thử native Python 3.12/ROS Jazzy.

`demo` chạy entrypoint/renderer gốc qua launcher tương thích và config riêng:
CV-MPC có sẵn trong framework, không phải demo RGL đã train theo README. Không
có checkpoint RGL tương ứng trong repo. Lần thử config nguyên bản ban đầu lỗi
đường dẫn máy tác giả; thêm symlink chỉ trong image. Config ngoài source bổ sung
các class mà test.py cần và chọn CV để không dùng checkpoint SGAN ngoài repo.
Launcher trả về env.unwrapped vì lớp bọc Gym không nhận reset positional args.

Video xuất tại `reports/compatible_native_demo.mp4`. Matplotlib Agg không mở
cửa sổ GUI. MP4 native phát 8fps trong khi timestep 0.25s: **2× tốc độ mô phỏng**.

## Quan sát từng yếu tố

```bash
bash experiments/social_navigation/round3/run.sh scene \
  --name dense --counts 15 15 0 0 --width 10 --seed 0 --video

bash experiments/social_navigation/round3/run.sh scene \
  --name narrow --counts 1 2 0 0 --width 1.5 --seed 0 --video

bash experiments/social_navigation/round3/run.sh scene \
  --name heterogeneous --counts 5 5 2 3 --width 10 --seed 0 --video
```

`--counts` lần lượt là ORCA, SFM, linear/CV, static. `--scenario` có passing,
crossing, passing_crossing hoặc random. Robot trong runner là ORCA holonomic
cố định, khác CV-MPC ở demo và khác Mecanum/MPPI ROS. Có `--invisible-robot` để
thử phản ứng. Seed gốc được đặt bằng 1000 + seed CLI; nếu cần lấy mẫu lại cảnh
ban đầu, dùng seed + 10000 mỗi lần, ghi rõ trong JSON/config.

Runner lấy cảnh và chuỗi goal bằng generator gốc rồi từ chối cảnh có overlap
ngay từ đầu. Đây là bước kiểm tra ở wrapper, không sửa thuật toán/generator.
Có tối đa 100 lần lấy mẫu. Dữ liệu gốc có overlap được giữ riêng tại
`reports/raw_initial/` (gitignored) và audit tổng hợp trong báo cáo.

```bash
bash experiments/social_navigation/round3/run.sh matrix
bash experiments/social_navigation/round3/run.sh matrix --resume
```

Ma trận có 30 cấu hình; chạy tuần tự, timeout 180s thời gian thực mỗi lượt.
Resume bỏ qua lượt đã xong, hình học ban đầu hợp lệ và không lỗi render.
Cùng tên/seed sẽ ghi đè kết quả. Config, sampling seed, quỹ đạo và summary nằm
trong configs/ và reports/; video đại diện được ghi cho seed CLI 0.

Native CoherentFilter renderer lỗi với dưới 4 người. Runner chọn DBSCAN có sẵn
cho lời gọi màu grouping của renderer ở trường hợp đó; màu theo policy và
chuyển động không đổi. Summary ghi rõ render_adjustments. Lỗi renderer được
tách khỏi trạng thái va chạm/goal của mô phỏng.

## Kết quả và giới hạn

[Báo cáo vòng 3](reports/ROUND3.md), [CSV](reports/results.csv),
[tổng hợp JSON](reports/analysis.json). Grouping là phân cụm quỹ đạo để nhận diện
và hiển thị; không phải mô hình trò chuyện hoặc luật gắn kết nhóm chủ động.
Static agents là người gần như đứng yên, không tự động thành nhóm hội thoại.

Tất cả demo ba framework vẫn độc lập với Gazebo. Chưa có so sánh fair cùng
nhà xưởng/robot/controller; không suy ra chất lượng Mecanum/Nav2 từ các số này.

Phân tích lại dữ liệu bằng venv phân tích vòng 1 (không chạy simulator ở venv này):

```bash
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/round3/analyze.py
```
