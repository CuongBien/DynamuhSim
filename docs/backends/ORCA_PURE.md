# ORCA thuần với RVO2

Chọn `humans=orca` để dùng một `OrcaHumanBackend` cho toàn bộ người trong episode. Backend chạy native `rvo2.PyRVOSimulator` từ phân phối `pyrvo2==0.0.0`, SHA `c2c46ba8d59556aa10faf03479293236efea154d`. Registry chỉ nạp binding khi tạo backend ORCA. Caller Python inject `create_human_backend("orca")` vào `MuJoCoEngine` cùng cấu hình `humans=orca`. [Cấu hình](../../configs/humans/orca.yaml) · [Hướng dẫn môi trường](../setup/HUMAN_MODELS_ENV.md)

| Tham số mặc định | Giá trị |
| --- | --- |
| Period solve | `simulation.human_control_dt_ns`, mặc định 100 ms |
| Preferred speed / dung sai đích | 0,8 m/s / 0,10 m |
| Neighbor distance / số lân cận tối đa | 4,0 m / 10 |
| Horizon agent / vật cản | 2,0 s / 2,0 s |
| Bán kính người / robot | Theo `HumanContext` và profile scene |
| Tốc độ tối đa của agent người | Theo `scenarios.human_profile.max_speed_mps`, 1,4 m/s trong fixture mặc định |
| Robot profile | `reciprocal_external_observed` |

## Đồng bộ và giả định tương hỗ

Reset khởi tạo simulator và xóa roster native, obstacle tree và latch goal của episode trước. Lần propose đầu thêm polygon tường solid theo thứ tự ID, chuẩn hóa CCW để RVO2 tạo segment và gọi `processObstacles()`. Trụ static trong context dùng polygon ngoại tiếp từ runtime. RVO2 dùng cây vật cản tĩnh, nên profile hiện chỉ hỗ trợ solids tĩnh của scene. Thêm người theo tuple ID đã sắp, sau đó thêm một slot robot. [API obstacle RVO2](https://gamma-web.iacs.umd.edu/RVO2/documentation/2.0/)

Mỗi callback gọi `setAgentPosition`, `setAgentVelocity` và `setAgentRadius` bằng snapshot hiện tại. Preferred velocity người hướng tới goal trong `world`. Gần đích, preferred speed chừa quãng đường cho một kỳ giữ lệnh và phanh với gia tốc của profile. Actor đã vào dung sai giữ ID và phát zero. `doStep()` giải ORCA native và tự tiến vị trí nội bộ. Backend chỉ đọc vận tốc người, rồi ghi đè toàn bộ pose và vận tốc nội bộ bằng snapshot đo tiếp theo. MuJoCo sở hữu tích phân thực thi. Output ORCA đi thẳng tới limiter chung.

Robot đi vào solver với pose, bán kính và vận tốc đo tại đầu callback. Preferred velocity của slot robot cũng lấy từ vận tốc vừa đo. Backend bỏ kết quả robot. Policy external tiếp tục gửi lệnh drivetrain qua MuJoCo. Native ORCA dùng nửa hiệu chỉnh tránh va chạm mỗi phía. Vì robot external có thể không nhận nửa còn lại, profile này không có bảo đảm tránh va chạm của hai agent hợp tác. Ca robot tiến vào khe hẹp vẫn được lưu với contact thật. [Nhóm tác giả ORCA](https://gamma.cs.unc.edu/ORCA/) · [Hiệu chỉnh 0,5 trong mã RVO2 đã khóa](https://github.com/sybrenstuvel/Python-RVO2/blob/c2c46ba8d59556aa10faf03479293236efea154d/src/Agent.cpp#L349) · [ADR-0005](../decisions/ADR-0005-orca-external-robot.md)

Seed backend được derive và lưu theo schema. Profile không rút RNG, nên hai bên có thể bế tắc khi đối xứng tuyệt đối. Unit test giữ ca này. Trace có các vector requested, limited, executed và pose để xem dao động đổi hướng. Clearance footprint cuối tick và contact native lấy ở substep có ngữ nghĩa riêng. Guard mocap và failure trace giữ riêng để chẩn đoán dừng hình học, teleport và lỗi hạ tầng. [Biên 2D–3D](../architecture/HUMAN_2D_3D_BRIDGE_NOTES.md)

## Chạy fixture kỹ thuật

```bash
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B \
  -m narrow_nav.simulation.smoke --ticks 12 --vx 0.35 humans=orca

mkdir -p runs/task_03_orca
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B \
  experiments/social_navigation/evaluate_orca.py --ticks 160 --policy yield \
  > runs/task_03_orca/yield.json
```

Evaluator chạy một fixture MuJoCo. Robot mặc định tiến 0,35 m/s. Cửa sổ [3,0; 4,5) s chọn `advance`, `stop`, `yield` ngang −0,22 m/s hoặc `reverse` −0,25 m/s. Hết cửa sổ, tất cả về lệnh tiến. Fixture mặc định dùng mecanum cho lệnh ngang. Đây là lệnh kỹ thuật, tách khỏi nhánh đối chứng chính thức của các Card.

Mean và max lấy chuẩn vận tốc executed theo tick robot. Gia tốc và jerk lấy sai phân vector theo thời lượng tick, vì vậy đỉnh ở chỗ đổi lệnh giữa tick phụ thuộc nhịp lấy mẫu. Time-to-goal là mốc đầu vào dung sai. Encounter đếm lần vào vùng khoảng cách tâm robot–người ≤1 m. Stall đếm thời lượng ngoài dung sai với tốc độ <0,02 m/s. Deadlock candidate cần ít nhất 1 s stall liên tục trong vùng có người hoặc robot gần theo cùng khoảng cách tâm. Đây là ngưỡng kỹ thuật của fixture. Contact đếm mẫu cặp theo substep, clearance lấy min cuối tick. Log giữ cấu hình, provenance, stop reason và prefix ngay cả khi episode failed.

Kết quả của TASK 03 dùng ORCA làm baseline đầu tiên qua runtime và evaluator. Kế hoạch Card, writer benchmark và các backend JuPedSim có cổng nghiệm thu riêng. [Báo cáo TASK 03](../reports/human_models/TASK_03_REPORT.md) · [Sai khác đặc tả](../research/HUMAN_MODEL_DEVIATIONS.md)
