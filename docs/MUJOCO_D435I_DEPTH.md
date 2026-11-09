# Thu depth D435i trong MuJoCo

`sensors=d435i` bật camera depth của `mecanum_robot`. Mặc định profile này công bố phép đo có nhiễu. Cấu hình không chọn profile vẫn chạy runtime vật lý không cần OpenGL. Chọn `sensors.noise.enabled=false` để đối chứng bằng phép đo lý tưởng đã giới hạn khoảng cách, có `source=d435i_ideal`. [Profile D435i](../configs/sensors/d435i.yaml)

## Lái robot và xem phép đo trực tiếp

Chạy trong phiên desktop từ thư mục gốc.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i
```

Ô trên hiện mẫu D435i mới nhất trong `Observation`. Ô dưới mặc định hiện depth sạch cùng mốc. Nhấn **V** để đổi giữa ảnh sạch, mask và bản đồ sai số, **P** để giữ frame, **W/A/S/D** để lái và **Q/E** để quay. Để xem lại dữ liệu trên đĩa, chạy recorder với `--report` theo lệnh dưới. [Lệnh teleop theo môi trường](MUJOCO_TELEOP_DEPTH.md#chọn-môi-trường) · [Phím và bố cục](MUJOCO_TELEOP_DEPTH.md#điều-khiển)

## Chạy một episode

Chạy từ thư mục gốc với môi trường đã cài `'.[dev,sim]'`. Trên Linux headless, chọn EGL. Với máy có màn hình và GLFW, có thể bỏ `MUJOCO_GL=egl`. Các lệnh dùng renderer và các phụ thuộc đã có của MuJoCo.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl .venv/bin/python \
  -m narrow_nav.simulation.record_depth --report \
  robots=mecanum_robot sensors=d435i worlds=corridor_090 scenarios=corridor_090 \
  simulation.observation_dt_ns=20000000 experiments.duration_ns=1000000000 seed=42
```

Lệnh in đường dẫn run. Mặc định lệnh gửi vận tốc zero qua `RobotAction`. Dùng `--vx 0.20` để thu khi tiến hoặc dùng `--vy` và `--wz` để đặt vận tốc ngang (m/s) và vận tốc góc (rad/s). Điều kiện kết thúc episode vẫn do runtime quyết định. Dùng `--output-root` để chọn thư mục lưu run.

`simulation.observation_dt_ns` dùng đồng hồ hiện có và phải chia hết cho bước vật lý. Ví dụ trên thu ở 50 Hz và robot nhận lệnh ở 20 Hz. Adapter thu frame 0 sau reset rồi thu sau từng substep đến hạn, sau khi cập nhật pose, camera và skin ở đúng thời điểm đó. `capture_time_ns = available_time_ns = sample_index × period_ns` vì profile mô phỏng thu đồng bộ với độ trễ bằng 0. Một tick có thể công bố nhiều frame hoặc dùng lại frame gần nhất nếu chưa đến mốc thu tiếp.

MuJoCo render Z theo trục quang học. Renderer đổi buffer sang mét, đảo thứ tự hàng cho khớp hệ tọa độ SDK, rồi áp `D435iNoiseModel` ngay trong capture. Runtime chỉ lấy mẫu `d435i_noisy` để tạo `Observation`. `get_clean_depth()` trả các frame render sạch tại biên thành công gần nhất qua kênh kiểm định. Teleop dùng chung capture và hiển thị cặp ảnh theo đúng mốc lấy mẫu. [Luồng dữ liệu](architecture/02_DATA_FLOW.md) · [Schema state/action](schemas/STATE_AND_ACTION_SCHEMA.md) · [Teleop](MUJOCO_TELEOP_DEPTH.md)

## Hình học và cơ sở nghiên cứu

Hình học mặc định lấy từ bản xuất `rs-enumerate-devices -c` của một D435i thật, firmware 5.12.14.50. Stream rectified Z16 424×240 có `fx = fy ≈ 212.115 px`, `cx ≈ 211.713 px`, `cy ≈ 117.930 px` và hệ số méo bằng zero. Trị tuyệt đối của tịnh tiến X giữa IR1 và IR2 là 50.105 mm, gần baseline danh định 50 mm. MuJoCo dùng các intrinsics đó cho frustum và coi tâm quang học đã có trên robot là tâm của camera trái. Frame `depth_optical` có +X sang phải, +Y xuống và +Z hướng ra trước. Với thiết bị khác, xuất lại intrinsics cho đúng độ phân giải và lấy baseline từ extrinsics của cặp IR rectified. [Bản xuất SDK của D435i](https://github.com/realsenseai/realsense-ros/issues/2788) · [Datasheet D400, mục 4](https://www.intelrealsense.com/wp-content/uploads/2022/04/Intel-RealSense-D400-Series-Datasheet-April-2022.pdf) · [Quy ước SDK](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0) · [MJCF camera](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)

| Thành phần | Cách triển khai và nguồn |
| --- | --- |
| Disparity và lượng tử hóa | Dùng `d = fx × B / Z`, lượng tử `d` theo bước 1/32 px rồi đổi về Z và làm tròn theo đơn vị Z16 1 mm. Bước disparity lấy theo độ phân giải subpixel được RealSense mô tả. Mô hình không giả lập A-factor của firmware. [RealSense Subpixel Linearity](https://dev.realsenseai.com/docs/white-paper-subpixel-linearity-improvement-for-intel-realsense-depth-cameras/) · [Định dạng Z16](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats) |
| Nhiễu phụ thuộc khoảng cách | Cộng Gaussian với RMS mặc định 0,08 px trong disparity. Tuyến tính hóa phép đổi cho `σ_Z ≈ Z² σ_d / (fx × B)`. Cơ chế tăng sai số theo khoảng cách đến từ hình học stereo. Độ lệch chuẩn Gaussian là tham số khởi tạo, chưa fit với preset, ánh sáng hay vật liệu của rig thực. [RealSense Tuning](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/) · [Projectors for D4xx](https://www.intelrealsense.com/wp-content/uploads/2019/03/WhitePaper_on_Projectors_for_RealSense_D4xx_1.0.pdf) |
| Che khuất và vùng không đo | Chiếu từng pixel sang scanline phải ở `x - d`, giữ disparity gần nhất ở pixel phải và loại những điểm ở ngoài ảnh phải hoặc bị che. Phép chiếu có cơ sở hình học, còn raster gần nhất và ngưỡng tranh chấp là xấp xỉ. Loại pixel không hit hoặc ngoài khoảng 0,105–6 m. Min-Z 0,105 m lấy theo bảng 4-11 cho mode này. Mốc 6 m là giới hạn thực nghiệm cấu hình được. [Datasheet, mục 4.5–4.7](https://www.intelrealsense.com/wp-content/uploads/2022/04/Intel-RealSense-D400-Series-Datasheet-April-2022.pdf) |
| Artifact cạnh và lỗ vùng | Lấy ý tưởng so disparity cục bộ và pseudo-match từ SRU. Tại bước nhảy disparity ngang, model có thể bỏ điểm hoặc lấy disparity của pixel kế bên. Các ô lỗ ngẫu nhiên lấy ý tưởng vùng thiếu liên tục từ baseline. Đây là các xấp xỉ tổng hợp. Các ngưỡng, xác suất và kích thước ô chưa hiệu chuẩn. Pixel không hợp lệ có giá trị `0.0` với `valid_mask=false`. [SRU, phụ lục B](https://journals.sagepub.com/doi/10.1177/02783649251401926) · [Baseline SRU](https://github.com/leggedrobotics/sru-depth-pretraining/blob/main/network/noise_utils/depth_noise_baseline.py) · [Quy ước Z16](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats) |
| Nhiễu thời gian | Cộng thành phần disparity AR(1) theo từng pixel ảnh. Mặc định RMS 0,02 px và tương quan 0,75 mỗi frame. Đặt `temporal_sigma_px=0` để bỏ thành phần này. RealSense đo nhiễu theo thời gian như một đặc tính chất lượng. Dạng AR(1), hệ số và biên độ ở đây là giả định tổng hợp chưa fit. Đổi nhịp capture sẽ đổi thời gian tương quan tính bằng giây. [Datasheet, bảng 4-12](https://www.intelrealsense.com/wp-content/uploads/2022/04/Intel-RealSense-D400-Series-Datasheet-April-2022.pdf) · [RealSense high-speed capture](https://dev.realsenseai.com/docs/high-speed-capture-mode-of-intel-realsense-depth-camera-d435/) |

Mã parametric của SRU dùng lọc lân cận, pseudo-match, điền điểm và lượng tử hóa disparity. Baseline dùng nhiễu Gaussian theo mét, mặt nạ lỗ giãn rộng, salt-and-pepper, dịch ảnh và làm mờ. Profile huấn luyện RealSense của SRU dùng ảnh đã resize 53×30, focal 28 px, baseline 0,12 m và xác suất augmentation 0,9. Bản triển khai tại đây dùng thuật toán NumPy độc lập, giữ lỗ thay vì điền vào quan sát và áp mô hình lên mọi frame của profile noisy. [SRU parametric](https://github.com/leggedrobotics/sru-depth-pretraining/blob/main/network/noise_utils/depth_noise.py) · [SRU baseline](https://github.com/leggedrobotics/sru-depth-pretraining/blob/main/network/noise_utils/depth_noise_baseline.py) · [YAML huấn luyện SRU](https://github.com/leggedrobotics/sru-depth-pretraining/blob/main/config/pretrain_realsense.yaml)

Đây là mô hình disparity gần đúng. Các thành phần IR/projector, vật liệu trong suốt/phản xạ, exposure, confidence của ASIC và preset firmware cần dữ liệu đo riêng nếu đưa vào nghiên cứu. Cần đo sai số mặt phẳng ở nhiều khoảng cách, fill rate theo cảnh và tương quan qua chuỗi tĩnh để hiệu chỉnh profile cho một rig cụ thể. [Mô phỏng active stereo của Zhang và cộng sự](https://arxiv.org/abs/2201.11924) · [Giao thức kiểm định](protocols/SIMULATION_VALIDATION_PROTOCOL.md)

## Dataset, tái lập và báo cáo

| Tệp trong `runs/<run_id>/` | Nội dung |
| --- | --- |
| `manifest.json`, `resolved.yaml` | ID, trạng thái kết thúc, commit, cờ dirty, fingerprint cấu hình/MJCF/asset, phiên bản Python, NumPy, MuJoCo, backend GL, profile hiệu chuẩn, model version và cấu hình nhiễu, convention pixel invalid, engine seed và seed camera. |
| `observations.jsonl`, `observations/depth_*.npz` | Slot robot-visible và các frame nguồn. Mỗi NPZ chứa `depth_m` float32, `valid_mask` bool, ID frame và timestamp capture/available. Mỗi frame được ghi đúng một lần theo sample index. |
| `actions.jsonl` | Lệnh requested và limited tại mỗi tick. |
| `ground_truth.jsonl`, `ground_truth/depth_*.npz` | Pose, executed, contact, trạng thái kết thúc và clean depth cho kiểm định. Frame sạch ghép với frame đo theo episode ID, sample index và thời điểm capture. Mặt nạ sạch chỉ báo hit của renderer, kể cả hit ngoài dải đo camera. |
| `ground_truth/diagnostics.svg` | Khi bật `--report`, mở SVG để xem cặp ảnh cuối, hai mask, bản đồ sai số spatial, biến thiên sai số giữa hai frame liên tiếp, fill rate và RMS biến thiên theo thời gian. Vùng sai số chỉ tính trên pixel hợp lệ ở các frame liên quan. |

Seed camera dùng luồng `sensor` với scope `sensor.depth` và NumPy PCG64. Reset tái tạo RNG và xóa trạng thái AR(1). Cùng scene, seed, lệnh và stack render/phần mềm tái tạo cùng các phép đo. Manifest ghi provenance để kiểm tra môi trường khi replay. Khác phiên bản driver hay phần mềm có thể làm thay đổi render trước khi áp nhiễu.

Kênh ground truth dành cho công cụ đánh giá và chẩn đoán. Khi đọc dữ liệu cho policy hoặc perception, đọc `observations.jsonl` và `observations/` cùng metadata hiệu chuẩn. Báo cáo dùng các frame đã thu để chẩn đoán, không tham gia vòng capture. [Schema episode và quy tắc công bố](schemas/EPISODE_SCHEMA.md)

```bash
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/pytest -q tests/simulation/test_d435i_depth.py
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/pytest -q
.venv/bin/ruff check .
```
