# 04 · Mô hình MuJoCo và hình học tham số

## Cách dựng world

`narrow_nav.simulation.world_builder.build_world(config)` nhận cấu hình Hydra đã resolve. Bộ kiểm tra cấu hình dựng biên vùng đi được, kiểm tra kích thước, ID, footprint ở spawn/goal và giao nhau giữa các footprint ban đầu. Bộ dựng dùng `ElementTree` để tạo hộp sàn và tường, ghép `templates/world_defaults.xml` với template robot/người, rồi xuất chuỗi MJCF tự chứa. Bước vật lý lấy từ `simulation.physics_dt_ns`. ID nghiệp vụ là tên body gốc của mỗi actor. Tên joint, actuator, geom và site của actor có tiền tố `<entity_id>/`. ID `world` dành riêng cho scene.

Ba tệp `worlds/*/main.xml` là bản tham chiếu hình học mặc định có thể nạp trực tiếp. Corridor tách hình học vào `geometry.xml`. Các bản này dùng chung defaults và có site spawn/goal. Builder ghép thêm actor khi dựng scene từ YAML. Test so sánh các hộp collision của bản tham chiếu với bản dựng từ cấu hình mặc định. Builder tìm template theo đường dẫn checkout tính từ module hoặc tham số `mjcf_root`. Scene sinh ra không có đường dẫn asset phụ thuộc thư mục làm việc.

`geometry_inspector.inspect_world` nhận scene đã dựng hoặc đường dẫn `main.xml`. Module dưới `simulation/mujoco/` biên dịch headless, gọi `mj_forward` và sao chép vị trí, kích thước của các hộp collision tĩnh đã biên dịch. Kết quả chỉ chứa dữ liệu hình học Python. [Tham chiếu MJCF](https://mujoco.readthedocs.io/en/latest/XMLreference.html)

## Hệ tọa độ và collision

World dùng mét, radian, XY nằm ngang, +Z hướng lên và mặt sàn ở Z = 0. Trục X dọc hành lang chính, gốc ở tâm hình chữ nhật hành lang chính. Chiều rộng thông thủy đo giữa hai mặt trong của tường dọc theo Y. Sàn dày 0,1 m. Tường có đáy tại Z = 0, đùn ra phía ngoài biên và nối kín tại các góc. Sàn và tường là các hộp collision tĩnh ở group 0 và 1. Điểm spawn/goal là site MJCF. Kiểm tra vị trí dùng footprint tròn và khoảng cách tới hộp tường trong XY. Tolerance cho kích thước và khoảng hở là `1e-9 m`.

| World | Hình học và biến cấu hình | Số đo mặc định |
| --- | --- | --- |
| `corridor` | Hành lang thẳng với hai tường song song, hai tường đầu và sàn hình chữ nhật. `worlds.length_m` và `worlds.width_m` lần lượt là `corridor_length_m` và `corridor_width_m` của builder. | Dài 10 m, thông thủy 2,0 m. |
| `blind_corner` | Nhánh cùng bề rộng nối về +Y ở đầu +X. `turn_length_m` đo từ mép trên của hành lang chính tới cuối nhánh. | Hai nhánh rộng 1,6 m, nhánh rẽ dài 4 m. Tia giữa hai điểm qua góc đụng tường ở 2,207 m trên đoạn 7,725 m. |
| `passing_bay` | Hốc ở phía +Y. `passing_bay_width_m` là tổng thông thủy từ tường phía nam tới tường trong hốc. `passing_bay_length_m` và `passing_bay_center_x_m` định kích thước và vị trí hốc dọc X. | Hành lang 1,4 m, hốc 2,4 m, dài 2,5 m và đặt giữa hành lang. |

`wall_height_m` và `wall_thickness_m` dùng chung cho ba layout. `robots.radius_m` và `scenarios.human_profile.radius_m` là `robot_radius_m` và `human_radius_m` của hai footprint. Scene `head_on` đặt hai làn tại Y = ±0,2 m để cùng cấu hình spawn còn hợp lệ ở bề rộng 1,0 m. Hai scenario bổ sung đặt đích sau góc rẽ và trong hốc.

## Giả định mô hình actor

TASK 03 bổ sung profile mecanum chuyển từ SDF. [Kích thước và cơ cấu con lăn](05_MUJOCO_RUNTIME.md)

- **Robot.** Chassis rigid có free joint, footprint tròn mặc định 0,25 m và tổng khối lượng 8 kg. Hai bánh trụ có bán kính 0,07 m, rộng 0,036 m, cách tâm nhau 0,32 m và nặng 0,25 kg mỗi bánh. Khối lượng chassis là tổng khối lượng trừ khối lượng hai bánh. Hai hinge cùng trục +Y dùng servo vận tốc hữu hạn mô-men, tối đa 1,5 N·m và dải lệnh ±22 rad/s. Trọng lực và tiếp xúc bánh–sàn sinh chuyển động. Hai gối đỡ thụ động lý tưởng chỉ chịu phản lực pháp tuyến với sàn. Collision chassis tách khỏi visual geom. Đây là mô hình bánh cứng và tiếp xúc ma sát, với thông số mặc định phục vụ kiểm thử thay vì hiệu chuẩn từ robot thật.
- **Người.** Body mocap đặt gốc ở mặt sàn, collision trụ cao 1,70 m và bán kính lấy từ profile. Visual geom riêng có contact mask bằng 0. Vị trí và yaw có thể được cập nhật theo ý đồ controller. Robot nhận tiếp xúc từ collision trụ, còn người mocap không đổi chuyển động theo lực va chạm. Contact mask của người chỉ ghép với robot. Runtime/controller phải kiểm soát xuyên tường và giao nhau khi người di chuyển. Kiểm tra spawn/goal trong builder chỉ bao phủ cấu hình tĩnh. Mô hình không mô phỏng dáng đi hoặc động lực học cơ sinh học.

## Kết quả kiểm chứng TASK 02

Môi trường kiểm tra dùng binding và engine MuJoCo **4.0.0**, khóa trong `uv.lock`. Cả ba bản tham chiếu và ba scene ghép actor biên dịch khi bỏ biến display và đổi thư mục làm việc. Hành lang đặt bề rộng 1,0 m, 1,4 m, 1,8 m cho số đo lần lượt **1,000000000000**, **1,400000000000** và **1,800000000000 m** từ hộp collision. Test kiểm vị trí spawn/goal, ray ở góc khuất, hốc đổi kích thước/vị trí, cấu hình sai, nhiều body mocap và liên kết actuator–joint.

Với bước vật lý 2 ms, sau 500 bước, hai bánh nhận cùng lệnh 6 rad/s làm robot tiến **0,389427 m**. Lệnh trái/phải −4/+4 rad/s cho yaw **1,385926 rad**, tâm dịch **0,000279 m**. Khi tiến vào tường tại X = 5 m, tâm chassis dừng tại **X = 4,750161 m** và có contact chassis–tường. Các số đo xác nhận truyền động vật lý cho fixture mặc định. Kiểm thử episode, hiệu chuẩn ma sát và đánh giá sai số trên robot thật thuộc phạm vi sau.

Bộ kiểm thử đạt **40 passed** và Ruff đạt. Chạy lại từ checkout sau khi cài `pip install -e '.[dev,sim]'`.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY pytest -q tests/unit tests/simulation
ruff check .
```
