# 04 · Mô hình MuJoCo và hình học tham số

## Cách dựng world

`narrow_nav.simulation.world_builder.build_world(config)` nhận cấu hình Hydra đã resolve. Bộ kiểm tra cấu hình dựng biên vùng đi được, kiểm tra kích thước, ID, footprint ở spawn/goal và giao nhau giữa các footprint ban đầu. Bộ dựng dùng `ElementTree` để tạo hộp sàn và tường, ghép `templates/world_defaults.xml` với template robot/người, rồi xuất chuỗi MJCF tự chứa. Bước vật lý lấy từ `simulation.physics_dt_ns`. ID nghiệp vụ là tên body gốc của mỗi actor. Tên joint, actuator, geom và site của actor có tiền tố `<entity_id>/`. ID `world` dành riêng cho scene.

Ba tệp `worlds/{corridor,blind_corner,passing_bay}/main.xml` là bản tham chiếu hình học mặc định có thể nạp trực tiếp. Corridor tách hình học vào `geometry.xml`. Các bản này dùng chung defaults và có site spawn/goal. Builder ghép thêm actor khi dựng scene từ YAML. Test so sánh các hộp collision của bản tham chiếu với bản dựng từ cấu hình mặc định. Builder tìm template theo đường dẫn checkout tính từ module hoặc tham số `mjcf_root`. Scene sinh ra không có đường dẫn asset phụ thuộc thư mục làm việc.

`geometry_inspector.inspect_world` nhận scene đã dựng hoặc đường dẫn `main.xml`. Module dưới `simulation/mujoco/` biên dịch headless, gọi `mj_forward` và sao chép vị trí, kích thước của hộp và trụ collision tĩnh đã biên dịch. Kết quả chỉ chứa dữ liệu hình học Python. [Tham chiếu MJCF](https://mujoco.readthedocs.io/en/latest/XMLreference.html)

## Hệ tọa độ và collision

World dùng mét, radian, XY nằm ngang, +Z hướng lên và mặt sàn ở Z = 0. Trục X dọc hành lang chính, gốc ở tâm hình chữ nhật hành lang chính. Chiều rộng thông thủy đo giữa hai mặt trong của tường dọc theo Y. Sàn dày 0,1 m. Vách đứng có đáy tại Z = 0, đùn ra phía ngoài biên và nối kín tại các góc. Sàn và tường là các hộp collision tĩnh ở group 0 và 1. Điểm spawn/goal là site MJCF. Kiểm tra vị trí dùng footprint tròn và khoảng cách tới vách trong XY. Tolerance cho kích thước và khoảng hở là `1e-9 m`.

| World | Hình học và biến cấu hình | Số đo mặc định |
| --- | --- | --- |
| `corridor` | World tham số dùng cho fixture. `worlds.length_m` và `worlds.width_m` đặt kích thước. Ba world cố định 30 m có sẵn để khảo sát bề rộng ở các mốc sau. | Dài 10 m, thông thủy 2,0 m. |
| `blind_corner` | Nhánh cùng bề rộng nối về +Y ở đầu +X. `turn_length_m` đo từ mép trên của hành lang chính tới cuối nhánh. Một hốc nằm trên vách +Y của đoạn thẳng trước khi rẽ. | Hai nhánh rộng 1,6 m, nhánh rẽ dài 4 m. Hốc rộng 1,20 m dọc X, sâu 1,10 m từ mặt tường, tâm miệng ở X = 1,20 m. Góc vẫn che khuất tia từ spawn tới đích. |
| `passing_bay` | Hốc ở phía +Y. `passing_bay_width_m` là tổng thông thủy từ tường phía nam tới tường trong hốc. `passing_bay_length_m` và `passing_bay_center_x_m` định kích thước và vị trí hốc dọc X. | Hành lang 1,4 m, hốc 2,4 m, dài 2,5 m và đặt giữa hành lang. |

`wall_height_m` và `wall_thickness_m` dùng chung cho ba layout. `blind_corner` và `passing_bay` dùng chung ba trường `passing_bay_*` để dựng sàn và tường hốc. Với góc khuất, hốc phải nằm trong đoạn thẳng với đủ vách giữa hốc và góc rẽ. `robots.radius_m` và `scenarios.human_profile.radius_m` là bán kính footprint. Scene `head_on` đặt hai làn tại Y = ±0,2 m để cùng cấu hình spawn còn hợp lệ ở bề rộng 1,0 m. Hai scenario bổ sung đặt đích sau góc rẽ và trong hốc.

## World chuyển từ DynamuhSim

Các scene tĩnh lấy bố cục ban đầu từ `src/custom_corridor/worlds/` tại commit `85287ff0781837b60ef6fde72d6244e3823291b5`. Package nguồn khai báo Apache-2.0. MJCF ghi nguồn tương ứng và dùng world XY với mặt sàn Z = 0. Kích thước hộp trong MJCF là nửa kích thước đầy đủ. Trụ đứng dùng bán kính và nửa chiều cao. Sàn plane ở các bản trường học trở thành hộp dày 0,1 m. Corridor 30 m và hospital đã được chỉnh lại hình học cho ma trận hiện tại. [World nguồn](https://github.com/CuongBien/DynamuhSim/tree/85287ff0781837b60ef6fde72d6244e3823291b5/src/custom_corridor/worlds) · [Khai báo giấy phép](https://github.com/CuongBien/DynamuhSim/blob/85287ff0781837b60ef6fde72d6244e3823291b5/src/custom_corridor/package.xml)

| Cặp `worlds` / `scenarios` | Hình học và actor |
| --- | --- |
| `corridor_070`, `corridor_090`, `corridor_120` | Ba hành lang thẳng 30 m, thông thủy cố định 0,70 / 0,90 / 1,20 m theo ID. Hai tường đầu khép tuyến. Mỗi scenario có một người. |
| `arena_obstacle`, `arena_dataset`, `arena_rvo2` | Hộp và trụ tĩnh theo bố cục nguồn. `arena_dataset` dùng chung MJCF với `arena_obstacle` và khởi tạo robot duy nhất. `arena_rvo2` giữ bố cục riêng với bốn người tại spawn. |
| `hospital_easy`, `hospital_medium`, `hospital_hard` | Cùng mặt bằng hành lang 3 m, ghế chờ và hốc nhường đường. Ba mức có 6, 12 và 20 người trong scenario, cộng 10 người ngồi cố định ở mỗi mức. |
| `school_arena`, `school_arena_1`, `school_arena_hunav`, `school_floor` | Ba bố cục tĩnh. `school_arena_hunav` dùng chung MJCF với `school_arena`. Các scenario có lần lượt 8, 0, 8 và 10 người. Dùng `worlds=school_floor scenarios=school_floor_baseline` để bỏ người. |

YAML `configs/worlds/` chọn `geometry_id` tương ứng với `sim/mujoco/worlds/<geometry_id>/main.xml`. Mỗi tệp MJCF nạp độc lập với shared defaults. Khi compose, `simulation/world_assets.py` kiểm tra hình học và tạo footprint từ cùng geom mà builder ghép vào scene. Geom tĩnh thuộc group 0 (sàn) hoặc group 1 (tường và vật cản). Các trụ giữ footprint tròn khi đo khoảng hở và được ghi là `robot_wall` khi có contact thật. Runtime ghi SHA-256 của MJCF world vào metadata cùng các template đã nạp. Thay spawn/goal qua `configs/scenarios/` hoặc Hydra override. Với world đã dựng sẵn, chỉnh kích thước trong MJCF.

Robot dùng một trong hai profile MuJoCo hiện có. Người trong scenario dùng body mocap của dự án và giữ vị trí trong runtime hiện tại. Spawn và goal nằm trong YAML theo từng scenario. `arena_dataset` bắt đầu với danh sách người rỗng vì proxy nguồn chưa được đặt trong vùng đi được. Các bản backup trùng hình học và bản sửa ID riêng của Gazebo được gộp vào world tương ứng.

Các phần phụ thuộc Gazebo/ROS như HuNav/RVO2, Nav2, bridge, bản đồ PGM, bộ chạy dataset, trolley động, animation COLLADA và pipeline camera/depth thuộc migration riêng. Tên `arena_rvo2` và `school_arena_hunav` dùng để nhận diện bố cục nguồn. Những cấu hình này chạy được vòng vật lý robot, reset, đọc trạng thái và contact với runtime hiện tại.

### Ngữ cảnh bề rộng hành lang

Nhóm Hydra `corridor_width` cung cấp `w070`, `w090` và `w120`, tương ứng 0,70 m, 0,90 m và 1,20 m. Áp dụng cho `blind_corner`, `passing_bay` và bốn ID `school_*`, tạo 18 tổ hợp world–bề rộng. Scenario `school_floor_baseline` thêm ba tổ hợp. Nhóm mặc định bỏ trống để giữ cấu hình gốc. Hospital, corridor và arena từ chối preset này. [Lệnh chạy từng world](../MUJOCO_WORLD_COMMANDS.md)

- `school_arena` và `school_arena_hunav` trùng hình học tĩnh. Hai scenario giữ ID cũ, dùng cùng vị trí và đích cho tám người, với một số yaw chênh dưới 0,000004 rad do làm tròn. `school_floor` có vị trí vật cản khác và thêm hai người. `school_arena_1` là hành lang thẳng riêng.
- Ba ID hospital dùng chung `hospital_medium/main.xml`. Mỗi ID giữ một mật độ và bố trí người riêng để đối chứng.
- `corridor_070`, `corridor_090` và `corridor_120` là ba điều kiện bề rộng cố định. Hình học chỉ gồm sàn và tường, với một người trong mỗi scenario.

Preset đặt khoảng cách giữa hai mặt tường collision, đo vuông góc với đoạn hành lang và tại mặt cắt không có đồ đạc. `blind_corner` dùng cùng bề rộng ở hai nhánh và giữ hốc sâu 1,10 m. `passing_bay` giữ phần mở rộng thêm 1,00 m. Các world school khai báo `width_bands` theo cặp biên `[thấp, cao]` trên X hoặc Y trong tọa độ nguồn. Builder thay kích thước và vị trí geom trên các dải đó, gồm lối cạnh cầu thang B–F. Hai hốc A05 và D01 của các bản school lớn có bề rộng miệng thay đổi theo trục giao. Miệng nhỏ nhất đo được là 0,583 m ở A05 với `w070`.

Vị trí spawn và goal trong YAML và Hydra override dùng hệ tọa độ nguồn. Builder chuyển chúng theo bố cục đã chọn, rồi kiểm tra footprint. `width_geom_offsets` và `width_actor_offsets` ghi các dịch chuyển trước khi đổi kích thước, chỉ áp dụng khi có preset. Chúng dời một số đồ đạc và hai điểm của các actor tương ứng để tránh chắn lối hoặc chồng footprint. Khoảng đi được thực tế tại chỗ có vật cản có thể hẹp hơn bề rộng danh định.

Kiểm thử headless biên dịch 21 tổ hợp preset bằng MuJoCo và đo tia đến hai mặt tường với dung sai 0,000001 m. Kiểm thử actuator chạy mecanum qua lối cạnh cầu thang `school_floor` với cả ba preset và đi vào hốc `blind_corner` ở preset 0,70 m.

### Hành lang và khu chờ bệnh viện

`hospital_medium/main.xml` là MJCF chung cho ba mức. Trục X chạy dọc tuyến, dài 56 m từ mặt trong tường đầu này đến mặt trong tường đầu kia. Tường dọc có mặt trong tại Y = ±1,50 m. Mỗi bên có 10 cửa phòng mở rộng 1,10 m giữa hai jamb, với tâm X từ −25,20 m đến +25,20 m và bước 5,60 m. Mỗi ghế chờ rộng 1,60 m và sâu 0,45 m, đặt phía hành lang sát đoạn tường đặc. Tại một ghế có người ngồi ở X = −27,30 m, khoảng thông thủy đo đến tường đối diện là 2,46 m.

Mười hốc có miệng riêng rộng 1,30 m và sâu 1,20 m, xen kẽ hai bên dọc X với bước 5,60 m. Các ID `n_01`–`n_05` và `s_01`–`s_05` dùng được với `--focus-bay`. X của tâm hốc phía bắc lần lượt là −23,50, −12,30, −1,10, 10,10 và 21,30 m. X của tâm hốc phía nam là −17,90, −6,70, 4,50, 15,70 và 26,90 m. Điểm giữa hốc nằm tại Y = +2,10 m phía bắc và Y = −2,10 m phía nam. Kiểm thử xác nhận các cửa và hốc mở tới hành lang cho footprint bán kính 0,25 m. Mecanum vào hốc `n_01` bằng actuator qua runtime.

Ba mức thay đổi mật độ actor ở đầu episode. Pose và goal trong YAML chuẩn bị cho các lượt ra cửa và cắt ngang khi backend người được kết nối. Người ngồi chờ là props tĩnh có geom ID `hospital_waiting_*`. Dùng `model_sha256` và `mjcf_source_sha256` để phân biệt các phiên bản bố cục. [Lệnh kiểm tra cấu hình](../MUJOCO_WORLD_COMMANDS.md#kiểm-tra-cấu-hình-trước-khi-triển-khai-hành-vi)

### Hốc nhường đường trong các world trường học

Geom có tiền tố `yield_bay_` tạo các hốc ba vách màu xanh, với miệng riêng trên tường hành lang và sàn liền ở Z = 0. Khoảng thông thủy là 0,90 m dọc hành lang và 0,95 m tính từ mặt tường vào hốc. Robot có đường kính footprint đến 0,50 m có thể đi thẳng vào giữa và đỗ lọt sau miệng hốc. Cửa lớp nằm ở hai bên vách ngăn. Trong `school_arena_1`, jamb của một số hốc thu hẹp hai cửa lớp liền kề xuống 0,70 m.

- `school_arena`, `school_arena_hunav` và `school_floor` có 13 hốc mỗi bản. Sáu hốc dọc A cách nhau 4,90 m, B–E có một hốc mỗi đoạn, và ba hốc dọc F cách nhau 4,70–4,90 m.
- `school_arena_1` có 8 hốc trên trục X, từ −14 m đến +14 m với bước 4 m.

Các bản school lớn có tường biên phía đông nối đến lớp 11. Giữa tường này và khối cầu thang có lối sàn phẳng rộng thông thủy 1,08 m, mở cả hai đầu để nối trực tiếp hành lang B và F. Tuyến vòng qua C–D–E cũng đi được. Vách đầu đông nam khép mép hành lang A–B. Trong `school_arena_1`, tường phía nam nối từ phòng 15 đến tường cuối hành lang. Hai bản `school_arena` và `school_arena_hunav` đặt ba lô phía tây sát đoạn tường đặc để giữ thông thủy cho robot khi biên đã khép kín.

Test kiểm thông thủy và lối vào của từng hốc trên MJCF đã biên dịch, lối qua cửa lớp ở mẫu đầu tiên và quá trình mecanum chạy bằng actuator rồi dừng trong hốc của `school_floor`.

### Chiều cao miệng hốc nhường đường

Sửa `yield_bay_opening_height_m` trong [configs/config.yaml](../../configs/config.yaml). Giá trị mặc định `0.8` là khoảng thông thủy tính bằng mét từ sàn đến mặt dưới của khối tường. Cấu hình dùng chung cho `blind_corner`, `passing_bay`, nhóm hospital và nhóm school. Giá trị phải lớn hơn 0 và thấp hơn đỉnh tường. Chọn khoảng thông thủy cao hơn đỉnh cụm cảm biến của robot để robot lọt vào.

Khối tường phủ toàn bộ chiều sâu hốc từ cao độ đã chọn tới đỉnh tường. MuJoCo dùng khối này cho va chạm 3D và render depth. Bản đồ footprint XY giữ lối đi phía dưới. Preset `corridor_width` vẫn đổi kích thước XY, còn chiều cao miệng hốc lấy từ khóa trên. Hình học cửa phòng và nhánh rẽ giữ nguyên. Các MJCF tham chiếu mở trực tiếp có chiều cao 0,80 m.

Ví dụ mở camera D435i đối diện hốc bắc đầu tiên của bệnh viện, với chiều cao thông thủy 0,90 m cho riêng lần chạy.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i \
  worlds=hospital_easy scenarios=hospital_easy yield_bay_opening_height_m=0.9 \
  'scenarios.robot.initial_pose=[-23.5,0.35,1.5707963267948966]'
```

## Hình ảnh người Quaternius

`simulation.human_visuals=quaternius` dùng hai bundle chuẩn hóa `casual_male` và `casual_female` trong `sim/mujoco/assets/humans/`. Đây là các mesh mặc quần áo Smooth Casual, cao 1,75 m và 1,65 m, với rig 42 bone và màu material gốc. Bundle giữ URL, CC0-1.0, SHA-256 của FBX nguồn và SHA-256 mảng runtime. Hai bundle là bản sao theo byte của dữ liệu đã dựng cho demo. Quy trình dựng lại dùng converter Blender ngoại tuyến trong `humans/`. [Quy trình và nguồn](../../humans/README.md#sources)

Bộ dựng chọn luân phiên hai mẫu theo thứ tự ID trong scenario. Bone mocap chỉ dùng cho skin render. Adapter lấy XY/yaw từ body vật lý và lấy mẫu clip idle theo thời gian mô phỏng ở biên control tick. Reset lặp lại pose ban đầu. Người giữ XY/yaw tại spawn với runtime hiện tại. Viewer dùng cùng lớp skin và cập nhật idle trong lúc xem. Các pose ngồi của hospital được dựng theo chiều cao ghế hành lang và dùng như vật cản tĩnh.

Hình trụ collision của người dùng geom group 3, ẩn mặc định trong viewer. Hai cấu hình render `quaternius` và `proxy` giữ cùng footprint, contact mask và ID vật lý. Các vật cản là người cố định trong bệnh viện giữ phân loại `robot_wall`; người thuộc scenario giữ `robot_human`. Tài nguyên mesh xuất hiện trong kết quả render MuJoCo. Các slot cảm biến depth và LiDAR vẫn ở trạng thái chưa khả dụng của runtime hiện tại.

## Giả định mô hình actor

TASK 03 bổ sung profile mecanum chuyển từ SDF. [Kích thước và cơ cấu con lăn](05_MUJOCO_RUNTIME.md)

- **Robot.** Chassis rigid có free joint, footprint tròn mặc định 0,25 m và tổng khối lượng 8 kg. Hai bánh trụ có bán kính 0,07 m, rộng 0,036 m, cách tâm nhau 0,32 m và nặng 0,25 kg mỗi bánh. Khối lượng chassis là tổng khối lượng trừ khối lượng hai bánh. Hai hinge cùng trục +Y dùng servo vận tốc hữu hạn mô-men, tối đa 1,5 N·m và dải lệnh ±22 rad/s. Trọng lực và tiếp xúc bánh–sàn sinh chuyển động. Hai gối đỡ thụ động lý tưởng chỉ chịu phản lực pháp tuyến với sàn. Collision chassis tách khỏi visual geom. Đây là mô hình bánh cứng và tiếp xúc ma sát, với thông số mặc định phục vụ kiểm thử thay vì hiệu chuẩn từ robot thật.
- **Người.** Body mocap đặt gốc ở mặt sàn, collision trụ cao 1,70 m và bán kính lấy từ profile. Chế độ `proxy` dùng visual geom có contact mask bằng 0. Chế độ mặc định dùng skin. Vị trí và yaw có thể được cập nhật theo ý đồ controller. Robot nhận tiếp xúc từ collision trụ, còn người mocap giữ chuyển động do controller chỉ định. Contact mask của người chỉ ghép với robot. Runtime/controller phải kiểm soát xuyên tường và giao nhau khi người di chuyển. Kiểm tra spawn/goal trong builder bao phủ cấu hình tĩnh. Động lực học cơ sinh học nằm ngoài mô hình.

## Kết quả kiểm chứng TASK 02

Môi trường kiểm tra dùng binding và engine MuJoCo **4.0.0**, khóa trong `uv.lock`. Cả ba bản tham chiếu và ba scene ghép actor biên dịch khi bỏ biến display và đổi thư mục làm việc. Hành lang đặt bề rộng 1,0 m, 1,4 m, 1,8 m cho số đo lần lượt **1,000000000000**, **1,400000000000** và **1,800000000000 m** từ hộp collision. Test kiểm vị trí spawn/goal, ray ở góc khuất, hốc đổi kích thước/vị trí, cấu hình sai, nhiều body mocap và liên kết actuator–joint.

Với bước vật lý 2 ms, sau 500 bước, hai bánh nhận cùng lệnh 6 rad/s làm robot tiến **0,389427 m**. Lệnh trái/phải −4/+4 rad/s cho yaw **1,385926 rad**, tâm dịch **0,000279 m**. Khi tiến vào tường tại X = 5 m, tâm chassis dừng tại **X = 4,750161 m** và có contact chassis–tường. Các số đo xác nhận truyền động vật lý cho fixture mặc định. Kiểm thử episode, hiệu chuẩn ma sát và đánh giá sai số trên robot thật thuộc phạm vi sau.

Bộ kiểm thử đạt **40 passed** và Ruff đạt. Chạy lại từ checkout sau khi cài `pip install -e '.[dev,sim]'`.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY pytest -q tests/unit tests/simulation
ruff check .
```
