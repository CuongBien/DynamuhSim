# So sánh chung ba framework: phản ứng và lựa chọn cho project

Ngày22/10/2026. Pilot ComplexityNav đã commit `c9dffe25`; bộ dữ liệu mới chạy trên HEAD này với source/input hashes riêng. **Đây là benchmark pilot offline, chưa phải cohort chung Gazebo/Nav2.** Không thay code robot hoặc AMCL/controller.

## Kết quả chính

36/36 lượt thực thi hoàn tất:6 cấu hình ×3 scene ×2 điều kiện robot. Có18 cặp far/near cùng initial fixture,6 repeat-prefix checks đều đạt. Tổng2880 update chính +120 update prefix =3000 update; không retry hoặc thay lượt có overlap bằng lượt đẹp.

| Cấu hình đã chạy | Case không ghi overlap>1mm /6 | Min người–người | Min robot–người | Min vật cản–người |
|---|---:|---:|---:|---:|
| SocNavGym native individual* |5/6 |0.263m |−0.186m |0.279m |
| SocNavGym guarded individual |6/6 |0.361m |0.037m |0.375m |
| CrowdSimPlus ORCA+ |6/6 |0.036m |0.027m |0.020m |
| CrowdSimPlus SFM |3/6 |−0.020m |0.115m |0.000m |
| ComplexityNav native mixed** |4/6 |0.021m |0.020m |−0.360m |
| ComplexityNav geometry mixed |6/6 |0.040m |0.040m |0.020m |

Các mẫu /6 gồm cả far controls, không phải success probability hoặc6 seed độc lập. Signed minima lưu nguyên;−0.36m của distance-to-solid trừ radius bị chặn khi tâm nằm trong box, không phải độ sâu xuyên tối đa.0.000m là chạm biên, không có social comfort margin. Khoảng hở dương2–4cm không tự chứng minh hành vi thoải mái hoặc body mesh không chạm.

*Soc native ở đây là wrapper giữ human step gốc, formation tắt, nhưng robot prediction nội bộ dựa dummy goal; không dùng measured input velocity để đặt agent velocity trong human ORCA. Cặp native/guard vì vậy đổi cả cách dự đoán robot và safety projection, không cô lập chỉ một nguyên nhân. **CN native không nhận nội vật cản vào human solver, giữ làm negative control cho warehouse compatibility, không coi là cấu hình triển khai hợp lệ.

![Min signed gaps — recorded offline data](minimum_gaps.png)

## Bài thử và phần đã khóa

6 người trong mỗi scene, human radius0.36m, nominal robot radius0.25m. Initial discs không overlap người/robot/42 vật cản, mọi goal nằm trong free space. Các goal đủ xa để không kỳ vọng vòng tuần tra hoàn tất trong10s; tiến độ là khoảng cách tới target giảm, không phải goal success.

- Opposed:6 người đi về phía đông, robot near đi ngược từ(−17,0) với0.5m/s trong8s rồi dừng.
- Crossing: người đi qua tim lối theo ba đường x=−26/−24/−22, robot near đi từ(−28,0) với0.75m/s trong8s rồi dừng.
- Blocked narrow: người đi hai phía của khe thật1.8m, robot near đứng ở(16,0), không chủ động nhường. Đây là bài phản ứng khi lối bị chắn, không đặt điều kiện mọi người phải đi qua được.

Far control của tất cả là robot đứng tại(28,18). Pose/velocity/yaw trace0.05s cùng file và SHA256 được dispatch tới các model; projection/metric robot interval dùng đủ chuyển động dự đoán. Timestep giữ0.1s cho Soc/CSP và0.25s cho CN, cùng endpoint so response0.5s. Không dựng fake sampling walls; Soc dùng factory60×40m với42 boxes, formation/dispersal tắt để so cá nhân.

Chỉ chuẩn hóa physical inputs/geometry, không chuẩn hóa native intended speed hoặc mọi quan sát nội bộ. CN là3 ORCA+3 SFM, các nhánh khác đồng nhất một policy; native speed CN chủ yếu0.5m/s, các nhánh khác cấu hình0.8m/s. Native Soc còn có actual peak khoảng0.852m/s, nên giá trị config không được gọi là hard ceiling. Không lấy raw travel/time để xếp hạng model. Radius đánh giá/fixture được chuẩn hóa, còn perceived radius và safety buffer nội bộ khác nhau: native Soc robot-agent mặc định0.36m, guarded projection0.255m, CN geometry projection0.27m. Không xem các clearance này là ablation chỉ thay thuật toán. Robot yaw trong scripted disc probe có thể đổi khi dừng; không nghiệm thu angular feasibility của robot hoặc chuyển trực tiếp trace này thành cmd_vel vật lý.

## Quan sát có đối chứng

SFM có human-human overlap lớn nhất khoảng2.03cm ở narrow far và1.71cm trong crossing; giữ nguyên kết quả soft-force, không thêm clipping giữa người để làm đẹp. ORCA+ không ghi overlap>1mm ở6 scene/control nhưng từng có1.6s tất cả người rất chậm trong crossing far đối xứng; đây là dấu hiệu có thể kẹt, không gọi là đi tự nhiên chỉ vì clearance dương.

Native Soc crossing near có robot–người gap−18.6cm. Guarded counterpart trong cùng input còn gap dương3.70cm; đây là thay đổi của cả adapter, không chứng minh upstream algorithm nói chung sai hoặc group/formation cũng đạt cùng kết quả. CN native xuyên machine ở blocked narrow; geometry counterpart không ghi gap âm>1mm, nhưng lớp cuối ORCA cũng làm nhánh SFM trở thành hybrid.

Near/far trajectories khác nhau ở cả18 cặp trên dữ liệu endpoint chung. Thời điểm đạt khác biệt vị trí>5cm qua hai mẫu liên tiếp được lưu, chỉ là ngưỡng phát hiện phản ứng trong trace, không phải thời gian nhận thức của người. Các kernel đo input cùng file nhưng observation API khác nhau: native Soc dự đoán robot về goal(3.5,1.5), CN dùng FullState có dummy goal(26,0), CSP dùng observable position/velocity. Guarded Soc/CN projection dùng velocity input thực. [Audit source/API](observation_api_audit.json) ghi rõ; không gọi là strict equal-observation native-algorithm ablation.

![Cùng input crossing gần robot — dữ liệu mô phỏng2D,không phải Gazebo,2×](crossing_near_2d_2x.gif)

GIF được vẽ từ trajectory thật đã chạy,21 mẫu0..10s mỗi0.5s, phát2×, giữ frame cuối0.25s; không phải camera Gazebo hoặc ảnh AI. Đối chiếu shape/time/inputs và finite swept gaps do analyzer GEOS riêng thực hiện. CSV giữ từng case, mọi minima và tick flags, không giấu contact bằng lấy trung bình.

## Chọn mô hình theo mục tiêu của bạn

| Mục tiêu | Đề xuất theo bằng chứng hiện có |
|---|---|
| Đi cùng nhóm, đứng tương tác, hình thành/tan nhóm | **Ưu tiên SocNavGym + adapter lifecycle đã nghiệm thu.** Native và bản sửa tách rõ; đã có Gazebo6 người hình thành/đi/tan, chưa chứng minh mọi nhóm đứng sẽ nhường robot. |
| Đối chứng tránh người độc lập, hình học nhà xưởng | **Giữ CrowdSimPlus ORCA+ làm baseline.**6 sample/control không overlap>1mm ởcohort này; không có luật giữ nhóm/hội thoại. |
| Crowd motion mềm, quan sát lực/né tương tác | SFM là phương án phụ có thể dùng để quan sát; công bố soft overlap, không coi là hard-body avoidance đã bảo đảm. |
| Nghiên cứu prediction/group detection hoặc alternative mixed policy | ComplexityNav geometry là biến thể phụ; native warehouse geometry không đạt, chưa chạy trained RGL và clustering không phải social formation. |

Đề xuất ưu tiên SocNavGym xuất phát chủ yếu từ năng lực nhóm đúng yêu cầu, không phải thắng một bảng điểm an toàn.6/6 của guarded individuals không được chuyển thành kết luận group/formation6/6: bằng chứng nhóm là [Gazebo lifecycle](../../../socnavgym_warehouse_lifecycle/reports/WAREHOUSE_LIFECYCLE_REPORT.md) riêng. [ComplexityNav Gazebo4 người](../../../complexitynav_gazebo/reports/GAZEBO_PILOT_REPORT.md) cũng là pilot riêng, radius0.3m/4 người, không gộp với cohort normalized0.36m/6 người này.

## Bằng chứng, giới hạn và phần còn lại

[Protocol](protocol.json) · [Validation](validation.json) · [Metrics CSV](metrics.csv) · [Near/far response](paired_response.json) · [Runtime commands](execution.json). Source snapshots/input SHA256, mọi trajectory và short repeat được lưu.6 mô hình chạy tuần tự khoảng410s wall; observations/harness/timestep khác nhau, không dùng wall time làm runtime ranking. Worker giới hạn2 CPU, container không mạng; không có rendering3D/GPU ởđợt offline này. GPU NVIDIA đã được kiểm chứng ở các pilot Gazebo riêng.

Chỉ một seed42 và10s/scene, chưa đủ kết luận dense crowd, deadlock dài hạn, subjective naturalness hoặc social comfort. Repeat20 native ticks là2s hoặc5s, không phải proof toàn episode deterministic. Flagged tick seconds không phải exact contact duration. Source/kết quả cũ và staging/outside worktree được bảo toàn.

Phần còn lại để chốt demo chung: lưu cohort này; đưa đúng fixture/config normalized vào một cohort Gazebo riêng, kiểm tra actual pose/scan và robot path thực rồi mới bật Nav2 so cùng scene nếu cần. Các native chẩn đoán thiếu geometry/velocity prediction không được âm thầm promote. Không thêm World Model, Isaac Sim, framework mới hoặc ma trận tuning AMCL.
