# ADR-0007 · Đồng bộ tác nhân ngoài trong JuPedSim

**Trạng thái · Đề xuất upstream, chưa triển khai.** Gate `BLOCKED_UPSTREAM_API` áp dụng cho wheel 1.4.2. Probe dựng lại SFM được giữ riêng với ID `sfm_reconstruction_probe`. [Kết quả TASK 04R](../reports/human_models/TASK_04R_REPORT.md)

## Đề xuất thay đổi tối thiểu

Nền upstream là tag `v1.4.2`, commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`. Wheel đang cài tự báo `80d0b3b1`. Bắt đầu với SFM. Mở rộng AVM/CSM cần thiết kế riêng cho state từng model. [Release](https://github.com/PedestrianDynamics/jupedsim/releases/tag/v1.4.2)

1. Thêm một danh sách tác nhân SFM do bên gọi sở hữu trong `libsimulator/src/Simulation.hpp` và `Simulation.cpp`. Mỗi tác nhân có một ID native ổn định, bán kính thật, pose và model-state velocity. Tại `Simulation::Iterate`, thêm bản sao của các actor này vào neighborhood search **dành cho operational step** sau khi lập chỉ mục người. Giữ actor external ngoài roster journey, tactical và danh sách được operational step cập nhật. Duy trì hai vòng tính và áp update hiện tại cho người.
2. Thêm một API theo batch, dự kiến `register_external_sfm_actor(radius)` và `apply_sfm_coupled_state(expected_iteration, external_samples, pedestrian_samples)`. Mỗi sample gồm ID, XY, heading và `vx/vy`. Chỉ nhận đúng boundary nguyên, đủ roster, đúng loại model và giá trị hữu hạn. Xác thực batch trước khi áp. Cập nhật pose và velocity người, giữ nguyên ID, journey, stage và các trường động học khác. Tạo lại chỉ mục từ snapshot đúng trước khi có truy vấn hoặc bước có thể đọc trạng thái cũ. Thao tác update/read không tăng clock.
3. Bổ sung C API trong `libjupedsim/include/jupedsim/` và `libjupedsim/src/simulation.cpp`. Expose từ binding native và wrapper `jupedsim.Simulation`, dùng kiểu giá trị và mã lỗi hiện có. Không expose con trỏ tùy ý hoặc setter từng field cho caller. Giữ API cũ tương thích và mã SFM `ComputeNewPosition`/`AgentForce` nguyên bản. [Nguồn Simulation](https://raw.githubusercontent.com/PedestrianDynamics/jupedsim/v1.4.2/libsimulator/src/Simulation.cpp) · [Nguồn neighborhood search](https://raw.githubusercontent.com/PedestrianDynamics/jupedsim/v1.4.2/libsimulator/src/NeighborhoodSearch.hpp) · [Nguồn operational step](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/OperationalDecisionSystem.hpp)
4. Validate initial overlap, outside/solid, radius, duplicate ID và coincident centers trước khi gọi lực native. Trả lỗi có ID, timestamp và clearance, giữ prefix đã xác nhận. Với overlap di động, bắt đầu bằng terminal có mã rõ. Profile phục hồi hoặc cho phép overlap hữu hạn cần test riêng. Waypoint giữ nguyên khi robot che phủ và log là bị che. Khi lấy được mẫu hợp lệ mới theo chính sách đã duyệt, kiểm re-entry cùng ID. Không suy đoán contact từ footprint.
5. Tầng bridge chỉ gửi trạng thái đo tại boundary. Ở các bước dự báo 10 ms giữa hai snapshot, cần một trajectory external được quy định, ví dụ ngoại suy từ vận tốc đo, và ghi sai số so với pose thật ở lần đo kế tiếp. Không đưa ground truth tương lai vào lệnh người. Giữ nhịp robot 50 ms, người 100 ms và physics 2 ms. Phương trình SFM 1.4.2 chỉ đọc vận tốc tương đối trong ma sát khi footprint chồng lấn, nên cổng này không biến SFM thành mô hình anticipation. [Nguồn SFM](https://raw.githubusercontent.com/PedestrianDynamics/jupedsim/v1.4.2/libsimulator/src/SocialForceModel.cpp)

## Cổng build và phát hành

- Pin commit nền và commit chứa thay đổi. Bản đóng gói thử nghiệm mang build ID riêng. Giữ LGPL-3.0-or-later, thông báo bản quyền, bản nguồn tương ứng và hướng dẫn build khi phân phối. Pin build inputs và compiler, xây wheel cp312 x86-64 từ checkout đã pin bằng packaging upstream trong môi trường tách biệt. Kiểm SHA-256 của source/patch và wheel, ghi tag ABI, compiler và `get_build_info()`. Wheel trong `.venv` của repo là đối chứng stock.
- Trước khi đánh giá robot, chạy kiểm tương thích ABI và lớp SFM, rồi đối chiếu từng tick người thường khi không có external actor với wheel stock. Kiểm đúng XY, velocity, ID, journey, stage sau batch hợp lệ và không đổi state/clock khi batch lỗi. Kiểm actor external đứng, tiến, ngang, lùi và quay mà không bị operational step sửa. Kiểm ID ổn định, map world, overlap/waypoint và re-entry, rebase sau limiter/guard, phân biệt contact native với clearance, replay, và giá trị không hữu hạn.
- Chỉ sau các bước trên mới tạo cấu hình nghiên cứu riêng có version và coupling profile. Kết quả của build tích hợp phải ghi provenance fork. TASK 04R chưa build hoặc chạy bản này. [Đặc tả bridge](../architecture/JUPEDSIM_MUJOCO_2D_3D_BRIDGE.md)

## TASK 04S · Biên thử nghiệm sample-and-hold

Cổng này đạt trên build local `1.4.2+task04s`. Stock wheel vẫn cần API upstream. Build và patch được tách riêng khỏi backend nghiên cứu.

Bản thử nghiệm có state SFM do MuJoCo cấp tại đầu mỗi callback người 100 ms. Trong mười bước dự báo 10 ms, neighborhood nhận cùng mẫu pose và vận tốc robot. Bước native chỉ tiến người. Mẫu hết hiệu lực sau bước thứ mười. Báo riêng sai số ghi/đọc ở boundary và độ cũ của mẫu so với pose MuJoCo đo được sau khi thực thi. Không dùng pose MuJoCo tương lai và không thêm coupling ở substep 2 ms.

Batch native đồng bộ lại XY và vận tốc người vào agent hiện có. Giữ orientation native, target, destination, journey, stage, tham số model và ID. Ở bước tiếp theo SFM và routing xử lý state này như bình thường. Đây là SFM được điều kiện hóa theo GT sau limiter và guard, nên quỹ đạo đóng vòng có thể khác SFM tự chạy liên tục dù phương trình giữ nguyên.

Native kiểm timestamp theo lưới 10 ms, số bước hiệu lực, roster, tọa độ hữu hạn, vùng đi được và tâm trùng. Overlap footprint hữu hạn giữa người và robot có nghĩa vật lý trong phương trình SFM và được giữ để kiểm thử ma sát. Ca MuJoCo có clearance âm được báo thất bại tương tác riêng với solver contact. [Báo cáo TASK 04S](../reports/human_models/TASK_04S_REPORT.md)

## TASK 05R · Dữ liệu scene và runtime

Lifecycle cấp `PlanarWorld` tùy chọn trong `HumanBackendConfig` từ cùng collision projection đã load trong adapter. Backend `sfm_jupedsim` dùng bản build cô lập 04S, không có đường fallback sang proxy tự tích phân. Profile giữ mẫu nhận callback 100 ms hoặc 50 ms, tương ứng 10 hoặc 5 bước native 10 ms. Cả hai dùng limiter và guard chung của MuJoCo. SFM dùng waypoint native và tốc độ mong muốn đã khai báo. Thời điểm vào dung sai đích được đo riêng, không gắn luật dừng ngoài SFM.

## TASK 04U · API dùng chung

Giữ phương trình operational của ba model và bổ sung API C/Python chung ở simulation. Mỗi model dùng đúng variant khi actor được thêm vào operational index. Ghi pose robot ngoài MuJoCo chỉ vào bộ dự báo, giữ robot khỏi roster cập nhật. Rebase ghi đúng state cần thiết của mỗi model và giữ nguyên identity, journey, stage, tham số và RNG. CSM V2 có hướng nhưng không có state vận tốc native. AVM dùng hướng và vận tốc đo của robot trong phương trình anticipation hiện có. Build 04U phát hành dưới dạng wheel Linux local có tag ABI riêng, với source archive, hai patch và fingerprint đi kèm trong hồ sơ build. [Hợp đồng và phạm vi](../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)
