# TASK 04S · External actor và đồng bộ trạng thái SFM native

**Cổng 04S · PASS cho bản thử nghiệm `1.4.2+task04s`.** Robot external giữ đúng mẫu pose/vận tốc MuJoCo qua mười bước dự báo. Native rebase cập nhật XY/vận tốc người tại chỗ, giữ ID và journey. Wheel JuPedSim 1.4.2 chính thức vẫn thiếu API này. Profile `sfm_external_hold_probe` dùng lifecycle/limiter/MuJoCo hiện có. Ca hành lang hẹp vẫn báo thất bại tương tác do contact và clearance âm. [ADR-0007](../../decisions/ADR-0007-jupedsim-external-actor.md) · [Patch thử nghiệm](../../../experiments/social_navigation/patches/jupedsim_v1_4_2_task04s.patch)

## Nguồn, ABI và phạm vi

- Nhánh `mujoco-hummodel`, HEAD trước/sau `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, upstream local 0/0. Checkout bắt đầu với thay đổi 04R và hai file `orca_progress_tools/` đang dirty. Index sạch. Hash `HUMAN_MODEL_FINAL.md` trước/sau khớp HEAD `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.
- Release `v1.4.2`, commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`. Archive PyPI SHA-256 `a61ebf4f535474c0e9838bea98f0996f58d87bfc8a5ddb0ca60fb01816a7ec44` theo lockfile. Sáu tệp làm nền patch khớp byte-for-byte với commit đã pin. `SocialForceModel.cpp` của build giống byte-for-byte với tag, SHA-256 `53569b2b310f84a855fd01ee2702792352acef6c553a39c356eed8650b44a2b3`. [Release](https://github.com/PedestrianDynamics/jupedsim/releases/tag/v1.4.2) · [Đối chiếu nguồn](../../../runs/task_04s/tag-compare.log)
- C++20, GCC 13.3.0, CMake 3.28.3, CPython 3.12.3. Vendored Boost 1.82.0, CGAL 6.0.1, fmt 10.2.1, GLM 1.0.1 và pybind11 2.12.0 từ archive đã hash. CMake Release, Unity batch 4, ba job, target `py_jupedsim`, tắt upstream test build và benchmark. Venv thử nghiệm đọc dependency runtime từ site-packages hiện có qua `.pth` riêng. MuJoCo 4.0.0, Shapely 2.2.0, NumPy 2.5.4, Hydra 1.3.8 và `pyrvo2` 0.0.0. `.venv`, `ai12`, wheel chính thức và lockfile giữ nguyên.
- Kiểm wrapper và extension đều import dưới experimental interpreter prefix. Bản local mang distribution `1.4.2+task04s`, ABI cp312 x86-64 ELF. Script đặt extension trong venv riêng, không phát hành wheel. Native version giữ 1.4.2. Git build info là `UNKNOWN` do archive thiếu `.git`. Provenance ghi commit nguồn, patch hash, native extension và thư viện tĩnh. Ba symbol C mới export. Header dùng được với C++20, struct mới 40 byte. C11 trả exit 1 trên cả nguồn gốc và bản thử nghiệm do các khai báo C++ đã có trong header upstream. [Build provenance](../../../runs/task_04s/reproduction/provenance.log) · [Log ABI](../../../runs/task_04s/abi-final.log)
- Đã đọc AGENTS, nguồn nghiên cứu, kiến trúc và schema, bridge/test, TASK 04/04R, ADR-0007 và các lớp C++/C/pybind/Python liên quan. Thêm patch sáu tệp, script build, probe và sáu test tích hợp. Bổ sung 04S vào ADR, ledger và tiến độ. MuJoCo native vẫn nằm sau adapter. [Script build](../../../scripts/diagnostics/build_jupedsim_04s.sh) · [Probe](../../../experiments/social_navigation/probe_jupedsim_external_actor.py) · [Tests](../../../tests/integration/test_jupedsim_external_actor.py)

## Audit và thiết kế

| Lớp | Điểm kiểm tra |
| --- | --- |
| State | `libsimulator/src/GenericAgent.hpp` giữ ID, position, orientation, target, destination, journey, stage và model variant. `SocialForceModelData.hpp` giữ velocity và tham số. Rebase chỉ ghi position/velocity vào object đang sống. |
| Neighbor | `NeighborhoodSearch.hpp` lưu bản sao state theo ô. Batch cập nhật lại chỉ mục người. `Simulation::Iterate` thêm external vào bản sao chỉ mục dành cho operational. |
| Tích phân | `OperationalDecisionSystem.hpp::Run` tính update rồi mới áp. Chỉ roster người nhận update. `SocialForceModel.cpp::ComputeNewPosition`, `ApplyUpdate` và lực SFM giữ nguyên. |
| Journey | `Simulation.cpp::Iterate` chạy removal, neighborhood, stage, strategical, tactical rồi operational. `StageSystem.hpp`, `StrategicalDesicionSystem.hpp` và `TacticalDecisionSystem.hpp` tiếp tục quyết định stage/destination cho người. External actor không có journey/stage. |
| Binding | Stock `libjupedsim/src/agent.cpp` và `python_bindings_jupedsim/agent.cpp` chỉ expose getter pose. `libjupedsim/src/social_force_model.cpp` và pybind có setter velocity SFM. API 04S thêm vào `libjupedsim/include/jupedsim/simulation.h`, `libjupedsim/src/simulation.cpp`, `python_bindings_jupedsim/simulation.cpp` và `python_modules/jupedsim/jupedsim/simulation.py`. |

Batch nhận timestamp ns, 1–10 bước hiệu lực và đủ roster. Từ chối sai lưới, cũ/lặp, ID sai, giá trị không hữu hạn, footprint giao tường và tâm gần trùng theo `Point::isZeroLength` của upstream. Xác thực bản sao và chỉ mục trước khi áp. Input lỗi giữ state/clock. Overlap người–robot hữu hạn được giữ để quan sát lực và contact. [Simulation 1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/Simulation.cpp) · [SFM 1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/SocialForceModel.cpp)

## Đồng bộ, causality và continuity

Callback người 100 ms lấy `HumanContext` hiện hành, dùng pose và vận tốc world XY, giữ mẫu qua mười bước native 10 ms. Robot control 50 ms và physics 2 ms giữ nguyên theo cấu hình thực. Mỗi lượt 3 s có 30 callback, 300 bước native, 60 tick robot và 1.500 bước MuJoCo. Requested khớp sai phân dự báo. Limited giữ qua hai tick. Executed khớp sai phân GT. So sánh round-off của phép nhân/chia dùng dung sai tuyệt đối `1e-15 m` ở thang mét. Readback và counter dùng so sánh chính xác.

Có **2.970** lần readback robot, 330 lần mỗi lượt. Sai số đồng bộ pose max/mean/p99 đều **0 m**, kèm khớp velocity, ID, bán kính, timestamp và cờ kinematic. Hold lag so với MuJoCo đo sau bước là số khác, max **0,034780 m**. p99 dùng nearest rank. Hai nhánh khác lệnh robot tại 0,70 s vẫn có cùng đề xuất người ở callback đó vì snapshot đầu khoảng giống nhau. GT mới chỉ đi vào callback kế tiếp và kênh đo lag.

Rebase nhận lại XY/vận tốc đúng bằng GT. Domain ID, native ID, journey và stage giữ nguyên trong fixture một waypoint. Test hai waypoint giữ cùng handle, target, orientation, mass và reaction time qua rebase, rồi stage chuyển bình thường ở bước kế. Chênh dự báo–GT trước rebase đạt 0,002150 m ở advance và 0,388664 m ở ca có guard. SFM đóng vòng nhận state thực thi mới sau limiter/guard. Rebase giữ intent đã có, kể cả khi transition native xảy ra trong cửa sổ dự báo.

## Bốn nhánh robot và contact

| Nhánh đối đầu seed 11 | Vận tốc world cuối vx / vy (m/s) | Clearance robot–người min (m) | Hold lag max / mean / p99 (m) |
| --- | --- | ---: | --- |
| Advance | 0,355864 / 0 | 0,139736 | 0,034780 / 0,024851 / 0,034780 |
| Stop | −0,001284 / 0 | 0,184257 | 0,034780 / 0,006191 / 0,034780 |
| Yield | 0,012288 / −0,211231 | 0,322108 | 0,034780 / 0,017477 / 0,034780 |
| Reverse | −0,263404 / 0 | 0,229704 | 0,034780 / 0,018293 / 0,034780 |

- Requested người giữa advance/yield chênh tối đa **0,582591 m/s**. Phép thử một bước giữ vận tốc robot bằng zero cho vận tốc X người −0,013858 m/s khi cách 0,60 m và +0,015983 m/s ở 1,20 m. Giữ XY tại 0,60 m, đổi vận tốc robot Y từ 0 lên 0,40 m/s không đổi output. Tại overlap 0,35 m, đổi cùng vận tốc tăng Y người 0,96 m/s do ma sát tiếp tuyến của SFM gốc. [Mã AgentForce và ForceBetweenPoints](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/SocialForceModel.cpp#L133-L173)
- Lặp seed 11 tái lập GT người và tạo episode ID mới. Seed 12 tạo seed stream khác và cùng trace trong probe xác định. Giao cắt 90° cho vận tốc Y world 0,355707 m/s từ lệnh X body.
- Ca hẹp báo `interaction_failure`, giữ 32 mẫu clearance âm với min **−0,062092 m**, **610** sự kiện solver substep `robot_human` và **822** substep guard. Probe tiếp tục trong ngân sách 3 s để ghi contact thật. Mask solver người–tường/người–người giữ `not_observable_solver`. [Log paired-action](../../../runs/task_04s/paired-final.json)

## Chạy lại và nghiệm thu

Từ root repo, đặt archive từ URL pin trong `uv.lock` dưới `runs/task_04s/`. Build script kiểm hash và yêu cầu thư mục đích mới. Giữ license LGPL-3.0-or-later và source archive cùng patch nếu phân phối build.

```bash
bash scripts/diagnostics/build_jupedsim_04s.sh \
  runs/task_04s/jupedsim-1.4.2.tar.gz runs/task_04s/reproduction
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY \
  runs/task_04s/reproduction/venv/bin/python -B \
  -m experiments.social_navigation.probe_jupedsim_external_actor
```

| Kiểm tra | Exit | Bằng chứng |
| --- | ---: | --- |
| Build sạch bằng script trên | 0 | `reproduction.log`, `reproduction/provenance.log`. |
| Build lại cùng lệnh vào `runs/task_04s/rebuild`, chạy parity và probe lần hai | 0 | Hai build cho trace/response giống nhau sau khi bỏ provenance và ID episode/run sinh mới. SHA-256 của hai binary khác nhau. `rebuild.log`, `rebuild-comparison.log`. |
| Chạy probe online như trên | 0 | `paired-final.json`. Tám fixture hoàn thành, một thất bại tương tác được giữ. |
| Stock và experimental chạy `python -B -m experiments.social_navigation.probe_jupedsim_external_actor --parity`, rồi `cmp` | 0 | 300 bước, hai người. ID/stage, XY, velocity và orientation khớp byte-for-byte. Cả hai trace SHA-256 `1e07106879bf20fce6cbe4f0a98358e79be31a2e86e92740090f3565ad209789`. |
| Experimental `python -B -m pytest -q tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py tests/integration/test_jupedsim_external_actor.py` | 0 | 23 passed, gồm 17 test bridge và 6 test 04S. `pytest-experimental.log`. |
| Stock `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -m pytest -q` | 0 | 199 passed, một skip của module yêu cầu bản thử nghiệm. `pytest-stock-full.log`. |
| ABI `c++ -std=c++20 -fsyntax-only`, `file`, `nm -D` và kiểm import hai môi trường | 0 | `abi-final.log`. Log gồm cả C11 exit 1 trên upstream và patch. |
| Ruff phạm vi, format, `bash -n`, `git diff --check` | 0 | Kiểm mã, shell và diff trong phạm vi. |
| `.venv/bin/ruff check . --output-format concise` | 1 | Bảy lỗi ở hai file `orca_progress_tools/` ngoài phạm vi. `ruff-root.log`. |

Giữ log hai lỗi test ban đầu về kiểu iterator và tuple trả về. Các assertion đồng bộ, causality và overlap vẫn chạy. Cổng 04S đạt trên profile sample-and-hold đã đo. Stock wheel vẫn `BLOCKED_UPSTREAM_API`. Backend nghiên cứu 05R chưa đăng ký. [Tiến độ](PROGRESS.md) · [Deviation](../../research/HUMAN_MODEL_DEVIATIONS.md)
