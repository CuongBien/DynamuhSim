# TASK 04U · Bridge native JuPedSim–MuJoCo dùng chung

**Cổng hạ tầng · `INFRASTRUCTURE_PASS`. Cổng tích hợp · `MODEL_INTEGRATION_PASS` cho SFM, AVM và CSM V2 trên wheel cô lập 04U.** Các cổng dùng fixture MuJoCo headless qua registry, đồng bộ robot/người, bước native, limiter, GT và solver contact thật. An toàn và hoàn thành mục tiêu chấm riêng theo fixture. Stock vẫn `BLOCKED_CAPABILITY` cho coupling và 04S có đường legacy SFM. [Hợp đồng](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Ma trận API](../../backends/JUPEDSIM_API_COMPATIBILITY.md)

## Checkout và provenance

- Nhánh `mujoco-hummodel`, HEAD `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, lệch upstream local 0/0. Index sạch. Working tree đã dirty trước 04U với 04R/04S/05R/06 và hai tệp `orca_progress_tools/`. SHA-256 của `HUMAN_MODEL_FINAL.md` trước và sau khớp HEAD `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Tệp MASTER trong `/tmp` vắng mặt. Dùng bản đính kèm đã áp dụng ở TASK 06.
- Đối chiếu AGENTS, nguồn nghiên cứu, các nhóm architecture/protocol/schema/backend/report, 04S/05R/06, deviation ledger, patch cũ, source archive, mã native, Python và MuJoCo. Nền JuPedSim `v1.4.2`, commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`, archive theo lock SHA-256 `a61ebf4f535474c0e9838bea98f0996f58d87bfc8a5ddb0ca60fb01816a7ec44`. Patch 04S `0d79faa084acf5b863a3944258499837b46f7f9ccae56d8119b3a555bab710e4`, patch 04U `fe1924d6e1a2fd0e64ca2c0b3fc08eb6e4de0399a2e59263979c8b70c695021b`. [Nguồn JuPedSim](https://github.com/PedestrianDynamics/jupedsim/tree/v1.4.2/libsimulator/src) · [Build sạch](../../../runs/task_04u/reproduction.log)
- Python 3.12.3 cp312 x86-64, MuJoCo 4.0.0, GCC 13.3.0, CMake 3.28.3. C++20, Release, Unix Makefiles, Unity batch 4, ba job, target `py_jupedsim`, tắt upstream test build và benchmark. Wheel headless `cp312-cp312-linux_x86_64`, SHA-256 `f3cf132b3987c368448c11913e1d11bb73e335032553e6222a043c0720437e3c`. Build kiểm ba tệp phương trình model từng byte so với archive. Venv riêng đọc dependency hiện có từ stock qua `.pth`, wheel cài local vào prefix riêng. `.venv` và `ai12` giữ nguyên. Hai build ở hai path cho hash binary/wheel khác nhau nhưng trace parity giống từng byte. [Provenance](../../../runs/task_04u/reproduction/provenance.log) · [Parity](../../../runs/task_04u/parity.log)

| Interpreter từ root repo | Distribution | SHA-256 extension đang import | Legacy SFM / universal |
| --- | --- | --- | --- |
| `.venv/bin/python` | `1.4.2` | `906db6cf6714c7cf9021ec2315c16bd9a54ad84edad30bc0a3ce44b9949f75bd` | Thiếu / thiếu |
| `runs/task_04s/reproduction/venv/bin/python` | `1.4.2+task04s` | `77257fc9679c3ac2691495c509ef444937301351c175525231628bc1ceae091f` | Có / thiếu |
| `runs/task_04u/reproduction/venv/bin/python` | `1.4.2+task04u` | `789bc8576238c4243e8ce393fcad882499ca03621434eb1ff0ef5313620e0e03` | Có / có |

[Đường dẫn import và symbol C](../../../runs/task_04u/abi-and-checkout.log)

## Nguyên nhân và thiết kế

| Vị trí | Phát hiện |
| --- | --- |
| Stock `python_bindings_jupedsim/agent.cpp::init_agent`, `python_modules/jupedsim/jupedsim/agent.py::Agent` | Wrapper và binding chỉ đọc position/orientation. SFM model state có setter velocity riêng, AVM thiếu accessor Python cho velocity nội bộ và CSM V2 không có field velocity trong state. |
| 04S `Simulation.cpp:66`, `:100`, `:135` | `RegisterExternalSfmActor` và `ApplySfmCoupledState` chỉ chấp nhận `SOCIAL_FORCE` ở 10 ms. Actor dùng `SocialForceModelData` và rebase gọi `std::get<SocialForceModelData>`. AVM và CSM V2 đều bị chặn ở native guard trước khi vào đồng bộ. Mở guard đơn lẻ vẫn sai variant. |
| `GenericAgent.hpp`, `NeighborhoodSearch.hpp`, `OperationalDecisionSystem.hpp::Run` | Agent có variant theo model. Grid sao chép object theo ô nên ghi state vào agent sống không tự cập nhật index. Dùng lại đường 04S tạo index người, thêm đúng variant external vào bản sao operational và chỉ áp update cho roster người. |
| `AnticipationVelocityModel.cpp::NeighborRepulsion` / `::UpdateDirection` | AVM đọc XY, hướng và vận tốc hai actor, kèm horizon. Rebase giữ RNG và tham số, cập nhật XY, hướng và velocity thực thi. |
| `CollisionFreeSpeedModelV2.cpp::GetSpacing` / `::NeighborRepulsion` / `::ApplyUpdate` | V2 đọc khoảng cách, bán kính và hướng di chuyển của người đang tính, rồi tính tốc độ mới từ spacing. Không đọc vận tốc robot hay giữ velocity state sau update. Rebase XY/hướng, lưu vận tốc robot riêng cho chẩn đoán. |
| 04U `Simulation.cpp:87`, `:153`, `:198`, `:307` | Registration, getter velocity tùy model, batch nguyên tử đủ roster và chèn actor vào index operational sau cập nhật. C API và pybind mở cùng hợp đồng. Ba phương trình native giữ nguyên. |

[Nguồn AVM](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp) · [Nguồn CSM V2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/CollisionFreeSpeedModelV2.cpp) · [Bản vá 04S](../../../experiments/social_navigation/patches/jupedsim_v1_4_2_task04s.patch) · [Bản vá 04U](../../../experiments/social_navigation/patches/jupedsim_v1_4_2_task04u.patch)

## Số đo

Các ca chính giữ 2 ms physics, 50 ms robot, 100 ms người, 10 ms native và snapshot GT 10 Hz. Sai lệch XY/vận tốc external ở boundary bằng 0 theo readback. Yaw kiểm modulo 2π ở dung sai `1e-12 rad`. XY người sau rebase khớp GT ở cả ba model và velocity native khớp ở SFM/AVM. CSM V2 đo velocity từ thực thi MuJoCo. Requested khớp sai phân dự báo, executed khớp sai phân GT. Requested và limited giữ qua tick robot xen giữa. Replay cùng seed có trace quỹ đạo và contact trùng chính xác. Chuyển external sang ô index khác đưa response trở lại đúng đối chứng xa.

| Model | Đổi robot xa thành tĩnh gần, chênh XY người sau 100 ms (mm) | Chỉ đổi robot `vx` từ 0 sang −0,7 m/s ở cùng XY/yaw, chênh XY (mm) | Advance/stop MuJoCo, phân kỳ XY người lớn nhất (m) | Clearance robot–người min advance / stop (m) |
| --- | ---: | ---: | ---: | --- |
| SFM | 1,258517 | 0 | 0,207998 | 0,139736 / 0,184257 |
| AVM | 55,714539 | 0,808423 | 0,029937 | 0,505704 / 0,528274 |
| CSM V2 | 83,305937 | 0 | 0,035497 | 0,168819 / 0,209880 |

AVM dùng tâm robot (1,20, 0,15) m trong phép cô lập vận tốc. Hai model còn lại dùng (0,85, 0,15) m. Phép thử theo model không dùng cột đầu để xếp hạng. Cửa sổ MuJoCo advance/stop dài 3 s, seed 11, cùng lịch lệnh tới 0,70 s, không contact. Hold lag max khoảng 0,034780 m trong cặp. Chênh dự báo–thực thi trước rebase trong cặp đạt max 0,002150 / 0,104446 / 0,105000 m cho SFM/AVM/CSM V2. Đây là chênh dự báo và thực thi, đo riêng với sai số ghi/đọc. [Test và số đo](../../../runs/task_04u/pytest-04u-final.log) · [Kiểm index](../../../runs/task_04u/pytest-native-index.log) · [Trace và manifest](../../../runs/task_04u/final-fixtures/)

## Kết quả kịch bản

Fixture đối đầu và giao cắt 8 s để hai người tiến tới goal, robot đỗ xa. Fixture cửa 0,72 m có hai phòng đủ chỗ chờ, robot dừng 2 s rồi tiến. Cửa yêu cầu cả người và robot tới đích, không contact hay clearance âm. Đây là tiêu chí kỹ thuật, tách khỏi Card benchmark. Cặp advance/stop chỉ đo cửa sổ đáp ứng. Clearance âm robot–người và solver contact giữ riêng. Người–tường và người–người chỉ có clearance với mask hiện có.

| Fixture | SFM | AVM | CSM V2 |
| --- | --- | --- | --- |
| Người–người đối đầu | `SCENARIO_SUCCESS`, clearance 0,000051 m | `SCENARIO_SUCCESS`, 0,942092 m | `SCENARIO_SUCCESS`, 0,150873 m |
| Người–người giao cắt | `SCENARIO_FAILURE`, hết 8 s, 0,085899 m | `SCENARIO_FAILURE`, hết 8 s, 0,000002 m | `SCENARIO_FAILURE`, hết 8 s, 0,003666 m |
| Cửa 0,72 m | `SCENARIO_FAILURE`, người chưa tới đích, clearance robot–người 0,172266 m | `SCENARIO_SUCCESS`, 0,218996 m | `SCENARIO_SUCCESS`, 0,083794 m |
| Hành lang 0,80 m, ca âm đi song song | `SCENARIO_FAILURE`, hết giờ, 0,005176 m và 0 mẫu solver | `SCENARIO_FAILURE`, −0,062310 m và 9 mẫu solver | `SCENARIO_FAILURE`, hết giờ, 0,128158 m và 0 mẫu solver |
| Probe contact chủ đích | Replay 05R, −0,062092 m và 610 mẫu solver | Ca hành lang 0,80 m ở trên | `desired_speed=0`, −0,061514 m và 20 mẫu solver |

Tổng hợp fixture 04U có 5 `SCENARIO_SUCCESS`, 8 `SCENARIO_FAILURE` và 9 cửa sổ chẩn đoán advance/stop/replay. Đối đầu, giao cắt và cửa không có contact solver. Sai lệch trước rebase lớn nhất trong cửa là 0,210 m ở CSM V2, còn readback đúng boundary. Dư đi song song của hành lang 0,80 m là −0,06 m với robot 0,18 m và người 0,25 m. Ca này giữ nguyên kết quả từng model. Probe CSM V2 tốc độ mục tiêu zero cô lập đường ghi solver khi robot vẫn nhận lệnh tiến. [Kết quả từng lượt](../../../runs/task_04u/summary.json)

Regression 05R chạy lại đủ 12 fixture trên 04U. Summary khớp từng byte, trace, contact và metrics khớp chính xác từng lượt. Giữ 6 PASS và 6 FAIL ở cổng kịch bản 05R. [Đối chiếu](../../../runs/task_04u/sfm-comparison.log)

Cổng bridge đạt ở miền input đã kiểm. Một footprint cắt solid tại mốc cập nhật, tâm trùng, timestamp lệch hoặc state ngoài geometry có thể bị từ chối và đóng episode theo lỗi backend. Sample hold không ngoại suy pose robot giữa callback. Footprint tròn theo profile repo. SFM/CSM V2 có thể phân kỳ advance/stop do robot đổi vị trí. Thử cùng vị trí không cho ảnh hưởng vận tốc trước overlap ở hai model này. Khả năng phản ứng với vận tốc của AVM không chứng minh an toàn cao hơn.

## Tệp thay đổi

- Native patch gồm `libsimulator/src/Simulation.{hpp,cpp}`, `libjupedsim/include/jupedsim/simulation.h`, `libjupedsim/src/simulation.cpp`, `python_bindings_jupedsim/simulation.cpp`, `python_modules/jupedsim/jupedsim/simulation.py`.
- Build `scripts/diagnostics/build_jupedsim_04u.sh`, `package_jupedsim_04u.py`. Runtime `humans/jupedsim_bridge/coupled_backend.py`, `humans/backends/jupedsim_{sfm,avm,csm}.py`, `humans/registry.py` và hai YAML AVM/CSM. Hạ tầng MuJoCo được dùng qua các cổng hiện có.
- Test `tests/integration/test_jupedsim_universal.py`, cập nhật test lịch sử AVM/04S để phân biệt API legacy và provenance, MJCF cửa hẹp, script parity `experiments/social_navigation/jupedsim_04u_parity.py`.
- Báo cáo này, hợp đồng external actor, kiến trúc bridge, ma trận compatibility, ADR-0007, tài liệu AVM/SFM, schema, progress và deviation ledger. Log, trace và wheel dưới `runs/task_04u/`.

## Tái lập và kiểm định

```bash
bash scripts/diagnostics/build_jupedsim_04u.sh \
  runs/task_04s/jupedsim-1.4.2.tar.gz runs/task_04u/reproduction
P=runs/task_04u/reproduction/venv/bin/python
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs -s \
  --basetemp runs/task_04u/final-fixtures tests/integration/test_jupedsim_universal.py
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B \
  -m narrow_nav.simulation.smoke --ticks 10 humans=avm_jupedsim simulation.human_visuals=proxy
```

Build yêu cầu thư mục đích mới. Giữ archive và hai patch nếu lưu hoặc phân phối wheel. Script cài với `.venv/bin/python -B -m pip --python <venv-04u>/bin/python install --no-index --no-deps --ignore-installed --no-compile <wheel>`. Wheel headless, tag Linux local và license LGPL-3.0-or-later. Trace lưu seed đã derive, Git, phiên bản, binary hash, config đã resolve và fingerprint MJCF.

Mọi lệnh pytest và smoke chạy headless với `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY`. `P` là Python 04U ở trên, `S` là `.venv/bin/python`, `L` là `runs/task_04s/reproduction/venv/bin/python`.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `bash scripts/diagnostics/build_jupedsim_04u.sh runs/task_04s/jupedsim-1.4.2.tar.gz runs/task_04u/reproduction` | 0 | Build từ archive sạch, cài wheel vào venv riêng. [Log](../../../runs/task_04u/reproduction.log) |
| `-B -m experiments.social_navigation.jupedsim_04u_parity` bằng S/L/P, rồi `cmp` JSON | 0 | 300 bước free walk cho mỗi model. Cả ba trace khớp từng byte trên stock, 04S và 04U. [Hash](../../../runs/task_04u/parity.log) |
| `$P -B -m pytest -q -rs -s --basetemp runs/task_04u/final-fixtures tests/integration/test_jupedsim_universal.py` | 0 | 19 đạt, 2 skip dành cho runtime cũ. [Log](../../../runs/task_04u/pytest-04u-final.log) |
| `$P -B -m pytest -q -rs --basetemp runs/task_04u/full-fixtures` | 0 | 238 đạt, 4 skip. Hai test từ chối AVM/CSM chỉ áp dụng cho runtime cũ. Hai skip còn lại kiểm fingerprint và cổng stock. Các test coupling 04U chạy đủ. [Log](../../../runs/task_04u/pytest-04u-full.log) |
| `$S -B -m pytest -q -rs` | 0 | 214 đạt, 23 skip. Stock chạy cổng từ chối AVM/CSM. Các skip thuộc đường native 04U và legacy SFM thử nghiệm. [Log](../../../runs/task_04u/pytest-stock-full.log) |
| `$L -B -m pytest -q -rs tests/integration/test_jupedsim_universal.py tests/integration/test_avm_bridge_capability.py` | 0 | 5 đạt, 19 skip cần wheel 04U. 04S tái xác nhận từ chối backend AVM/CSM và legacy native API từ chối AVM. [Log](../../../runs/task_04u/pytest-04s-gates-final.log) |
| `$P -B -m pytest -q -s tests/integration/test_jupedsim_universal.py -k native_external` | 0 | 3 đạt sau khi bổ sung xác minh index ở boundary kế. [Log](../../../runs/task_04u/pytest-native-index.log) |
| `$P -B -m pytest -q -rs tests/integration/test_jupedsim_external_actor.py tests/integration/test_sfm_research_backend.py` và cùng bộ trên `$L -B -m pytest -q` | 0 | Mỗi build 9 đạt, 1 skip dành cho stock. [04U](../../../runs/task_04u/pytest-sfm-04u-final.log) · [04S](../../../runs/task_04u/pytest-04s-regression.log) |
| `$P -B -m experiments.social_navigation.evaluate_sfm --output runs/task_04u/sfm-05r-regression` | 0 | 12 lượt, 6 đạt và 6 lỗi kịch bản giữ nguyên. [Log](../../../runs/task_04u/sfm-05r-fixtures.log) |
| `$P -B -m narrow_nav.simulation.smoke --ticks 10 humans=<id> simulation.human_visuals=proxy` cho cả ba ID | 0 | 250 bước physics mỗi lượt. [SFM](../../../runs/task_04u/smoke-sfm.json) · [AVM](../../../runs/task_04u/smoke-avm.json) · [CSM V2](../../../runs/task_04u/smoke-csm.json) |
| `nm -D` trên extension, kiểm SHA-256 và RECORD wheel, `tar -xOf …` rồi `cmp` ba tệp phương trình | 0 | Export 4 symbol 04U, RECORD 33 entry hợp lệ và nguồn model trùng archive. [ABI](../../../runs/task_04u/abi-and-checkout.log) · [Wheel](../../../runs/task_04u/wheel-check.log) · [Nguồn](../../../runs/task_04u/model-source.log) |
| `.venv/bin/ruff check src tests experiments/social_navigation scripts/diagnostics`, `bash -n scripts/diagnostics/build_jupedsim_04u.sh`, `git diff --check` | 0 | Kiểm phạm vi đạt. [Log](../../../runs/task_04u/ruff-scoped.log) |
| `.venv/bin/ruff check .` | 1 | 7 lỗi có sẵn trong hai tệp `orca_progress_tools/`. [Log](../../../runs/task_04u/ruff-root.log) |

Giữ các log thử đầu. Phép cô lập AVM ở (0,85; 0,15) m có chênh 0,003245 mm dưới ngưỡng kỹ thuật 0,01 mm. Theo công thức native, khoảng cách dự đoán ở trạng thái đầu được chặn về 0 trong cả hai nhánh. Phép cuối chuyển cặp test AVM ra (1,20; 0,15) m, giữ nguyên tham số native. Assertion thử ban đầu dự đoán contact cho cả ba model ở hành lang âm, nhưng SFM và CSM V2 dừng ở giới hạn thời lượng. Kết quả đó được giữ và bổ sung probe CSM V2 đứng yên để đo đường solver riêng. Test legacy ban đầu áp fingerprint 04S lên wheel 04U. Bản cuối giữ kiểm fingerprint cho interpreter 04S và chạy assertion legacy API trên cả hai. [Lượt 04U đầu](../../../runs/task_04u/pytest-04u-initial.log) · [Ca âm thử đầu](../../../runs/task_04u/pytest-targeted.log) · [Legacy thử đầu](../../../runs/task_04u/pytest-sfm-04u.log)
