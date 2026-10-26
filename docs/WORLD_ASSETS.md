# World runtime, dataset và catalog lịch sử

Build mặc định không cài ba thư mục payload lịch sử vào package `custom_corridor`. Ba payload hiện được hydrate theo nhu cầu thay vì nằm sẵn ở HEAD; dữ liệu vẫn có trong revision Git đã pin và archive local. Chuyển này giảm số file/byte của checkout hiện tại, không giảm lịch sử clone Git. Xem [hydrate và kiểm chứng](HISTORICAL_ARTIFACTS.md#hydrate-vào-đường-dẫn-consumer).

Policy duy nhất nằm ở `src/custom_corridor/config/world_payloads.txt`, mỗi dòng là một đường dẫn tương đối dưới `worlds`:

- `school_arena/demo_1/dataset_v1.0`
- `school_arena/demo_1/scenario_generator/generated`
- `school_arena/demo_1/scenario_generator/generated_v1`

World runtime, model, launch, URDF, RViz, config và map vẫn ở vị trí cũ. `arena_dataset/arena_dataset.sdf` là world chạy mô phỏng, vẫn được cài; nó khác thư mục dữ liệu `dataset_v1.0`. Code/generator/config lịch sử ngoài ba payload trên vẫn được giữ. Trước build legacy, hydrate đủ ba nhóm theo hướng dẫn; CMake sẽ báo lỗi khi thiếu thư mục payload.

## Build và kiểm tra prefix mới

Từ source checkout tương ứng, tạo build/install base mới:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/runtime build \
  --base-paths src/custom_corridor src/custom_corridor_plugins \
  --build-base build/runtime --install-base install/runtime --symlink-install
source install/runtime/setup.bash
python3 tools/catalog_assets.py verify-install --fresh \
  --share-dir "$PWD/install/runtime/custom_corridor/share/custom_corridor" \
  --output "$PWD/experiments/social_navigation/benchmark/generated/runtime_install_audit.json"
```

Helper `tools/process_runtime.sh` được cài cạnh `school_runtime.sh` để launcher School hoạt động từ installed share và khi chép vào container. Install audit đối chiếu helper này với Git blob nguồn. Cache Python không được chép từ source vào package; `--fresh` kiểm tra thêm prefix mới không có cache. Bỏ `--fresh` khi kiểm tra prefix đã chạy Python, vì runtime có thể tự tạo bytecode.

Copy install và symlink install cùng dùng policy. Công cụ so từng asset committed với Git blob/SHA256 và phát hiện payload còn sót trong prefix. Không chỉ kiểm tra vài tên file hoặc tin flag build.

**CMake không xóa asset đã cài trước đây.** Chuyển option OFF rồi build lại cùng prefix cũ không dọn dataset cũ; dùng install base mới và audit. Đợt này không tự xóa hay sửa install/build của checkout chính.

## Cách cài đầy đủ cho lịch sử

Nếu một workflow cần cả payload trong installed share:

```bash
colcon --log-base log/legacy build --packages-select custom_corridor \
  --build-base build/legacy --install-base install/legacy \
  --cmake-args -DDYNAMUHSIM_INSTALL_LEGACY_WORLD_PAYLOADS=ON
python3 tools/catalog_assets.py verify-install --legacy \
  --history-index docs/artifacts/history_2026-10-25.json \
  --share-dir "$PWD/install/legacy/custom_corridor/share/custom_corridor" \
  --output "$PWD/experiments/social_navigation/benchmark/generated/legacy_install_audit.json"
```

Đây là tùy chọn tương thích, không bật mặc định. Generator và production loader cũ dùng đường dẫn tương đối với source/config: `configs/dataset_v1.yaml` trỏ `output_dir: ../dataset_v1.0`; `launch_episode.sh` nhận episode path từ người gọi. Chạy luồng lịch sử từ source checkout hoặc full legacy install tương ứng, không coi một runtime prefix thiếu payload là bản dataset hoàn chỉnh.

## Catalog không phải payload backup

```bash
STORE="$PWD/experiments/social_navigation/benchmark/generated/artifact_store"
python3 tools/catalog_assets.py create --store "$STORE" \
  --ref e96e3108bb9ac954403b747e03ac8c5346d6560a \
  --output "$PWD/experiments/social_navigation/benchmark/generated/asset_catalog.json"
python3 tools/catalog_assets.py verify --store "$STORE" \
  --catalog "$PWD/experiments/social_navigation/benchmark/generated/asset_catalog.json" \
  --output "$PWD/experiments/social_navigation/benchmark/generated/asset_catalog_verify.json"
```

`--ref` chọn revision cho create/install audit, mặc định HEAD và luôn được resolve thành full commit. Catalog đọc committed Git objects, không trộn working tree dirty hoặc file untracked. Mỗi payload có Git tree, Git blob ID/size/mode và SHA256 từng file; full inventory nén được lưu theo hash trong `catalogs/objects/` của store. Index gọn có thể giữ trong Git. Ví dụ trên pin mốc còn đầy đủ payload; catalog mới ở HEAD chỉ mô tả dữ liệu còn committed tại HEAD, không tự nối lại dữ liệu đã externalize. Dùng catalog/index lịch sử đã pin cho backup và hydrate.

Catalog bao phủ ba world payload và các nhóm `benchmark/**/reports`. Nó ghi những gate runtime đang dùng, consumer source-relative đã xác định và các source map JSON đã freeze trỏ vào nhóm. Đây là danh sách consumer đã biết, không phải phân tích dependency động đầy đủ.

Mọi entry hiện là `inventory_only`, `payload_externalized=false`, `removal_allowed=false`. Verify từ chối index đã tự đổi cờ cho phép xóa, thiếu group/file, sai tree/consumer hoặc sai SHA256. Muốn chuyển payload ra Git cần archive + roundtrip riêng, giữ gate nhỏ và làm rõ đường hydrate cho consumer lịch sử. Không dùng công cụ `evict-raw` cho dữ liệu tracked; công cụ đó vẫn chặn tracked files.

## Phạm vi kiểm chứng

Build thực được kiểm tra ở prefix cô lập, so đủ asset bytes và kiểm tra package/launch resolve. Đây không phải lượt chạy vật lý/Nav2 mới. Các report/freeze, split, outcome và dataset01 giữ nguyên. Biên bản và catalog đã đối chiếu sẽ nằm trong `docs/verification/` và `docs/artifacts/`.

## Kết quả kiểm chứng build runtime trước khi externalize (25/10/2026)

[Biên bản](verification/world_assets_2026-10-25.json) · [Catalog đã pin revision](artifacts/catalog_2026-10-25.json).

**64 tests PASS.** Ba build ROS2 Jazzy ở prefix mới đều thành công: runtime-copy, runtime-symlink và legacy-copy. Mọi asset được đối chiếu byte với Git; hai runtime prefix kiểm thêm package resolution, Xacro, hai launch descriptions và ba executable mỗi prefix.

| Phạm vi asset committed | Runtime copy/symlink | Full legacy copy |
|---|---:|---:|
| File trong worlds |173|11.997|
| Byte trong worlds |2.266.224|298.021.418|
| File trong7 thư mục asset |218|12.042|
| Byte trong7 thư mục asset |17.463.438|313.218.632|

Default package không có cả ba payload directory; world runtime dạng arena/corridor/school/hospital vẫn có đủ. Runtime-symlink thực sự có173 symlink world file; runtime-copy có0. Prefix cũ cài đầy đủ được kiểm và FAIL runtime audit vì còn payload, xác nhận cần prefix mới. Bản install legacy-copy tạo riêng cho kiểm thử đã được dọn sau audit; source và log/lệnh build được giữ.

Catalog gồm50 nhóm/14.912 file,535.425.398 byte logical; đã deep-verify SHA256 của9.291 Git blob duy nhất. Có7 gate hiện hành cần giữ, cùng các consumer/freeze reference đã xác định. Index49.657 byte giữ trong Git;50 full inventory nén (777.980 byte) ở store local. Đây là metadata, không phải535MB payload đã được backup/chuyển đi.

Mọi entry vẫn `inventory_only`, không cho xóa dữ liệu. Muốn dựng lại inventory cache từ clone có commit đã pin:

```bash
python3 tools/catalog_assets.py create \
  --ref e96e3108bb9ac954403b747e03ac8c5346d6560a --store "$STORE" \
  --output "$PWD/experiments/social_navigation/benchmark/generated/catalog_rebuilt.json"
python3 tools/catalog_assets.py verify --store "$STORE" \
  --catalog docs/artifacts/catalog_2026-10-25.json \
  --output "$PWD/experiments/social_navigation/benchmark/generated/catalog_rebuilt_verify.json"
```

3.454 file lịch sử được bảo vệ giữ nguyên byte; checkout/index chính không đổi. Không di chuyển payload tracked, không thay dataset01 hoặc rewrite Git history. Bước tiếp theo là archive + restore từng payload đã catalog và xử lý consumer source-relative/gate trước khi cân nhắc bỏ payload khỏi checkout Git.

## Archive payload lịch sử

Công cụ archive/verify/restore riêng đã bổ sung tại [Archive lịch sử](HISTORICAL_ARTIFACTS.md). Catalog gốc vẫn giữ nguyên trạng thái inventory; index backup lưu riêng và không cho phép tự xóa tracked payload.
