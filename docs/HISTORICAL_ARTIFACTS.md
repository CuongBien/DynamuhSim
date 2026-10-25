# Archive và khôi phục payload lịch sử

`tools/archive_history.py` đóng gói **toàn bộ nhóm trong catalog đã pin**, đọc committed Git blobs thay vì working tree. Công cụ archive không xóa, di chuyển hoặc ghi đè dataset/report nguồn. Ba world payload đã được chuyển khỏi HEAD ở bước hydrate tiếp theo, mô tả phía dưới. Capture ON/OFF/MCAP riêng vẫn dùng `tools/artifact_store.py`.

## Ba thao tác

```bash
STORE="$PWD/experiments/social_navigation/benchmark/generated/artifact_store"
python3 tools/archive_history.py pack --store "$STORE" \
  --catalog docs/artifacts/catalog_2026-10-25.json \
  --output experiments/social_navigation/benchmark/generated/history_index.json
python3 tools/archive_history.py verify --store "$STORE" \
  --index experiments/social_navigation/benchmark/generated/history_index.json \
  --output experiments/social_navigation/benchmark/generated/history_verify.json
python3 tools/archive_history.py restore --store "$STORE" \
  --object-id OBJECT_ID_FROM_INDEX \
  --output experiments/social_navigation/benchmark/generated/restored_history_GROUP
```

Mỗi `--output` phải chưa tồn tại. Index liệt kê `id`, `root`, `object_id`, số file/byte và archive bytes cho từng nhóm. Pack kiểm catalog với revision, tree và SHA256 từng Git blob trước khi archive. Thay đổi local và file untracked không được đưa vào backup; chúng vẫn giữ nguyên tại chỗ.

## Định dạng và tính di động

| Thành phần | Nội dung |
|---|---|
| `history/objects/<SHA256>.tar.gz` | Payload thật và hai tệp metadata; tên archive là hash của toàn bộ archive |
| `history/manifests/<SHA256>.json` | Inventory, mode, kích thước, SHA256 và Git blob từng tệp; danh tính nhóm và catalog |
| Index | Catalog gốc được pin và ánh xạ đủ mọi nhóm sang archive |
| `RESTORE/payload/<repository-relative-root>/...` | Dữ liệu đã khôi phục tại đúng cấu trúc đường dẫn tương đối |
| `RESTORE/.artifact/` | Inventory và provenance gắn trong archive |
| `RESTORE/restore_receipt.json` | Chỉ được tạo sau khi kiểm lại toàn bộ file đã ghi ra đĩa |

Mang theo **index + cả `history/objects/` và `history/manifests/`** để kiểm/restore trên máy khác. Không cần Git objects, catalog inventory cache, ROS hoặc các producer cũ cho hai thao tác này; vẫn cần Python và mã công cụ hiện tại. Store hiện là local, chưa có bản sao remote được xác nhận.

Archive có thứ tự ổn định, mtime/uid/gid cố định, gzip không mang timestamp/tên tệp. Cùng dữ liệu và cùng toolchain tạo cùng object; pack lại không nhân đôi object. Không cam kết byte archive giống nhau giữa mọi phiên bản Python/zlib.

Verify kiểm hash toàn archive, metadata gắn trong archive, coverage từng member, regular-file type, kích thước, mode, SHA256 và Git blob. Restore giải nén thủ công vào thư mục mới, rồi đọc lại file từ đĩa để so hash/mode/coverage. Không dùng `extractall`, không chạy/import các Python snapshot trong payload. Đường dẫn đi ra ngoài, member lặp/thiếu, hardlink và symlink đều bị từ chối; hiện toàn bộ 14.912 payload committed là regular file. Executable mode và file rỗng được hỗ trợ và có test.

## Ranh giới trước khi bỏ payload khỏi Git

Catalog gốc tiếp tục là `inventory_only`; index backup và receipt vẫn có `removal_allowed=false`. Có backup không đồng nghĩa consumer đã được chuyển sang hydrate, và công cụ không cung cấp lệnh xóa tracked data.

- Bảy gate/protocol JSON đang dùng phải còn ở đường dẫn hiện hành.
- Dataset school và scenario generator vẫn có consumer source-relative; cần đường hydrate tương ứng trước khi bỏ payload gốc.
- Source map/freeze lịch sử có thể trỏ vào Python snapshot trong reports. Restore dữ liệu bảo toàn byte; tái chạy còn cần source revision, dependencies và môi trường tương ứng. Đây chưa phải kiểm chứng replay mọi experiment.
- `dataset01` của WM không thuộc đợt chuyển này. Không thay split, nhãn, outcome hoặc freeze.
- Bỏ file trong commit mới chỉ giảm checkout hiện tại. Giảm clone/history là việc riêng; đợt này không rewrite lịch sử Git.

Muốn chạy consumer cũ, chuẩn bị **checkout replay riêng** ở revision đã ghi, đối chiếu payload khôi phục rồi hydrate vào đúng root. Công cụ hiện chỉ xuất thư mục restore mới; chưa tự chép đè vào checkout. Bước tiếp theo là thêm kế hoạch hydrate/kiểm tra consumer, giữ gate nhỏ, rồi chuyển từng nhóm đã xác nhận.

## Kết quả kiểm chứng ngày 25/10/2026

[Index archive](artifacts/history_2026-10-25.json) · [Biên bản](verification/historical_archive_2026-10-25.json).

**75 tests PASS**. Toàn bộ 50 nhóm đã pack, verify và restore: 14.912 tệp, 535.425.398 byte payload. Mỗi bản restore được đọc lại để so SHA256, mode và coverage; 100 tệp metadata kèm theo cũng được kiểm. Không gọi Git/process khi verify và restore trong lượt kiểm này.

| Nhóm | Số tệp | Payload bytes | Archive bytes |
|---|---:|---:|---:|
| School `dataset_v1.0` |10.120|272.955.268|188.427.447|
| Scenario `generated` |917|14.207.921|905.906|
| Scenario `generated_v1` |787|8.592.005|579.084|
| 47 nhóm benchmark reports |3.088|239.670.204|118.907.647|
| Tổng |14.912|535.425.398|308.820.084|

Archive bytes chưa gồm manifest riêng và index; chi tiết footprint có trong biên bản. Store canonical nằm tại `experiments/social_navigation/benchmark/generated/artifact_store` của checkout chính; không dùng một store mới cùng đường dẫn tương đối bên trong worktree nếu muốn đọc các object này.

CLI pack được chạy lần hai: index/object giống byte và không tăng số archive. CLI verify toàn bộ index và restore nhóm `generated_v1` được chạy với PATH rỗng trên bản store di động không có `catalogs/`, xác nhận không dựa vào Git hoặc inventory cache. Bản store di động thử dùng hardlink cùng đĩa, không được tính là backup độc lập.

Đã kiểm lại các bản giải nén thử rồi dọn riêng chúng; giữ 50 archive và manifest canonical. 14.912 tệp nguồn giữ nguyên byte; trạng thái/index checkout chính không đổi. Index 67.526 byte được giữ trong Git, còn archive/manifest nằm local và chưa upload lên remote. Vì nguồn vẫn còn trong Git, bước này tạo backup kiểm chứng và **chưa giảm dung lượng checkout/clone**. Chuyển dữ liệu ra checkout phải đi cùng hydrate consumer và chính sách lưu artifact lâu dài.

## Hydrate vào đường dẫn consumer

`tools/hydrate_history.py` đưa **một nhóm** vào đúng `root` trong checkout. Khác với restore ra thư mục riêng, hydrate bảo toàn đường dẫn mà config/loader/script lịch sử đã dùng.

```bash
python3 tools/hydrate_history.py --source git \
  --group world:school_arena/demo_1/dataset_v1.0 \
  --report experiments/social_navigation/benchmark/generated/hydrate_dataset.json
# Hoặc dùng archive đã có, không cần Git objects của payload:
python3 tools/hydrate_history.py --source archive --store /PATH/TO/STORE \
  --group world:school_arena/demo_1/scenario_generator/generated_v1 \
  --report experiments/social_navigation/benchmark/generated/hydrate_scenarios.json
# Kiểm tra chỉ đọc; exit 1 khi absent, exit 0 khi toàn bộ payload đúng:
python3 tools/hydrate_history.py --source git --check \
  --group world:school_arena/demo_1/dataset_v1.0 \
  --report experiments/social_navigation/benchmark/generated/hydrate_check.json
```

Mặc định checkout và index là repository chứa tool; `--checkout` và `--index` cho phép chọn rõ. Git source đọc revision đầy đủ đã pin, không đọc HEAD/working tree và không tự truy cập mạng. Clone nông thiếu commit cần fetch revision đó trước; clone đầy đủ vẫn có dữ liệu cũ dù payload đã bỏ ở HEAD. Archive source cần object + manifest đúng hash, không cần Git hoặc inventory cache.

Payload được ghi vào staging trên cùng filesystem, kiểm lại từng SHA256/mode rồi publish thư mục bằng thao tác atomic không ghi đè (`renameat2` trên Linux, `rename` trên Windows). Nếu đích xuất hiện giữa chừng, hydrate dừng và giữ dữ liệu đó. Nền tảng không có thao tác publish hỗ trợ sẽ báo lỗi. Thư mục đích có sẵn chỉ được chấp nhận khi toàn bộ inventory đúng, không có file dư; file khác, thiếu, rỗng hoặc symlink bị chặn. Tool không merge, xóa hoặc thay nội dung người dùng.

Ba world payload được ignore riêng sau migration; report/gate không thay đổi. `DYNAMUHSIM_INSTALL_LEGACY_WORLD_PAYLOADS=ON` yêu cầu ba thư mục tồn tại và báo lệnh hydrate nếu thiếu. Sau hydrate vẫn nên chạy `--check` cho từng nhóm trước khi dùng; CMake guard chỉ kiểm thư mục tồn tại, không thay kiểm integrity. Default build không cần hydrate.

Audit **full legacy install** sau hydrate phải có index lịch sử, vì HEAD không còn chứa các blob payload trong cây hiện tại:

```bash
python3 tools/catalog_assets.py verify-install --legacy \
  --history-index docs/artifacts/history_2026-10-25.json \
  --share-dir /PATH/TO/install/custom_corridor/share/custom_corridor \
  --output /PATH/TO/NEW_legacy_audit.json
```

Nếu một payload trong policy không còn tracked và không cung cấp index, audit trả FAIL với `historical_payload_inventory_available=false`. Có index, audit dựng inventory từ revision đã pin và kiểm từng byte của đủ payload đã cài; không chỉ kiểm phần resource còn tracked ở HEAD.

## Kết quả chuyển ba world payload khỏi HEAD (25/10/2026)

[Manifest migration](artifacts/world_payload_migration_2026-10-25.json) · [Biên bản hydrate/consumer/build](verification/hydration_2026-10-25.json).

Đã bỏ **11.824 tệp / 295.755.194 byte** của ba payload school khỏi cây Git hiện tại. Dữ liệu vẫn được phục hồi từ commit đã pin hoặc archive local. Không rewrite lịch sử, nên clone đầy đủ vẫn tải lịch sử cũ; giảm ở đây là số file/byte của checkout HEAD. Cả 47 nhóm reports (3.088 tệp / 239.670.204 byte), bảy gate và source freeze được giữ nguyên byte.

**85 tests PASS.** Hydrate thật bằng CLI được chạy cho cả ba nhóm từ archive và từ Git khi HEAD không còn track payload. Mỗi nguồn khôi phục đúng đường dẫn, hash/mode/coverage; kết quả JSON consumer trước migration, sau hydrate archive và sau hydrate Git giống byte hoàn toàn.

- Production config vẫn resolve đúng `dataset_v1.0`; `read_plan` và `EpisodeLoader` đọc được 605/605 episode inputs, so identity/seed/hash với plan, không có loader failure.
- Consumer `scenario_coverage.py` kiểm 134/134 episode trong `generated` và 72/72 trong `generated_v1`: artifact PASS, không có artifact FAIL hoặc lỗi không phân loại.
- Production audit chỉ đọc vẫn là **INCOMPLETE**. Registry cũ có 12 PASS, 1 FAIL_INFRASTRUCTURE, 591 PLANNED, 1 RUNNING. Lượt parity chạy không source ROS nên còn báo `rosbag2_py unavailable`; đây không phải kiểm chứng bag readability hoặc chứng nhận dataset hoàn chỉnh. Không đổi outcome/nhãn/split.
- Payload vắng mặt: build runtime-copy và runtime-symlink thành công; audit đủ 173 world files. Build legacy báo thiếu payload cùng lệnh hydrate và report theo từng nhóm.
- Sau hydrate: full legacy copy build thành công và audit với history index kiểm đủ 11.997 world files / 12.042 resource files. Bỏ history index khiến audit FAIL đúng, tránh PASS trên tập file còn tracked ở HEAD.

Đã đối chiếu lại rồi dọn **chỉ các payload hydrate và full legacy install tạo để kiểm thử**; giữ archive canonical, log và hai runtime prefix. Checkout nhánh dọn dẹp hiện không có ba payload; hydrate theo nhu cầu sẽ làm chúng xuất hiện dưới dạng ignored data. Checkout/index chính của người dùng không đổi.

Phần còn lại là phân loại consumer của 47 nhóm report trước khi chuyển chúng: gate đang dùng, source snapshot cần replay và media/số liệu lịch sử. Hydrate dữ liệu không tự chứng minh mọi workflow ROS/Docker lịch sử hoạt động.

## Migration report tiếp theo

Đã chuyển 24 nhóm report lịch sử khỏi HEAD và giữ 23 nhóm cùng bảy gate. Xem [Report và dependency](REPORT_STORAGE.md) để xem đúng revision trên GitHub, hydrate kèm dependency và đọc kết quả parity; các số “47 nhóm reports còn giữ” phía trên ghi lại giai đoạn world trước khi chuyển report.
