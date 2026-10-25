# Archive và khôi phục payload lịch sử

`tools/archive_history.py` đóng gói **toàn bộ nhóm trong catalog đã pin**, đọc committed Git blobs thay vì working tree. Công cụ không xóa, di chuyển hoặc ghi đè dataset/report nguồn. Capture ON/OFF/MCAP riêng vẫn dùng `tools/artifact_store.py`.

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
