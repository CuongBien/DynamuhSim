# Review tích hợp nhánh cleanup — 27/10/2026

Review `codex/shared-navigation-runtime` tại `f856a36` và bản sửa artifact đi kèm,
so với `test/gazebo-harmonic-benchmark` tại `9833565`.

## Kết quả và bản sửa

Phát hiện một lỗi phục hồi: inventory bỏ thư mục, nên bundle có symlink nội bộ
trỏ đến thư mục rỗng đóng gói thành công nhưng restore thất bại. Đã tái hiện trước
khi sửa. Không mất raw vì eviction yêu cầu restore thành công trước.

Inventory mới lưu directory entries; archive kiểm tra loại/quyền, restore tạo
thư mục trước symlink và áp quyền sau khi ghi con. Đường eviction đọc được cả
bundle cũ không có directory entries. Hai regression mới kiểm tra empty/nested
directories, symlink, quyền và pack/restore/evict của định dạng cũ.

Không phát hiện thêm lỗi chặn trong các luồng đã rà: scoped process cleanup,
Nav2 profiles/runtime/recording/capture, shared velocity runtime, analyzers,
source resolution, hydration/media publishing và artifact pack/restore/evict.
Đối chiếu caller thực tế không thấy consumer dùng sai API ProcessScope.
Đây là review theo rủi ro, không phải chứng minh mọi nhánh chạy đều tương đương.

## Kiểm chứng

- 280 tests hiện hành PASS: root 181, scenario generator 13, dataset runner 75,
  dataset production 11. Mười test artifact nằm trong root, không cộng hai lần.
- Regression mới thất bại trước sửa và PASS sau sửa; bundle cũ vẫn phục hồi/evict.
- 1.814 protected files giữ nguyên SHA256; các file tracked khác ngoài allowlist
  của đợt sửa và trạng thái/index/patch checkout chính được đối chiếu riêng.
- Build symlink/copy, asset audits, 33 CLI, worker/native parity và các replay /
  Gazebo probe của đợt cleanup trước được tham chiếu trong acceptance log.
  Không chạy lại simulator/build trong review này vì bản sửa chỉ thuộc artifact.
- Không train, tái tạo dataset, đổi controller/tolerance hay sửa receipt cũ.

Các giới hạn cũ vẫn còn: group standing42 từng báo action SUCCEEDED nhưng không
đạt goal theo world pose; capture standing45 có integrity PASS nhưng navigation
TIMEOUT. Các freeze lịch sử có gap được giữ nguyên, không được đổi thành PASS.
Chi tiết nằm trong [acceptance log](CLEANUP_ACCEPTANCE_2026-10-26.md).

## Tích hợp

Remote benchmark vẫn ở `9833565`, là tổ tiên của cleanup. `merge-tree --write-tree`
không xung đột; tree kết quả bằng tree cleanup. Đây là kiểm tra commit, không áp
thay đổi vào checkout benchmark đang dirty.

Có 958 tracked paths cùng thay đổi ở checkout chính và nhánh cleanup: 942 có cùng
nội dung/quyền thực thi hoặc cùng đã xoá, 16 khác nội dung. Ngoài ra còn 1.707
tracked paths thay đổi riêng ở checkout chính. Không reset/stash/merge checkout
đó trong review này. Bước tích hợp kế tiếp cần bảo toàn thay đổi local, đối chiếu
16 paths khác nhau và những thay đổi riêng trước khi cập nhật checkout chính.

Evidence cục bộ (ignored): `benchmark/generated/final_branch_review_20261027/`,
gồm test logs, primary snapshots, overlap inventory và preservation checks.
