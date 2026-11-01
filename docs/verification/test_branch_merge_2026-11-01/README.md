# Merge detector vào nhánh test — 01/11/2026

Nhánh đích **test/gazebo-harmonic-benchmark**, trước merge dd7a2b0; nguồn **feat/depth-detector-generalization**, commit47fce50f. Merge --no-ff giữ đầy đủ lịch sử hai nhánh, không conflict. Trước bổ sung README/receipt này, tree được stage trùng hoàn toàn source47fce50f.

Checkout mới của nhánh test chạy344tests OK (2skip,42,691s),116protected filesPASS. Model/config default và toàn bộ file world byte-identical với nhánh test trước merge. Weights mới vẫn experimental, không promote model từ việc merge code. Không chạy Gazebo mới hoặc train; dòng StartingGazebo/timeout tronglog là mock failure-path tests.

[Acceptance](acceptance.json) · [Full suite](full_tests.log) · [Baseline](baseline.json) · [Review detector và910frame đã đo](../../INCOMING_DETECTOR_REVIEW_2026-10-31.md).

Workspace chính vẫn feat/mujoco-backend tại b777e64; cuong.txt và tools/mcp_server/ không bị sửa. Nhánh detector giữ47fce50f, không viết lại history.
