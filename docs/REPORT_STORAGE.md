# Report lịch sử và dependency hydrate

**Layout ngày 28/10/2026:** 23 cây control/report còn giữ đã chuyển nguyên byte sang `experiments/archive/social_navigation/reports/<stage>/`. Đường dẫn benchmark cũ là alias; các manifest lịch sử giữ nguyên. 13 thư mục cohort chỉ còn vỏ được gộp tại [danh mục nghỉ dùng](history/retired_benchmarks.md).


Đã phân loại đủ **47 nhóm / 3.088 tệp / 239.670.204 byte** report từ revision trước migration. **24 nhóm / 1.060 tệp / 169.294.406 byte** được chuyển khỏi HEAD; **23 nhóm / 2.028 tệp / 70.375.798 byte** được giữ nguyên byte, gồm bảy gate và report capture/perception/WM cần kiểm dependency động.

[Dependency plan](artifacts/report_dependency_plan_2026-10-25.json) · [Manifest migration](artifacts/report_migration_2026-10-25.json) · [Biên bản](verification/report_migration_2026-10-25.json).


**Cập nhật media:** 23 nhóm trên hiện giữ 1.814 control/model/snapshot files; 214 media đã chuyển riêng và hydrate theo nhu cầu. Các số liệu 2.028 tệp phía trên ghi lại thời điểm trước đợt media. Xem [Media và trạng thái test](REPORT_MEDIA.md) trước khi chạy viewer/audit cần ảnh.

## Xem và phục hồi report

Mỗi liên kết nhóm trong bảng dưới trỏ vào revision còn đủ report, nên có thể xem ảnh/số liệu trên GitHub mà không hydrate checkout. Để chạy consumer cũ ở đúng đường dẫn:

```bash
python3 tools/hydrate_reports.py --group reports:localization_validation \
  --dry-run --report /tmp/report-plan.json
python3 tools/hydrate_reports.py --group reports:localization_validation \
  --source git --report /tmp/report-hydrate.json
# Hoặc khôi phục toàn bộ 24 nhóm từ archive local:
python3 tools/hydrate_reports.py --all-externalized --source archive \
  --store /PATH/TO/artifact_store --report /tmp/all-report-hydrate.json
```

`localization_validation` tự lấy kèm `shutdown_fix`, là dependency đã nhận diện của validator cũ. Có thể lặp `--group` nhiều lần. `--checkout`, `--index`, `--plan` cho phép chọn rõ workspace và metadata. Mỗi report output phải mới. Dry-run không tạo payload. Các nhóm đã có và khớp toàn bộ inventory được giữ nguyên; dữ liệu khác/thiếu/dư không bị ghi đè. Nếu đã tạo report mới trong cùng root, dùng checkout replay riêng để hydrate bản lịch sử.

Git source đọc đúng commit đã pin, cần lịch sử đầy đủ và không tự truy cập mạng. Archive source cần object + manifest trong store local. Hydrate phục hồi các report thành ignored files. Nó không tự phục hồi raw runtime/MCAP, cài dependency ROS/Docker hoặc chạy mô phỏng.

## 24 nhóm đã externalize

| Nhóm | Tệp | MB payload | Dependency report đã biết |
|---|---:|---:|---|
| [avoidance_comparison](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/avoidance_comparison/reports) | 38 | 4.69 | — |
| [bottleneck](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/bottleneck/reports) | 43 | 2.64 | — |
| [complexitynav_contract](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/complexitynav_contract/reports) | 27 | 0.55 | — |
| [complexitynav_gazebo](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/complexitynav_gazebo/reports) | 58 | 2.25 | complexitynav_geometry |
| [complexitynav_geometry](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/complexitynav_geometry/reports) | 32 | 2.42 | — |
| [crossing](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/crossing/reports) | 36 | 3.62 | — |
| [crowdsim_comparison](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/crowdsim_comparison/reports) | 41 | 5.87 | — |
| [diagnostic](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/diagnostic/reports) | 14 | 1.65 | — |
| [geometry_comparison](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/geometry_comparison/reports) | 46 | 5.56 | — |
| [gpu_runtime](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/gpu_runtime/reports) | 12 | 0.13 | — |
| [group_gazebo_demo](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/group_gazebo_demo/reports) | 31 | 2.60 | — |
| [human_response_replay](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/human_response_replay/reports) | 138 | 12.71 | — |
| [individual_goals](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/individual_goals/reports) | 33 | 2.98 | — |
| [localization_diagnosis](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/localization_diagnosis/reports) | 18 | 2.93 | bottleneck |
| [localization_trials](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/localization_trials/reports) | 58 | 19.23 | shutdown_fix |
| [localization_validation](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/localization_validation/reports) | 112 | 40.64 | shutdown_fix |
| [navigation_diagnostics](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/navigation_diagnostics/reports) | 21 | 5.63 | bottleneck, localization_diagnosis |
| [root](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/reports) | 48 | 5.43 | bottleneck, crossing, individual_goals, sfm_comparison |
| [sfm_comparison](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/sfm_comparison/reports) | 29 | 3.45 | — |
| [shutdown_fix](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/shutdown_fix/reports) | 24 | 0.41 | — |
| [socnavgym_lifecycle](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/socnavgym_lifecycle/reports) | 31 | 9.13 | — |
| [socnavgym_lifecycle_fix](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/reports) | 45 | 11.16 | socnavgym_lifecycle |
| [socnavgym_warehouse_lifecycle](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports) | 61 | 5.95 | — |
| [stability](https://github.com/CuongBien/DynamuhSim/tree/ba9e0debea3ec1993e96a50a0cadeffd52a86353/experiments/social_navigation/benchmark/stability/reports) | 64 | 17.68 | — |

Các nhóm này có **313 media / 55.969.119 byte**, **17 snapshot / 84.775 byte** và **730 tệp data/metadata / 113.240.512 byte**. Chuyển cả nhóm giữ được cấu trúc đường dẫn và cho phép kiểm inventory toàn bộ, không trộn một phần report với gate đã sửa.

## 23 nhóm giữ lại

| Nhóm | Lý do giữ |
|---|---|
| camera_height_ab | capture/perception/WM replay retained pending dynamic dependency verification |
| common_gazebo | dependency of a retained report group |
| common_response | current runtime gate/protocol; source_sha256 references into this report tree |
| dataset_pilot | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_identity_guard | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_perception_corpus | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_person_classifier | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_proxy_ablation | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_proxy_lifecycle | capture/perception/WM replay retained pending dynamic dependency verification; static reference from shared tooling or preserved capture pipeline |
| depth_seed43_validation | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_surface_guard | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_track_refinement | capture/perception/WM replay retained pending dynamic dependency verification |
| depth_visibility_pilot | capture/perception/WM replay retained pending dynamic dependency verification |
| early_avoidance_pilot | source_sha256 references into this report tree; capture/perception/WM replay retained pending dynamic dependency verification; static reference from shared tooling or preserved capture pipeline |
| lidar_person_pilot | capture/perception/WM replay retained pending dynamic dependency verification |
| perception_target_pilot | capture/perception/WM replay retained pending dynamic dependency verification |
| portable_episode | capture/perception/WM replay retained pending dynamic dependency verification |
| shared_nav2_pilot | current runtime gate/protocol |
| socnavgym_group_nav2 | current runtime gate/protocol; static reference from shared tooling or preserved capture pipeline |
| socnavgym_nav2_pilot | current runtime gate/protocol; static reference from shared tooling or preserved capture pipeline |
| socnavgym_response_fix | dependency of a retained report group |
| wm_dataset_replay | capture/perception/WM replay retained pending dynamic dependency verification |
| wm_episode_batch | source_sha256 references into this report tree; capture/perception/WM replay retained pending dynamic dependency verification; static reference from shared tooling or preserved capture pipeline |

## Kiểm chứng và giới hạn

**95 tests PASS.** Trước khi chuyển, archive và từng file nguồn đều được đối chiếu hash. Sau đó cả 24 nhóm được hydrate thật từ archive và Git; probe trước migration, sau archive và sau Git cho JSON kết quả giống byte.

Probe đọc 574 JSON, giải mã 312 PNG/GIF tới frame cuối, giải nén 18 gzip, đọc 47 JSONL, parse bốn XML và compile 17 snapshot Python để kiểm cú pháp. Các tệp còn lại được kiểm hash; một MP4 được kiểm hash, không giải mã vì không có ffmpeg. Không thực thi/import snapshot.

Đã tính lại reported/actual/false success từ 12 trial AMCL và đối chiếu bốn dòng aggregate; không đổi outcome. Consumer gốc `socnavgym_warehouse_lifecycle/check_preflight.py` PASS trước/sau. Khi 24 nhóm vắng mặt, ba profile runtime vẫn qua gate, CLI Nav2/capture/audit vẫn mở được và không có group đã chuyển trong danh sách source runtime/capture hiện tại.

Planner đọc **3.214 source/JSON committed**, phân tích biểu thức Path tĩnh bằng AST và source_sha256 trong JSON; không chạy source. Đây chưa phải graph đầy đủ của import/shell/config động. Vì vậy các nhóm capture/perception/WM còn được giữ theo policy, không kết luận chúng không dùng chỉ vì thiếu reference tĩnh. Recreate plan khi các report đã vắng mặt:

```bash
python3 tools/report_dependencies.py \
  --ref ba9e0debea3ec1993e96a50a0cadeffd52a86353 \
  --output /tmp/rebuilt-report-plan.json
```

Giữ nguyên index/catalog lịch sử làm nguồn hydrate. Sau migration, tạo plan với HEAD sẽ từ chối vì cây report không còn đủ bản gốc; dùng revision trên. Không dùng plan để tự động xóa dữ liệu: thao tác migration đã được kiểm archive, source và consumer riêng.

Đã dọn riêng các bản hydrate dùng kiểm thử; giữ archive canonical và log. Checkout/index chính, dataset01, split/nhãn/freeze và 23 nhóm report giữ lại không thay đổi. Không chạy Gazebo/Docker/Nav2 mới và không rewrite lịch sử Git. Tổng hai đợt world + report chuyển **465.049.600 byte / 12.884 tệp payload** khỏi HEAD; dung lượng lịch sử clone vẫn giữ nguyên.

23 nhóm control/report được giữ trong archive có chủ đích; việc chuyển thêm payload cần kiểm dependency động/raw input theo từng consumer. Đây là điều kiện cho một đợt externalize khác, không phải danh sách mã còn nợ cleanup. Trạng thái dọn mã hiện hành ở [phạm vi đã chốt](CLEANUP_SCOPE.md).
