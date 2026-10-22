# Bộ so sánh đầu vào chung — offline response pilot

[Báo cáo và đề xuất mô hình](reports/run01/COMMON_RESPONSE_REPORT.md):36/36 phép thử thực thi xong,18 cặp far/near,6 nhánh của3 framework;20-step repeat prefix đạt cho cả6. Đây là **offline simulator data, không phải benchmark chung Gazebo/Nav2**. Các pilot Gazebo trước vẫn giữ riêng.

3 scene: người đi về phía robot, giao cắt, và lối hẹp có robot đứng chắn. Mỗi scene có far control và near robot,6 người,10s/điều kiện, cùng initial discs/targets/42 AABBs và scripted robot trace. Human radius0.36m, nominal robot radius0.25m. Đầu vào0.05s; native planning dt0.1/0.25s được giữ, đối chiếu endpoint chung0.5s. Native speed/observation/prediction assumptions khác nhau, nên không xếp hạng raw distance/timing hoặc gọi đây là equal-algorithm ablation.

Nhánh: SocNavGym native individual/guarded individual, CrowdSimPlus ORCA+/SFM native motion functions, ComplexityNav native mixed/geometry mixed. Các scene so avoidance tắt formation SocNavGym; khả năng group/conversation dùng bằng chứng lifecycle/Gazebo cũ ở báo cáo riêng. Native CN không dùng nội vật cản, là negative control. Native Soc suy ra robot preferred velocity từ goal nội bộ; guard mới dùng measured input velocity. [Audit API](reports/run01/observation_api_audit.json) ghi rõ giới hạn, không giấu thay đổi dưới nhãn native.

Chạy lại từ repository root, tuần tự vào output MỚI:

```bash
python3 experiments/social_navigation/benchmark/common_response/preflight.py
python3 experiments/social_navigation/benchmark/common_response/run.py --report-name my_run
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/common_response/analyze.py \
  --report-dir experiments/social_navigation/benchmark/common_response/reports/my_run
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/benchmark/common_response/plot.py \
  --report-dir experiments/social_navigation/benchmark/common_response/reports/my_run
```

Inputs đã khóa trong inputs/ và có SHA256; run.py tái dùng byte-for-byte, không chạy prepare.py để thay scene. prepare.py chỉ là source tạo bộ inputs ban đầu, sẽ từ chối ghi đè. preflight dựng lại dependency ignored nếu thiếu, kiểm exact3 bookkeeping edits để CrowdSimPlus chứa6 người; native advance/humans/metadata AST giữ nguyên. Không thay native force, collision constrain hoặc robot source.

Runtime hiện có: SocNavGym Python3.12 venv; CrowdSimPlus/ComplexityNav Python3.8.13 container ghim content ID. Mỗi lần một model, tối đa2 CPU/4GB cho container,2-core affinity/one BLAS thread cho worker. Container không mạng, chỉ mount thư mục cohort và geometry code riêng; không mount ROS workspace. Không cài system package, CUDA hay đổi ROS Python. Đợt này không có rendering3D để chuyển sang GPU; GPU NVIDIA đã đo ở các pilot Gazebo riêng. Cohort đo khoảng410s wall, runtime bao gồm observation/harness khác nhau nên không dùng xếp hạng tốc độ framework.

analyze.py dùng GEOS segment-to-box và closest relative approach, giữ tất cả overlap; threshold báo cáo1mm, signed minima vẫn lưu. Flagged tick seconds là thời gian các interval có dấu âm theo ngưỡng, không phải duration tiếp xúc chính xác. Prefix20 bước chỉ chứng minh repeat ngắn (2s hoặc5s theo dt), không chứng nhận mọi episode deterministic.

Mỗi model có source/config metadata, kết quả từng case, trajectory đầy đủ và prefix. Console logs có bản gzip byte-identical. Source hashes, manifest, input geometry checks và preservation checkpoint giữ bằng chứng; máy khác cần checkpoint audit tương đương thay vì viết lại lịch sử. Original robot/Nav2 và các report trước không sửa. Runtime errors hoặc process timeout sẽ được giữ, không gọi prefix an toàn là completed run.

Khuyến nghị theo mục tiêu đã đo: ưu tiên SocNavGym adapter đã nghiệm thu cho nhóm/formation/đứng tương tác; CrowdSimPlus ORCA+ làm đối chứng avoidance độc lập; SFM có soft overlap cần công bố; ComplexityNav geometry là biến thể phụ, chưa có trained RGL hoặc social formation. Bước tiếp theo: lưu cohort, chốt scene/config chung trước một cohort Gazebo riêng; không dùng6/6 trong một seed ngắn làm chứng nhận an toàn hoặc tính tự nhiên.
