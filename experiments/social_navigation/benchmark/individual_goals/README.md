# CrowdSimPlus: đích riêng cho từng người

Runner ma trận và analyzer của cohort này đã nghỉ dùng. Báo cáo giữ nguyên;
[source và lệnh cũ](https://github.com/CuongBien/DynamuhSim/tree/9833565fa546d55cb0e553f8388bf4a703c59583/experiments/social_navigation/benchmark)
được ghim theo revision lịch sử. Các lệnh `trial`/kiểm định còn dưới đây vẫn được hỗ trợ.

Đây là fixture mới để kiểm tra tuần tra SFM/ORCA+, không chỉnh policy native.
Giữ initial poses seed 42/43, map, Mecanum/Nav2, timestep 0,1 s, human radius
0,36 m và preferred speed 0,8 m/s. Ba người có hai endpoint x=-28/-21, với
y=-1,4/0/+1,4 theo thứ tự y ban đầu. Các endpoint được kiểm tra không lấn
factory. Không có lane-following constraint, cohesion hoặc group membership.

`--goal-layout shared` vẫn là mặc định, giữ hash fixture cũ. Fixture này dùng CrowdSimPlus counterflow và đặt suffix `_individual_goals`
cho kết quả. Có thêm [fixture crossing riêng](../crossing/README.md) dùng
cùng route selection, với initial poses/endpoint khác.
Người đổi endpoint sau đúng native `Human.reached_destination`, không đổi
arrival threshold, forces hoặc sửa pose để bảo đảm đến đích.

## Chạy một lượt để quan sát

Từ repository root, trong terminal Ubuntu; đóng phiên trước rồi chạy:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy sfm --goal-layout individual \
  --condition counterflow --seed 42 --gui --diagnostics \
  --report-dir "experiments/social_navigation/benchmark/generated/individual_demo_$(date +%Y%m%d_%H%M%S)"
```

Robot tự nhận goal (-14,0). GUI tự đóng khi lượt benchmark kết thúc.
Đổi `--human-policy orca_plus` để chạy đối chiếu; dùng report directory mới.

## Protocol cohort lịch sử

Runner lịch sử dùng image đã ghim và tự kiểm định trước khi chạy bốn lượt Gazebo tuần
tự. Mỗi policy/seed phải khớp full native env.step qua 30 bước, giữ nguyên
shared-mode dynamics so với kernel trước thay đổi, và hoàn tất >=2 chặng mỗi
người trong 30 sim s với robot cố định ở xa. SFM seed 42 có thêm shared-endpoint
control với cùng robot input; đối chứng được ghi nhận, không ép phải thất bại.
Các probe worker không được tính thành lượt Nav2. Dùng `--validate-only` để
chỉ chạy bước này; validation cache chỉ dùng lại khi kernel hash không đổi.

`reports/run01/kernel_before.py` là snapshot adapter trước khi thêm per-human
routes, được giữ để kiểm tra regression, không phải policy upstream mới.
Source CrowdSimPlus trong Docker và plugin C++ không đổi.

[Report](reports/run01/INDIVIDUAL_GOALS_REPORT.md). Đây vẫn là ba người độc lập
trên lối chính bên trái nhà xưởng, chưa phải group walking, conversation hoặc
benchmark bottleneck/density. Goal success cần đọc cùng số chặng của từng
người và clearance. Không gộp thống kê với fixture shared-endpoint trước.
