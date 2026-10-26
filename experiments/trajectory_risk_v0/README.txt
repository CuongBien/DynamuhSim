PBL6 HUMAN SCENARIOS V1
=======================

001 empty_safe
002 static_people
003 single_crossing
004 multi_crossing
005 head_on
006 same_direction
007 mixed_flow
008 group_blocking
009 wait_then_cross
010 door_entry
011 door_exit
012 yielding
013 dense_social

Each scenario contains exactly 24 active humans.
Interaction humans use only stationary / autonomous / scripted modes supported by MultiHumanScenarioSystem.
Background humans stay around |y| >= 3.0.
door_entry and door_exit use a virtual doorway zone because arena_dataset.sdf has no physical room/door geometry.

Install:
  mkdir -p ~/nav_ws/experiments/trajectory_risk_v0/scenarios
  cp scenarios/scenario_*.json ~/nav_ws/experiments/trajectory_risk_v0/scenarios/

Run one:
  ./run_scenario.sh 1

Run all:
  ./run_all_scenarios.sh

Collector hiện tại
-----------------
Các launcher dùng scripts/collect_episode_v2.py; batch tự tạo kịch bản dùng
scripts/collect_dataset_scenarios.py. Dùng run_all_scenarios.sh ở thư mục này
để chạy đủ 13 tệp scenarios_v1/scenario_001.json đến scenario_013.json.
Collector đời đầu collect_episode.py / collect_candidate.py đã được loại bỏ;
có thể xem lại trong lịch sử Git. Dữ liệu và scenario cũ được giữ nguyên.
