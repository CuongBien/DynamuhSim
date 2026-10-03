#!/usr/bin/env bash
# =============================================================================
# run_v2_all.sh
# Chạy tuần tự tất cả 13 kịch bản v2, mỗi kịch bản điều hướng đến goal cố định.
#
# Cách dùng:
#   cd ~/nav_ws/experiments/trajectory_risk_v0
#   ./run_v2_all.sh
#
# Override ví dụ:
#   TIMEOUT_SEC=900 PAUSE_BETWEEN=15 ./run_v2_all.sh
#
# Yêu cầu:
#   - Gazebo + corridor_tb3 đã được launch sẵn (arena_dataset world).
#   - Nav2 stack đang chạy.
# =============================================================================
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXP="$WS/experiments/trajectory_risk_v0"
SCENARIOS_DIR="$EXP/scenarios_v1"
DATA_DIR="$EXP/data_v2"

# ── Tham số có thể override qua env ─────────────────────────────────────────
GOAL_X="${GOAL_X:-4.5}"
GOAL_Y="${GOAL_Y:-0.0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-600}"   # giây thực mỗi kịch bản (tăng nếu RTF thấp)
PAUSE_BETWEEN="${PAUSE_BETWEEN:-3}" # giây nghỉ giữa 2 kịch bản (để Nav2 reset)

# ── Danh sách 13 kịch bản theo thứ tự tăng dần độ khó ──────────────────────
SCENARIO_FILES=(
  "scenario_v2_001.json"   # 01 empty_safe
  "scenario_v2_002.json"   # 02 static_people
  "scenario_v2_006.json"   # 03 same_direction   (dễ hơn multi)
  "scenario_v2_003.json"   # 04 single_crossing
  "scenario_v2_005.json"   # 05 head_on
  "scenario_v2_004.json"   # 06 multi_crossing
  "scenario_v2_010.json"   # 07 door_entry
  "scenario_v2_011.json"   # 08 door_exit
  "scenario_v2_009.json"   # 09 wait_then_cross
  "scenario_v2_012.json"   # 10 yielding
  "scenario_v2_007.json"   # 11 mixed_flow
  "scenario_v2_008.json"   # 12 group_blocking
  "scenario_v2_013.json"   # 13 dense_social     (khó nhất)
)

TOTAL=${#SCENARIO_FILES[@]}

# ── Setup môi trường ─────────────────────────────────────────────────────────
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p "$DATA_DIR"

# ── Hàm teleport robot về vị trí start ──────────────────────────────────────
ROBOT_START_X="${ROBOT_START_X:--13.0}"
ROBOT_START_Y="${ROBOT_START_Y:-0.0}"
ROBOT_START_YAW="${ROBOT_START_YAW:-0.0}"
GZ_WORLD="${GZ_WORLD:-arena_dataset}"

reset_robot() {
  echo "  [RESET] Teleport robot về start (x=$ROBOT_START_X, y=$ROBOT_START_Y)..."

  # Dừng robot bằng cmd_vel = 0
  set +e
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" > /dev/null 2>&1 || true
  sleep 0.5

  # Tính quaternion từ yaw
  local qz qw
  qz=$(python3 -c "import math; print(math.sin($ROBOT_START_YAW / 2.0))")
  qw=$(python3 -c "import math; print(math.cos($ROBOT_START_YAW / 2.0))")

  # Teleport qua gz service
  local req="name: \"burger\", position: {x: $ROBOT_START_X, y: $ROBOT_START_Y, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}"
  local result
  result=$(gz service \
    -s "/world/${GZ_WORLD}/set_pose" \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "$req" 2>&1)
  set -e

  if echo "$result" | grep -q "true"; then
    echo "  [RESET] OK — robot đã về vị trí start."
  else
    echo "  [RESET] WARN — set_pose reply: $result"
  fi

    # Chờ odom ổn định sau teleport
  sleep 1.5

  # Reset AMCL → RViz cập nhật đúng vị trí
  echo "  [RESET] Publish /initialpose để reset AMCL..."
  set +e
  ros2 topic pub --once /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{
      header: {frame_id: map},
      pose: {
        pose: {
          position: {x: $ROBOT_START_X, y: $ROBOT_START_Y, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}
        },
        covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                     0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0.07]
      }
    }" > /dev/null 2>&1 || true
  set -e

  # Chờ AMCL hội tụ
  sleep 1.5
}
# ── Hàm chạy 1 kịch bản ─────────────────────────────────────────────────────

# ── Hàm chạy 1 kịch bản ─────────────────────────────────────────────────────
run_episode() {
  local idx="$1"          # 01..13
  local scenario_file="$2"
  local scenario_path="$SCENARIOS_DIR/$scenario_file"

  # Đọc metadata từ JSON
  local family
  family=$(python3 -c \
    "import json,sys; d=json.load(open('$scenario_path')); print(d.get('scenario_family','unknown'))")

  local STAMP
  STAMP=$(date +%Y%m%d_%H%M%S)
  local OUT="$DATA_DIR/ep${idx}_${family}_${STAMP}"
  mkdir -p "$OUT"

  echo ""
  echo "╔══════════════════════════════════════════════════════════════╗"
  printf  "║  Episode %s/%s  %-44s ║\n" "$idx" "$TOTAL" "$family"
  echo "╚══════════════════════════════════════════════════════════════╝"
  echo "  Scenario : $scenario_file"
  echo "  Goal     : x=$GOAL_X  y=$GOAL_Y"
  echo "  Timeout  : ${TIMEOUT_SEC}s"
  echo "  Output   : $OUT"
  echo ""

  # [1] Publish scenario + khởi động người
  echo "[1/5] Publish scenario & start humans"
  cd "$EXP"
  python3 scripts/publish_gz_scenario.py "$scenario_path"

  # [2] Ghi rosbag
  echo "[2/5] Start rosbag2"
  ros2 bag record -o "$OUT/rosbag" \
    /scan /odom /cmd_vel /tf /tf_static /clock \
    > "$OUT/rosbag.log" 2>&1 &
  BAG_PID=$!

  # [3] Ghi CSV frame
  echo "[3/5] Start CSV logger"
  python3 scripts/log_frame_csv.py \
    --output "$OUT/frames.csv" \
    > "$OUT/csv_logger.log" 2>&1 &
  CSV_PID=$!

    # Cleanup nội bộ — có timeout để không bị kẹt chờ rosbag flush
  cleanup_ep() {
    # Gửi SIGINT để rosbag2/logger flush gracefully
    kill -INT "$CSV_PID" 2>/dev/null || true
    kill -INT "$BAG_PID" 2>/dev/null || true

    # Chờ tối đa 15s cho CSV logger
    local deadline=$(( $(date +%s) + 15 ))
    while kill -0 "$CSV_PID" 2>/dev/null && [ "$(date +%s)" -lt "$deadline" ]; do
      sleep 0.5
    done
    kill -9 "$CSV_PID" 2>/dev/null || true

    # Chờ tối đa 20s cho rosbag2 (cần thêm thời gian để flush MCAP)
    deadline=$(( $(date +%s) + 20 ))
    while kill -0 "$BAG_PID" 2>/dev/null && [ "$(date +%s)" -lt "$deadline" ]; do
      sleep 0.5
    done
    kill -9 "$BAG_PID" 2>/dev/null || true

    # Đảm bảo không còn zombie
    wait "$CSV_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  }
  # Không dùng trap ở đây vì vòng lặp; gọi trực tiếp ở cuối

  # Lưu scenario
  cp "$scenario_path" "$OUT/scenario.json"
  sleep 2   # chờ người vào vị trí

  # [4] Gửi NavigateToPose
  echo "[4/5] Navigate to goal (x=$GOAL_X, y=$GOAL_Y) — timeout=${TIMEOUT_SEC}s"
  set +e
  timeout "${TIMEOUT_SEC}s" \
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
    --feedback | tee "$OUT/navigation.log"
  NAV_RC=${PIPESTATUS[0]}
  set -e

  echo "$NAV_RC" > "$OUT/navigation_return_code.txt"

  # Phân loại kết quả
  case "$NAV_RC" in
    0)   STATUS="SUCCESS" ;;
    124) STATUS="TIMEOUT" ;;
    *)   STATUS="FAILED (rc=$NAV_RC)" ;;
  esac
  echo "  → Navigation result: $STATUS"
  echo "$STATUS" > "$OUT/status.txt"

  # [5] Dừng người
  echo "[5/5] Stop humans"
  gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "stop"' || true

  # Dừng logger + bag
  cleanup_ep

  echo ""
  echo "  Episode done → $OUT"
}

# ── Vòng lặp chính ───────────────────────────────────────────────────────────
PASS=0; FAIL=0; TIMEOUT_COUNT=0
SUMMARY=()

for i in "${!SCENARIO_FILES[@]}"; do
  IDX=$(printf "%02d" $((i + 1)))
  FILE="${SCENARIO_FILES[$i]}"

  run_episode "$IDX" "$FILE"

  STATUS_VAL=$(cat "$DATA_DIR"/ep${IDX}_*/status.txt 2>/dev/null | tail -1 || echo "?")
  SUMMARY+=("[$IDX] ${FILE%%.json}: $STATUS_VAL")

    case "$STATUS_VAL" in
    SUCCESS) PASS=$((PASS + 1))              ;;
    TIMEOUT*) TIMEOUT_COUNT=$((TIMEOUT_COUNT + 1)) ;;
    *)        FAIL=$((FAIL + 1))             ;;
  esac

    # Reset robot + nghỉ giữa 2 kịch bản
  if [ $((i + 1)) -lt "$TOTAL" ]; then
    echo ""
    echo "  ── Chuyển sang kịch bản tiếp theo ──"
    reset_robot
    echo "  ── Nghỉ ${PAUSE_BETWEEN}s để Nav2 ổn định ──"
    sleep "$PAUSE_BETWEEN"
  fi
done

# ── Tổng kết ─────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                     KẾT QUẢ TỔNG HỢP                       ║"
echo "╠══════════════════════════════════════════════════════════════╣"
for line in "${SUMMARY[@]}"; do
  printf "║  %-60s ║\n" "$line"
done
echo "╠══════════════════════════════════════════════════════════════╣"
printf "║  SUCCESS: %-3s  TIMEOUT: %-3s  FAILED: %-3s  TOTAL: %-3s       ║\n" \
       "$PASS" "$TIMEOUT_COUNT" "$FAIL" "$TOTAL"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Tất cả dữ liệu: $DATA_DIR"