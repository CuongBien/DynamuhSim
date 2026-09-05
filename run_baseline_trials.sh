#!/usr/bin/env bash

# ------------------------------------------------------
# Environment
# ------------------------------------------------------

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

set -Eeuo pipefail


# ------------------------------------------------------
# Configuration
# ------------------------------------------------------

BASELINE_DIR="${BASELINE_DIR:-$ROOT/experiments/corridor_090/baseline_01}"

TRIAL_COUNT="${1:-1}"
START_TRIAL="${START_TRIAL:-3}"

WIDTH="${WIDTH:-0.90}"

# MAP-frame coordinates, NOT Gazebo/world coordinates.
MAP_INITIAL_X="${MAP_INITIAL_X:--0.022}"
MAP_INITIAL_Y="${MAP_INITIAL_Y:-0.042}"
MAP_INITIAL_YAW="${MAP_INITIAL_YAW:--0.004}"

GOAL_X="${GOAL_X:-11.74}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
GOAL_TOLERANCE="${GOAL_TOLERANCE:-0.15}"

TRIAL_TIMEOUT_S="${TRIAL_TIMEOUT_S:-180}"
READY_TIMEOUT_S="${READY_TIMEOUT_S:-60}"

ANALYZER="${ANALYZER:-$ROOT/analyze_baseline.py}"


# ------------------------------------------------------
# Validation
# ------------------------------------------------------

[[ "$TRIAL_COUNT" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: TRIAL_COUNT must be positive"
    exit 2
}

[[ "$START_TRIAL" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: START_TRIAL must be positive"
    exit 2
}

cleanup_processes() {
    local pid

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM -- "-$pid" 2>/dev/null ||
            kill -TERM "$pid" 2>/dev/null ||
            true
        fi
    done

    sleep 1

    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null ||
            kill -KILL "$pid" 2>/dev/null ||
            true
        fi
    done

    PIDS=()
}


kill_stale_ros_processes() {
    echo "[CLEANUP] Removing stale processes..."

    pkill -TERM -x amcl 2>/dev/null || true
    pkill -TERM -x map_server 2>/dev/null || true
    pkill -TERM -x planner_server 2>/dev/null || true
    pkill -TERM -x controller_server 2>/dev/null || true
    pkill -TERM -x bt_navigator 2>/dev/null || true
    pkill -TERM -x behavior_server 2>/dev/null || true
    pkill -TERM -x waypoint_follower 2>/dev/null || true
    pkill -TERM -x lifecycle_manager 2>/dev/null || true
    pkill -TERM -x parameter_bridge 2>/dev/null || true
    pkill -TERM -x robot_state_publisher 2>/dev/null || true
    pkill -TERM -x rviz2 2>/dev/null || true
    pkill -TERM -f 'ros2 bag record' 2>/dev/null || true
    pkill -TERM -f 'record_obstacle.py' 2>/dev/null || true
    pkill -TERM -f 'gz sim' 2>/dev/null || true

    sleep 2

    pkill -KILL -x amcl 2>/dev/null || true
    pkill -KILL -x map_server 2>/dev/null || true
    pkill -KILL -x planner_server 2>/dev/null || true
    pkill -KILL -x controller_server 2>/dev/null || true
    pkill -KILL -x bt_navigator 2>/dev/null || true
    pkill -KILL -x behavior_server 2>/dev/null || true
    pkill -KILL -x waypoint_follower 2>/dev/null || true
    pkill -KILL -x lifecycle_manager 2>/dev/null || true
    pkill -KILL -x parameter_bridge 2>/dev/null || true
    pkill -KILL -x robot_state_publisher 2>/dev/null || true
    pkill -KILL -x rviz2 2>/dev/null || true
    pkill -KILL -f 'ros2 bag record' 2>/dev/null || true
    pkill -KILL -f 'record_obstacle.py' 2>/dev/null || true
    pkill -KILL -f 'gz sim' 2>/dev/null || true

    sleep 2
}


trap cleanup_processes EXIT INT TERM


# ------------------------------------------------------
# Readiness helpers
# ------------------------------------------------------
wait_for_gazebo_ready() {
    echo "[WAIT] Waiting for Gazebo + robot..."

    local deadline=$((SECONDS + READY_TIMEOUT_S))

    while (( SECONDS < deadline )); do

        local topics
        topics="$(timeout 5s ros2 topic list 2>/dev/null || true)"

        if grep -qx "/clock" <<< "$topics" &&
           grep -qx "/scan" <<< "$topics" &&
           grep -qx "/tf" <<< "$topics"
        then
            echo "[READY] Gazebo topics detected"
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] Gazebo/robot not ready"
    return 1
}


wait_for_nav2_nodes() {
    echo "[WAIT] Waiting for Nav2 nodes..."

    local deadline=$((SECONDS + READY_TIMEOUT_S))

    while (( SECONDS < deadline )); do
        local nodes
        nodes="$(timeout 2s ros2 node list 2>/dev/null || true)"

        if grep -qx "/map_server" <<< "$nodes" \
            && grep -qx "/amcl" <<< "$nodes" \
            && grep -qx "/controller_server" <<< "$nodes" \
            && grep -qx "/bt_navigator" <<< "$nodes"; then
            echo "[READY] Nav2 nodes created"
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] Nav2 nodes did not start"
    return 1
}


wait_for_lifecycle_state() {
    local node="$1"
    local wanted="$2"

    local deadline=$((SECONDS + READY_TIMEOUT_S))

    while (( SECONDS < deadline )); do
        local state=""
        state="$(timeout 2s ros2 lifecycle get "$node" 2>/dev/null || true)"

        if echo "$state" | grep -Eq "^${wanted}[[:space:]]*\[[0-9]+\]"; then
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] $node did not reach $wanted"
    return 1
}


wait_for_action_server() {
    local deadline=$((SECONDS + READY_TIMEOUT_S))

    echo "[WAIT] Waiting for /navigate_to_pose..."

    while (( SECONDS < deadline )); do
        if timeout 2s ros2 action list 2>/dev/null |
            grep -qx '/navigate_to_pose'
        then
            echo "[READY] /navigate_to_pose"
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] /navigate_to_pose unavailable"
    return 1
}


# ------------------------------------------------------
# AMCL
# ------------------------------------------------------


publish_initial_pose() {
    echo "[POSE] Publishing /initialpose in MAP frame..."

    # Quaternion from yaw:
    # qz = sin(yaw / 2)
    # qw = cos(yaw / 2)

    local qz
    local qw

    qz="$(python3 -c \
        "import math; print(math.sin(float('$MAP_INITIAL_YAW') / 2.0))")"

    qw="$(python3 -c \
        "import math; print(math.cos(float('$MAP_INITIAL_YAW') / 2.0))")"

    ros2 topic pub \
        --rate 2 \
        /initialpose \
        geometry_msgs/msg/PoseWithCovarianceStamped \
        "{
            header: {
                frame_id: map
            },
            pose: {
                pose: {
                    position: {
                        x: $MAP_INITIAL_X,
                        y: $MAP_INITIAL_Y,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: $qz,
                        w: $qw
                    }
                },
                covariance: [
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0
                ]
            }
        }" >/dev/null 2>&1 &

    local pub_pid=$!

    sleep 3

    kill "$pub_pid" 2>/dev/null || true
    wait "$pub_pid" 2>/dev/null || true
}


# ------------------------------------------------------
# Ground truth
# ------------------------------------------------------

wait_for_ground_truth() {
    local path="$1"
    local deadline=$((SECONDS + READY_TIMEOUT_S))

    echo "[WAIT] Waiting for obstacle ground truth..."

    while (( SECONDS < deadline )); do
        if [[ -s "$path" ]]; then
            local lines
            lines="$(wc -l < "$path")"

            if (( lines >= 3 )); then
                echo "[READY] Ground truth recording"
                return 0
            fi
        fi

        sleep 1
    done

    echo "[ERROR] Ground truth recorder timeout"
    return 1
}


stop_capture() {
    local pid="$1"
    local signal="$2"

    [[ -z "$pid" ]] && return 0

    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi

    kill -"$signal" "$pid" 2>/dev/null || true

    for ((i=0; i<10; i++)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return 0
        fi

        sleep 1
    done

    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}


write_invalid_result() {
    local result_json="$1"
    local trial_name="$2"
    local status="$3"
    local reason="$4"

    printf \
        '{"trial":"%s","status":"%s","reason":"%s"}\n' \
        "$trial_name" \
        "$status" \
        "$reason" \
        > "$result_json"
}


# ------------------------------------------------------
# Trial
# ------------------------------------------------------

run_trial() {
    local trial_id="$1"

    local trial_name
    printf -v trial_name 'trial_%02d' "$trial_id"

    local trial_dir="$BASELINE_DIR/$trial_name"
    local run_dir="$BASELINE_DIR/.run_logs/$trial_name"

    local gt="$BASELINE_DIR/obstacle_ground_truth_$(printf '%02d' "$trial_id").csv"

    local bag_log="$run_dir/rosbag.log"
    local recorder_log="$run_dir/obstacle_recorder.log"
    local goal_log="$run_dir/goal.log"
    local launch_log="$run_dir/gazebo.log"
    local nav_log="$run_dir/nav2.log"

    local result_json="$run_dir/trial_result.json"
    local analysis_dir="$trial_dir/analysis"

    local status="INVALID"
    local reason=""

    local goal_pid=""
    local bag_pid=""
    local recorder_pid=""

    echo
    echo "=========================================="
    echo "STARTING $trial_name"
    echo "=========================================="

    if [[ -e "$trial_dir" ]]; then
        echo "[ERROR] Trial directory already exists:"
        echo "        $trial_dir"
        return 2
    fi

    mkdir -p "$run_dir"

    rm -f "$gt"

    : > "$launch_log"
    : > "$nav_log"
    : > "$goal_log"

    kill_stale_ros_processes


    # --------------------------------------------------
    # 1. Gazebo
    # --------------------------------------------------

    echo "[1/8] Starting Gazebo..."

    setsid ros2 launch \
        custom_corridor \
        corridor_tb3.launch.py \
        width:="$WIDTH" \
        >"$launch_log" 2>&1 &

    local launch_pid="$!"
    PIDS+=("$launch_pid")

    if ! wait_for_gazebo_ready; then
        echo "[ERROR] Gazebo/robot not ready"
        echo
        echo "========== GAZEBO LOG =========="
        if [[ -f "$launch_log" ]]; then
            tail -100 "$launch_log"
        else
            echo "Gazebo log not found: $launch_log"
        fi
        echo "================================"
        return 1
    fi


    # --------------------------------------------------
    # 2. Nav2
    # --------------------------------------------------

    echo "[2/8] Starting Nav2..."

    setsid ros2 launch \
        custom_corridor \
        nav2_corridor.launch.py \
        >"$nav_log" 2>&1 &

    local nav_pid="$!"
    PIDS+=("$nav_pid")

    if ! wait_for_nav2_nodes; then
        status="INVALID"
        reason="NAV2_NODES_NOT_READY"

        write_invalid_result \
            "$result_json" \
            "$trial_name" \
            "$status" \
            "$reason"

        return 0
    fi


    # --------------------------------------------------
    # 3. Nav2 lifecycle readiness
    # --------------------------------------------------

    echo "[3/8] Waiting for Nav2 lifecycle manager..."

    local nav_nodes=(
        /map_server
        /amcl
        /planner_server
        /controller_server
        /bt_navigator
        /behavior_server
        /waypoint_follower
    )

    local nav_deadline=$((SECONDS + READY_TIMEOUT_S))

    local all_active=false

    while (( SECONDS < nav_deadline )); do
        # Check if both localization and navigation lifecycle managers confirmed active
        if grep -q "lifecycle_manager_localization.*Managed nodes are active" "$nav_log" &&
           grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$nav_log"; then
            echo "[READY] Nav2 stack ACTIVE (confirmed via lifecycle managers)"
            all_active=true
            break
        fi

        local nodes_ok=true
        for node in "${nav_nodes[@]}"; do
            local state=""
            state="$(timeout 3s ros2 lifecycle get "$node" 2>/dev/null || true)"

            if ! grep -Eq '^active[[:space:]]*\[[0-9]+\]' <<< "$state"; then
                nodes_ok=false
                break
            fi
        done

        if [[ "$nodes_ok" == true ]]; then
            echo "[READY] Nav2 stack ACTIVE"
            all_active=true
            break
        fi

        sleep 1
    done

    if [[ "$all_active" != true ]]; then
        echo "[ERROR] Nav2 lifecycle did not become ACTIVE"
        echo
        echo "========== NAV2 LOG =========="
        tail -150 "$nav_log" 2>/dev/null || true
        echo "=============================="

        status="INVALID"
        reason="NAV2_LIFECYCLE_FAILED"

        write_invalid_result \
            "$result_json" \
            "$trial_name" \
            "$status" \
            "$reason"

        return 0
    fi

    echo "[READY] Nav2 stack ACTIVE"

    if ! wait_for_action_server; then
        status="INVALID"
        reason="ACTION_SERVER_NOT_AVAILABLE"

        echo "[DIAG] Actions:"
        timeout 3s ros2 action list 2>&1 || true

        write_invalid_result \
            "$result_json" \
            "$trial_name" \
            "$status" \
            "$reason"

        return 0
    fi

    # --------------------------------------------------
    # 4. AMCL initial pose
    # --------------------------------------------------

    echo "[4/8] Publishing AMCL initial pose..."

    publish_initial_pose

    echo "[WAIT] Waiting for AMCL pose..."

    local amcl_ready=false
    local amcl_deadline=$((SECONDS + READY_TIMEOUT_S))

    while (( SECONDS < amcl_deadline )); do
        # 1. Check if AMCL has processed initial pose and set it
        if grep -q "Setting pose" "$nav_log"; then
            echo "[READY] AMCL pose confirmed (Setting pose active in nav2 log)"
            amcl_ready=true
            break
        fi

        # 2. Check if /amcl_pose message can be received
        if timeout 2s ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
            echo "[READY] AMCL pose available via /amcl_pose"
            amcl_ready=true
            break
        fi

        # 3. Check if map -> odom TF is published by AMCL
        if timeout 2s ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1; then
            echo "[READY] AMCL transform map -> odom available"
            amcl_ready=true
            break
        fi

        sleep 1
    done

    if [[ "$amcl_ready" != true ]]; then
        echo "[ERROR] AMCL did not publish pose"

        status="INVALID"
        reason="AMCL_POSE_NOT_READY"

        write_invalid_result \
            "$result_json" \
            "$trial_name" \
            "$status" \
            "$reason"

        return 0
    fi


    # --------------------------------------------------
    # 5. Ground truth
    # --------------------------------------------------

    echo "[5/8] Starting obstacle recorder..."

    python3 \
        "$ROOT/record_obstacle.py" \
        --output "$gt" \
        >"$recorder_log" 2>&1 &

    recorder_pid="$!"
    PIDS+=("$recorder_pid")

    if ! wait_for_ground_truth "$gt"; then
        status="INVALID"
        reason="GROUND_TRUTH_NOT_RECORDING"

        write_invalid_result \
            "$result_json" \
            "$trial_name" \
            "$status" \
            "$reason"

        return 0
    fi


    # --------------------------------------------------
    # 6. Rosbag
    # --------------------------------------------------

    echo "[6/8] Starting rosbag..."

    ros2 bag record \
        -o "$trial_dir" \
        /clock \
        /cmd_vel \
        /odom \
        /scan \
        /tf \
        /tf_static \
        /map \
        /plan \
        /local_plan \
        >"$bag_log" 2>&1 &

    bag_pid="$!"
    PIDS+=("$bag_pid")

    sleep 2


    # --------------------------------------------------
    # 7. Navigation goal
    # --------------------------------------------------

    echo "[7/8] Sending navigation goal..."

    local goal_yaml

    goal_yaml="{
        pose: {
            header: {
                frame_id: map
            },
            pose: {
                position: {
                    x: $GOAL_X,
                    y: $GOAL_Y,
                    z: 0.0
                },
                orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0
                }
            }
        }
    }"

    timeout \
        "${TRIAL_TIMEOUT_S}s" \
        ros2 action send_goal \
        /navigate_to_pose \
        nav2_msgs/action/NavigateToPose \
        "$goal_yaml" \
        --feedback \
        >"$goal_log" 2>&1 &

    goal_pid="$!"
    PIDS+=("$goal_pid")

    if wait "$goal_pid"; then

        if grep -q \
            'Goal finished with status: SUCCEEDED' \
            "$goal_log"
        then
            status="SUCCESS"

        elif grep -qE \
            'Goal finished with status: (ABORTED|CANCELED)' \
            "$goal_log"
        then
            status="FAILURE"

        elif grep -q \
            'Goal was rejected' \
            "$goal_log"
        then
            status="INVALID"
            reason="GOAL_REJECTED"

        else
            status="INVALID"
            reason="UNRECOGNIZED_ACTION_RESULT"
        fi

    else

        if grep -qi \
            'timed out' \
            "$goal_log" ||
            ! grep -q \
            'Goal finished with status:' \
            "$goal_log"
        then
            status="TIMEOUT"
            reason="GOAL_TIMEOUT"
        else
            status="INVALID"
            reason="GOAL_PROCESS_ERROR"
        fi
    fi


    # --------------------------------------------------
    # 8. Stop recording
    # --------------------------------------------------

    echo "[8/8] Stopping recording..."

    stop_capture "$bag_pid" TERM
    bag_pid=""

    stop_capture "$recorder_pid" TERM
    recorder_pid=""

    if [[ -d "$trial_dir" && ! -f "$trial_dir/metadata.yaml" ]]; then
        echo "[REINDEX] Reindexing rosbag metadata..."
        ros2 bag reindex -s mcap "$trial_dir" >/dev/null 2>&1 || true
    fi


    # --------------------------------------------------
    # Analyze
    # --------------------------------------------------

    echo "[ANALYZE] Running analyzer..."

    local analyzer_status=0

    if [[ -s "$gt" && -d "$trial_dir" ]]; then

        mkdir -p "$analysis_dir"

        python3 \
            "$ANALYZER" \
            --bag "$trial_dir" \
            --ground-truth "$gt" \
            --output "$analysis_dir" \
            --goal-x "$GOAL_X" \
            --goal-y "$GOAL_Y" \
            --goal-tolerance "$GOAL_TOLERANCE" \
            || analyzer_status=$?

    else
        analyzer_status=2
    fi


    if [[ "$analyzer_status" -ne 0 ]]; then
        status="INVALID"
        reason="ANALYZER_FAILED_$analyzer_status"
    fi


    # --------------------------------------------------
    # Result
    # --------------------------------------------------

    printf \
        '{
            "trial":"%s",
            "status":"%s",
            "reason":"%s",
            "goal":{
                "x":%s,
                "y":%s,
                "tolerance":%s
            },
            "map_initial_pose":{
                "x":%s,
                "y":%s,
                "yaw":%s
            },
            "bag":"%s",
            "ground_truth":"%s",
            "analysis":"%s",
            "logs":"%s"
        }\n' \
        "$trial_name" \
        "$status" \
        "$reason" \
        "$GOAL_X" \
        "$GOAL_Y" \
        "$GOAL_TOLERANCE" \
        "$MAP_INITIAL_X" \
        "$MAP_INITIAL_Y" \
        "$MAP_INITIAL_YAW" \
        "$trial_dir" \
        "$gt" \
        "$analysis_dir" \
        "$run_dir" \
        > "$result_json"

    cp "$result_json" "$trial_dir/trial_result.json"

    echo
    echo "------------------------------------------"
    echo "$trial_name: $status${reason:+ ($reason)}"
    echo "------------------------------------------"
}


# ======================================================
# BATCH
# ======================================================

echo
echo "=========================================="
echo "AUTOMATED TRIAL RUNNER"
echo "=========================================="
echo "Baseline       : $BASELINE_DIR"
echo "Width          : $WIDTH"
echo "Trials         : $TRIAL_COUNT"
echo "Start          : $START_TRIAL"
echo "Map initial    : ($MAP_INITIAL_X, $MAP_INITIAL_Y, $MAP_INITIAL_YAW)"
echo "Goal           : ($GOAL_X, $GOAL_Y)"
echo "Timeout        : ${TRIAL_TIMEOUT_S}s"
echo "=========================================="
echo


for ((offset=0; offset<TRIAL_COUNT; offset++)); do

    trial_id=$((START_TRIAL + offset))

    run_trial "$trial_id"

    echo
    echo "[BATCH] Cleaning up after trial $trial_id..."

    cleanup_processes
    kill_stale_ros_processes

    echo "[BATCH] Trial $trial_id finished."
    echo

done


echo
echo "=========================================="
echo "BATCH COMPLETE"
echo "=========================================="
echo "Completed $TRIAL_COUNT trial(s)"
echo "Directory: $BASELINE_DIR"
echo "=========================================="