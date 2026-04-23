#!/bin/bash
# record_demo.sh — Record expert demonstration data for imitation learning on the Polaris e4.
#
# Usage:
#   ./record_demo.sh <session_label>
#
# Example:
#   ./record_demo.sh route_a
#   ./record_demo.sh parking_lot_test
#
# Output: ~/CS588_STUDENTS/group_12/data/<session_label>/YYYYMMDD_HHMMSS/

set -e

# ── Config ────────────────────────────────────────────────────────────────────
if [ -z "$1" ]; then
    echo "ERROR: session label required."
    echo "Usage: $0 <session_label>   (e.g. route_a, parking_lot_test)"
    exit 1
fi

SESSION_LABEL="$1"
DATA_ROOT="$HOME/CS588_STUDENTS/group_12/data"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_PATH="${DATA_ROOT}/${SESSION_LABEL}/${TIMESTAMP}"

# Split into new bag file every 2 GB (0 = no split)
MAX_BAG_SIZE=2000000000  # bytes

# ── Topic lists ───────────────────────────────────────────────────────────────

# --- Essential: visual input + calibration ---
OBSERVATION_TOPICS=(
    /oak/rgb/image_raw
    /oak/rgb/camera_info
    /oak/imu/data
)

# --- Essential: steering labels + vehicle state ---
ACTION_TOPICS=(
    /joy
    /pacmod/steering_cmd
    /pacmod/steering_rpt
    /pacmod/accel_cmd
    /pacmod/accel_rpt
    /pacmod/brake_cmd
    /pacmod/brake_rpt
    /pacmod/vehicle_speed_rpt
    /pacmod/enabled
)

# --- Transforms (needed for post-processing alignment) ---
TF_TOPICS=(
    /tf
    /tf_static
)

# --- Nice to have (comment out any you don't need to save disk space) ---
EXTRA_TOPICS=(
    /oak/stereo/image_raw
    /oak/stereo/camera_info
    /pacmod/wheel_speed_rpt
    /navsatfix
)

ALL_TOPICS=(
    "${OBSERVATION_TOPICS[@]}"
    "${ACTION_TOPICS[@]}"
    "${TF_TOPICS[@]}"
    "${EXTRA_TOPICS[@]}"
)

# ── Pre-flight checks ─────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo "ERROR: ros2 not found. Source your ROS 2 workspace first:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  source ~/Development/gem_ws/install/setup.bash"
    exit 1
fi

mkdir -p "${DATA_ROOT}/${SESSION_LABEL}"

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # no color

# ── Topic health check (runs all in parallel, 10 s timeout each) ───────────────
check_topic() {
    local topic="$1"
    if timeout 10 ros2 topic echo --once --qos-reliability best_effort "$topic" > /dev/null 2>&1; then
        echo -e "  ${GREEN}[LIVE]${NC}  $topic"
    else
        echo -e "  ${RED}[DEAD]${NC}  $topic"
    fi
}

# ── Print summary ─────────────────────────────────────────────────────────────
echo "============================================================"
echo "  GEM e4 Imitation Learning — Demo Recording"
echo "============================================================"
echo "  Session  : ${SESSION_LABEL}"
echo "  Bag path : ${BAG_PATH}"
echo "  Max size : $(( MAX_BAG_SIZE / 1000000 )) MB per file (then splits)"
echo "  Topics   : ${#ALL_TOPICS[@]} total"
echo ""
echo "  Make sure each of the following is running in its own terminal"
echo "  (run 'source install/setup.bash' in each terminal first):"
echo ""
echo "    T1: ros2 launch basic_launch sensor_init.launch.py"
echo "    T2: ros2 launch basic_launch corner_cameras.launch.py"
echo "    T3: ros2 launch basic_launch visualization.launch.py"
echo "    T4: ros2 launch basic_launch dbw_joystick.launch.py"
echo ""
echo "  Then verify:"
echo "    - Joystick is connected"
echo "    - DBW is ENABLED (hold LB+RB on Logitech F310)"
echo "    - Be ready to brake manually at any time"
echo ""
read -rp "  Press ENTER to check topics, or Ctrl+C to abort... "

# ── Topic health check ────────────────────────────────────────────────────────
echo ""
echo "  Checking topics (10 s timeout each, running in parallel)..."
echo ""

echo "  --- Observation ---"
PIDS=()
for t in "${OBSERVATION_TOPICS[@]}"; do
    check_topic "$t" &
    PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

echo "  --- Action ---"
PIDS=()
for t in "${ACTION_TOPICS[@]}"; do
    check_topic "$t" &
    PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

echo "  --- Transforms ---"
PIDS=()
for t in "${TF_TOPICS[@]}"; do
    check_topic "$t" &
    PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

echo "  --- Extra ---"
PIDS=()
for t in "${EXTRA_TOPICS[@]}"; do
    check_topic "$t" &
    PIDS+=($!)
done
for pid in "${PIDS[@]}"; do wait "$pid"; done

echo ""
echo -e "  ${YELLOW}NOTE:${NC} EXTRA topics are nice-to-have. DEAD extras won't block recording."
echo -e "  ${YELLOW}NOTE:${NC} Observation + Action topics should all be LIVE."
echo ""
read -rp "  If critical topics are LIVE, press ENTER to start recording, or Ctrl+C to abort... "

# ── Bag size monitor (background) ─────────────────────────────────────────────
monitor_bag() {
    while true; do
        sleep 5
        if [ -d "${BAG_PATH}" ]; then
            size=$(du -sh "${BAG_PATH}" 2>/dev/null | cut -f1)
            echo -e "  ${YELLOW}[$(date +%H:%M:%S)]${NC} Recording... bag size: ${size}"
        fi
    done
}

# ── Record ────────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  Recording started — press Ctrl+C to stop."
echo "============================================================"

monitor_bag &
MONITOR_PID=$!
trap "kill ${MONITOR_PID} 2>/dev/null; echo ''" EXIT

ros2 bag record \
    --output "${BAG_PATH}" \
    --max-bag-size "${MAX_BAG_SIZE}" \
    "${ALL_TOPICS[@]}"

# ── Post-recording summary ────────────────────────────────────────────────────
kill "${MONITOR_PID}" 2>/dev/null || true
echo ""
echo "============================================================"
echo "  Recording stopped."
echo "  Bag saved to: ${BAG_PATH}"
echo ""
echo "  Bag contents:"
ros2 bag info "${BAG_PATH}" 2>/dev/null || true
echo "============================================================"