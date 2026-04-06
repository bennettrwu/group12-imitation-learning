#!/bin/bash
# record_cameras.sh — Record all 4 corner camera feeds
#
# Usage: ./record_cameras.sh <session_label>

set -e

if [ -z "$1" ]; then
    echo "ERROR: session label required."
    echo "Usage: $0 <session_label>"
    exit 1
fi

SESSION_LABEL="$1"
DATA_ROOT="$HOME/CS588_STUDENTS/group_12/data"
BAG_PATH="${DATA_ROOT}/${SESSION_LABEL}/cameras_$(date +%Y%m%d_%H%M%S)"

mkdir -p "${DATA_ROOT}/${SESSION_LABEL}"

echo "Recording 4 corner cameras to: ${BAG_PATH}"
echo "Press Ctrl+C to stop."

ros2 bag record \
    --output "${BAG_PATH}" \
    --max-bag-size 2000000000 \
    --qos-profile-overrides-path <(cat <<EOF
/lucid/camera_fl/image_raw: {reliability: best_effort, durability: volatile}
/lucid/camera_fr/image_raw: {reliability: best_effort, durability: volatile}
/lucid/camera_rl/image_raw: {reliability: best_effort, durability: volatile}
/lucid/camera_rr/image_raw: {reliability: best_effort, durability: volatile}
EOF
) \
    /lucid/camera_fl/image_raw \
    /lucid/camera_fr/image_raw \
    /lucid/camera_rl/image_raw \
    /lucid/camera_rr/image_raw

echo "Done. Bag saved to: ${BAG_PATH}"
ros2 bag info "${BAG_PATH}"