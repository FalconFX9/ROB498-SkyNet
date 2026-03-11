#!/bin/bash
# record_flight.sh — wrapper around ros2 launch that saves console output and
# vision_pose data to a timestamped directory, keeping only the 5 most recent.
#
# Usage:
#   ./record_flight.sh flight_test_2.launch.py [ros2 launch args...]
#   ./record_flight.sh flight_test_3.launch.py part:=1
#
# Logs are saved to: ~/mary_logs/flight_YYYYMMDD_HHMMSS/
#   console.log      — full stdout/stderr of the launch
#   vision_pose.csv  — /mavros/vision_pose/pose data (via pose_logger_node)

set -e

LAUNCH_FILE="${1:?Usage: $0 <launch_file.py> [args...]}"
shift

LOG_BASE="$HOME/mary_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$LOG_BASE/flight_$TIMESTAMP"

mkdir -p "$LOG_DIR"

# ── Rotate old logs: keep only the 5 most recent ──────────────────────────────
EXISTING=$(ls -dt "$LOG_BASE"/flight_* 2>/dev/null | wc -l)
if [ "$EXISTING" -ge 5 ]; then
    ls -dt "$LOG_BASE"/flight_* 2>/dev/null | tail -n +6 | xargs rm -rf
fi

echo "========================================"
echo "  Flight log: $LOG_DIR"
echo "  Launch:     $LAUNCH_FILE $*"
echo "========================================"

# ── Launch with logging ────────────────────────────────────────────────────────
ros2 launch mary_bringup "$LAUNCH_FILE" \
    record_log:=true \
    log_dir:="$LOG_DIR" \
    "$@" 2>&1 | tee "$LOG_DIR/console.log"
