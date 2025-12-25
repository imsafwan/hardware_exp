#!/bin/bash
# run_ugv.sh
set -e

echo "🚙 Starting UGV task runner..."
python3 ugv_action_test_v4.py &
UGV_TASK_PID=$!

echo "📡 Starting UGV broadcaster..."
python3 task_status_broadcaster.py &
UGV_BROADCAST_PID=$!

echo "🛰️ Starting UGV GPS logger..."
python3 gps_logger.py &
UGV_GPS_PID=$!

# Trap CTRL+C to clean up all processes
trap "echo '🛑 Stopping UGV processes...'; \
      kill $UGV_TASK_PID $UGV_BROADCAST_PID $UGV_GPS_PID 2>/dev/null || true; exit 0" SIGINT

# Wait until all finish
wait
