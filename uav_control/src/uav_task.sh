#!/bin/bash
# run_uav.sh
set -e

echo "🚙 Starting UAV task runner..."
python3 uav_action_test_v4.py &
UAV_TASK_PID=$!

echo "📡 Starting UAV broadcaster..."
python3 task_status_broadcaster.py &
UAV_BROADCAST_PID=$!

# Trap CTRL+C to clean up both processes
trap "echo '🛑 Stopping UAV processes...'; kill $UAV_TASK_PID $UAV_BROADCAST_PID 2>/dev/null || true; exit 0" SIGINT

# Wait until both finish
wait