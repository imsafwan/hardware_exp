#!/bin/bash
# run_ugv.sh
set -e

echo "🚙 Starting ugv task runner..."
python3 ugv_action_test_v3.py &
ugv_TASK_PID=$!

echo "📡 Starting ugv broadcaster..."
python3 task_status_broadcaster.py &
ugv_BROADCAST_PID=$!

# Trap CTRL+C to clean up both processes
trap "echo '🛑 Stopping ugv processes...'; kill $ugv_TASK_PID $ugv_BROADCAST_PID 2>/dev/null || true; exit 0" SIGINT

# Wait until both finish
wait