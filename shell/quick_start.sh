#!/usr/bin/env bash
set -euo pipefail

# Quick sensor system startup - based on your proven tmux pattern

# Make sure ROOT_SENSOR_DIR is set
if [[ -z "${ROOT_SENSOR_DIR:-}" ]]; then
    echo "Setting up environment..."
    source ../startup.sh
fi

# Default values
DB_NAME="${1:-test_db}"
SESSION_NAME="sensor_system"

# Setup paths
TMP_DB_PATH="$HOME/.local/share/elodin/${DB_NAME}"
TMP_DB_META_PATH="${TMP_DB_PATH}_metadata"
LOG_DIR="${TMP_DB_META_PATH}/log"
TIMESTAMP=$(date +%m_%d_%y__%H_%M_%S)

# Create log directory
mkdir -p "$LOG_DIR"

# Log files
DB_LOG="$LOG_DIR/db_$TIMESTAMP.log"
SENSOR_LOG="$LOG_DIR/sensors_$TIMESTAMP.log"

echo "🚀 Starting Sensor System..."
echo "   Database: $DB_NAME"
echo "   Logs: $LOG_DIR"

# Timing constants
SLEEP_TIME_SHORT=1
SLEEP_TIME_SHELL_ENTER=2
SLEEP_TIME_LONG=3

# Kill existing session if running
tmux has-session -t "$SESSION_NAME" 2>/dev/null && tmux kill-session -t "$SESSION_NAME"

# Start new session - EXACTLY like your working script
tmux new-session -d -s "$SESSION_NAME" -c "$ROOT_SENSOR_DIR"
sleep $SLEEP_TIME_SHELL_ENTER
tmux send-keys -t "$SESSION_NAME" "cd shell && source startup_db.sh $DB_NAME 2>&1 | tee $DB_LOG" C-m
tmux select-pane -t "$SESSION_NAME":0 -T "DB"

# Start a background watcher to wait for "Database is ready!" in log - EXACTLY like your working script
(
    sleep $SLEEP_TIME_LONG
    while true; do
        if grep -q "Database is ready!" "$DB_LOG" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done

    # Once DB is ready, start sensor generator - EXACTLY like your working script
    tmux split-window -h -t "$SESSION_NAME":0 -c "$ROOT_SENSOR_DIR"
    sleep $SLEEP_TIME_SHELL_ENTER
    tmux send-keys -t "$SESSION_NAME":0.1 "cd scripts && ./fake_sensor_generator 127.0.0.1 2240 2>&1 | tee $SENSOR_LOG" C-m
    tmux select-pane -t "$SESSION_NAME":0.1 -T "Sensors"
    sleep $SLEEP_TIME_SHORT

    # Add third pane for visualizer
    tmux split-window -v -t "$SESSION_NAME":0.1 -c "$ROOT_SENSOR_DIR"
    sleep $SLEEP_TIME_SHELL_ENTER
    tmux send-keys -t "$SESSION_NAME":0.2 "elodin" C-m
    tmux select-pane -t "$SESSION_NAME":0.2 -T "Visualizer"

    echo "✅ Sensor system started successfully!"
    echo "   - Database: $DB_NAME"
    echo "   - Logs: $LOG_DIR"
    echo ""
    echo "To attach to the session: tmux attach -t $SESSION_NAME"
    echo "To stop the system: tmux kill-session -t $SESSION_NAME"
) &

echo "Starting sensor system..."
echo "Waiting for database to be ready..."

# Wait a bit for the background process to complete
sleep 5

# Attach to the session - EXACTLY like your working script
tmux attach -t "$SESSION_NAME"
