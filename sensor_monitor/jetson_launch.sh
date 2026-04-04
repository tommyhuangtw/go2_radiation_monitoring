#!/usr/bin/env bash
# ==========================================================
#  Jetson Launch Script — starts all Go2 patrol services
#
#  Launches:
#    1. dds_bridge.py    (port 9877 — robot DDS relay)
#    2. sensor_bridge.py (port 9876 — Arduino sensor relay)
#    3. gantry_control.py (port 8090 — stepper motor HTTP API)
#
#  Usage:
#    scp sensor_monitor/jetson_launch.sh unitree@192.168.0.207:~/
#    ssh unitree@192.168.0.207
#    chmod +x ~/jetson_launch.sh
#    ./jetson_launch.sh
#
#  Ctrl+C stops all services cleanly.
# ==========================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="/tmp/go2_logs"
mkdir -p "$LOG_DIR"

# Ports used by each service
PORT_DDS=9877
PORT_SENSOR=9876
PORT_GANTRY=8090

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

PIDS=()

# ----------------------------------------------------------
# Helpers
# ----------------------------------------------------------

log() { echo -e "${CYAN}[launcher]${NC} $1"; }
ok()  { echo -e "  ${GREEN}✓${NC} $1"; }
err() { echo -e "  ${RED}✗${NC} $1"; }
warn() { echo -e "  ${YELLOW}!${NC} $1"; }

kill_port() {
    local port=$1
    local pids
    pids=$(lsof -ti :"$port" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        warn "Killing existing process(es) on port $port (PIDs: $pids)"
        echo "$pids" | xargs kill -9 2>/dev/null || true
        sleep 0.5
    fi
}

detect_port() {
    local pattern=$1
    local ports
    ports=$(ls $pattern 2>/dev/null | head -1)
    echo "$ports"
}

cleanup() {
    echo ""
    log "Shutting down all services..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    # Wait briefly, then force kill stragglers
    sleep 1
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    log "All services stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ----------------------------------------------------------
# Banner
# ----------------------------------------------------------

echo ""
echo -e "${BOLD}========================================${NC}"
echo -e "${BOLD}  Go2 Jetson Service Launcher${NC}"
echo -e "${BOLD}========================================${NC}"
echo ""

# ----------------------------------------------------------
# Step 1: Kill stale processes on all ports
# ----------------------------------------------------------

log "Checking for stale processes..."
kill_port $PORT_DDS
kill_port $PORT_SENSOR
kill_port $PORT_GANTRY

# ----------------------------------------------------------
# Step 2: Detect hardware
# ----------------------------------------------------------

log "Detecting hardware..."

ARDUINO_PORT=$(detect_port "/dev/ttyACM*")
CP2102_PORT=$(detect_port "/dev/ttyUSB*")

if [ -n "$ARDUINO_PORT" ]; then
    ok "Arduino: $ARDUINO_PORT"
else
    warn "Arduino not found (/dev/ttyACM*) — sensor_bridge will be skipped"
fi

if [ -n "$CP2102_PORT" ]; then
    ok "CP2102:  $CP2102_PORT"
else
    warn "CP2102 not found (/dev/ttyUSB*) — gantry_control will be skipped"
fi

echo ""

# ----------------------------------------------------------
# Step 3: Launch services
# ----------------------------------------------------------

# --- DDS Bridge ---
log "Starting DDS bridge (port $PORT_DDS)..."
python3 "$SCRIPT_DIR/dds_bridge.py" --interface eth0 --video --lidar \
    > "$LOG_DIR/dds_bridge.log" 2>&1 &
PIDS+=($!)
ok "DDS bridge started (PID ${PIDS[-1]}, log: $LOG_DIR/dds_bridge.log)"

# --- Sensor Bridge ---
if [ -n "$ARDUINO_PORT" ]; then
    log "Starting sensor bridge (port $PORT_SENSOR)..."
    python3 "$SCRIPT_DIR/sensor_bridge.py" --port "$ARDUINO_PORT" \
        > "$LOG_DIR/sensor_bridge.log" 2>&1 &
    PIDS+=($!)
    ok "Sensor bridge started (PID ${PIDS[-1]}, log: $LOG_DIR/sensor_bridge.log)"
else
    warn "Sensor bridge SKIPPED (no Arduino detected)"
fi

# --- Gantry Controller ---
if [ -n "$CP2102_PORT" ]; then
    log "Starting gantry controller (port $PORT_GANTRY)..."
    python3 "$SCRIPT_DIR/gantry_control.py" --web --port "$CP2102_PORT" \
        > "$LOG_DIR/gantry_control.log" 2>&1 &
    PIDS+=($!)
    ok "Gantry controller started (PID ${PIDS[-1]}, log: $LOG_DIR/gantry_control.log)"
else
    warn "Gantry controller SKIPPED (no CP2102 detected)"
fi

# ----------------------------------------------------------
# Step 4: Verify services are running
# ----------------------------------------------------------

sleep 2
echo ""
log "Verifying services..."

ALL_OK=true
for pid in "${PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        err "Process $pid died — check logs in $LOG_DIR/"
        ALL_OK=false
    fi
done

if $ALL_OK; then
    ok "All services running!"
else
    err "Some services failed to start. Check logs:"
    echo "    tail -f $LOG_DIR/dds_bridge.log"
    echo "    tail -f $LOG_DIR/sensor_bridge.log"
    echo "    tail -f $LOG_DIR/gantry_control.log"
fi

# ----------------------------------------------------------
# Step 5: Print summary
# ----------------------------------------------------------

echo ""
echo -e "${BOLD}========================================${NC}"
echo -e "${BOLD}  SERVICES RUNNING${NC}"
echo -e "${BOLD}========================================${NC}"
echo -e "  DDS Bridge:    port ${GREEN}$PORT_DDS${NC}"
[ -n "$ARDUINO_PORT" ] && \
echo -e "  Sensor Bridge: port ${GREEN}$PORT_SENSOR${NC} ($ARDUINO_PORT)" || \
echo -e "  Sensor Bridge: ${YELLOW}SKIPPED${NC}"
[ -n "$CP2102_PORT" ] && \
echo -e "  Gantry:        port ${GREEN}$PORT_GANTRY${NC} ($CP2102_PORT)" || \
echo -e "  Gantry:        ${YELLOW}SKIPPED${NC}"
echo ""
echo -e "  Logs: ${CYAN}$LOG_DIR/${NC}"
echo -e "  Live: ${CYAN}tail -f $LOG_DIR/*.log${NC}"
echo -e "${BOLD}========================================${NC}"
echo ""
echo -e "  Press ${RED}Ctrl+C${NC} to stop all services."
echo ""

# ----------------------------------------------------------
# Wait for all children
# ----------------------------------------------------------

wait
