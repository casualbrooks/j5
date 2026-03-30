#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
APP_ROOT="$REPO_ROOT/ros_ws/src/j5_perception/race-master-pro"
BACKEND_ROOT="$APP_ROOT/backend"
FRONTEND_ROOT="$APP_ROOT/frontend"

MODE="standalone"
HOST="0.0.0.0"
API_PORT="8080"
UI_PORT="5173"
PI_IP=""
ROS_SETUP=""
DOCTOR_MODE="false"
VENV_DIR="$BACKEND_ROOT/.venv"
VENV_PYTHON="$VENV_DIR/bin/python"

find_python_bin() {
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return 0
  fi
  if command -v python >/dev/null 2>&1; then
    command -v python
    return 0
  fi
  return 1
}

source_setup_bash() {
  local setup_file="$1"
  local restore_nounset="false"
  if [[ ! -f "$setup_file" ]]; then
    echo "Missing setup script: $setup_file"
    return 1
  fi
  if [[ $- == *u* ]]; then
    restore_nounset="true"
    set +u
  fi
  # shellcheck disable=SC1090
  source "$setup_file"
  local source_status=$?
  if [[ "$restore_nounset" == "true" ]]; then
    set -u
  fi
  return "$source_status"
}

is_port_in_use() {
  local host="$1"
  local port="$2"
  local python_bin="${PYTHON_BIN:-$(find_python_bin || true)}"
  if [[ -z "$python_bin" ]]; then
    return 2
  fi
  "$python_bin" - "$host" "$port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((host, port))
    except OSError:
        raise SystemExit(0)

raise SystemExit(1)
PY
}

ensure_port_available() {
  local name="$1"
  local host="$2"
  local port="$3"
  if is_port_in_use "$host" "$port"; then
    echo "$name port $port is already in use on $host."
    exit 1
  fi
}

ensure_process_running() {
  local pid="$1"
  local name="$2"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "$name failed to stay running. Review the logs above and retry."
    exit 1
  fi
}

apply_defaults() {
  if [[ -z "$PI_IP" ]]; then
    PI_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  if [[ -z "$PI_IP" ]]; then
    PI_IP="localhost"
  fi
}

usage() {
  cat <<USAGE
Usage: $0 [--mode standalone|ros2] [--host 0.0.0.0] [--api-port 8080] [--ui-port 5173] [--pi-ip <LAN_IP>] [--ros-setup /path/to/install/setup.bash] [--doctor]

Starts the Race Master Pro FastAPI backend plus the Vite walkthrough UI.
- standalone: start backend + Vite UI only
- ros2: source ROS setup before launch so CLI/tools are available for wizard checks
- doctor: print dependency/path diagnostics and exit
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --api-port) API_PORT="$2"; shift 2 ;;
    --ui-port) UI_PORT="$2"; shift 2 ;;
    --pi-ip) PI_IP="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    --doctor) DOCTOR_MODE="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

for cmd in npm; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing dependency: '$cmd' is not on PATH."
    exit 2
  fi
done

PYTHON_BIN="$(find_python_bin || true)"
if [[ -z "$PYTHON_BIN" ]]; then
  echo "Python is required but neither 'python3' nor 'python' was found on PATH."
  exit 2
fi

if [[ "$MODE" != "standalone" && "$MODE" != "ros2" ]]; then
  echo "--mode must be standalone or ros2"
  exit 1
fi

apply_defaults

if [[ "$DOCTOR_MODE" == "true" ]]; then
  echo "[run_race_master_pro] doctor: app_root=$APP_ROOT"
  echo "[run_race_master_pro] doctor: python=$PYTHON_BIN"
  echo "[run_race_master_pro] doctor: npm=$(command -v npm || echo missing)"
  echo "[run_race_master_pro] doctor: ros2=$(command -v ros2 || echo missing)"
  echo "[run_race_master_pro] doctor: api_port=$API_PORT ui_port=$UI_PORT host=$HOST"
  exit 0
fi

if [[ "$MODE" == "ros2" ]]; then
  if [[ -z "$ROS_SETUP" ]]; then
    if [[ -f "$HOME/alive/j5/ros_ws/install/setup.bash" ]]; then
      ROS_SETUP="$HOME/alive/j5/ros_ws/install/setup.bash"
    elif [[ -f "$HOME/j5/ros_ws/install/setup.bash" ]]; then
      ROS_SETUP="$HOME/j5/ros_ws/install/setup.bash"
    elif [[ -f "$REPO_ROOT/ros_ws/install/setup.bash" ]]; then
      ROS_SETUP="$REPO_ROOT/ros_ws/install/setup.bash"
    fi
  fi
  if [[ -n "$ROS_SETUP" && -f "$ROS_SETUP" ]]; then
    if [[ -f "/opt/ros/iron/setup.bash" ]]; then
      source_setup_bash "/opt/ros/iron/setup.bash"
    fi
    source_setup_bash "$ROS_SETUP"
  fi
fi

cleanup() {
  if [[ -n "${UI_PID:-}" ]]; then kill "$UI_PID" 2>/dev/null || true; fi
  if [[ -n "${API_PID:-}" ]]; then kill "$API_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

ensure_port_available "API" "$HOST" "$API_PORT"
ensure_port_available "UI" "$HOST" "$UI_PORT"

if [[ ! -d "$VENV_DIR" ]]; then
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

"$VENV_PYTHON" -m pip install -r "$BACKEND_ROOT/requirements.txt" >/dev/null

export VITE_API_BASE_URL="http://$PI_IP:$API_PORT"
export VITE_WS_URL="ws://$PI_IP:$API_PORT/ws"

(
  cd "$BACKEND_ROOT"
  "$VENV_PYTHON" -m uvicorn app.main:app --host "$HOST" --port "$API_PORT"
) &
API_PID=$!

(
  cd "$FRONTEND_ROOT"
  if [[ ! -d node_modules ]]; then
    npm install >/dev/null
  fi
  rm -rf "$FRONTEND_ROOT/node_modules/.vite" 2>/dev/null || true
  npm run dev -- --host "$HOST" --port "$UI_PORT" --force
) &
UI_PID=$!

sleep 2
ensure_process_running "$API_PID" "Backend"
ensure_process_running "$UI_PID" "Frontend"

echo "Race Master Pro started"
echo "- Mode: $MODE"
echo "- Backend (local): http://localhost:$API_PORT"
echo "- Backend (LAN):   http://$PI_IP:$API_PORT"
echo "- Frontend (LAN):  http://$PI_IP:$UI_PORT"
echo "- WebSocket (LAN): ws://$PI_IP:$API_PORT/ws"
echo "- Wizard UI:       open Settings tab in the Vite app to verify/start/pause/resume race services"
if [[ "$MODE" == "ros2" && -n "$ROS_SETUP" ]]; then
  echo "- ROS setup:       $ROS_SETUP"
fi

echo "Press Ctrl+C to stop all processes."
wait
