#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

MODE="standalone"
HOST="0.0.0.0"
API_PORT="4000"
UI_PORT="3000"
PI_IP=""
RACE_TOPIC="/race/lap_event"
ROS_SETUP=""
VENV_DIR="apps/racemanager/service/.venv"
VENV_PYTHON="$VENV_DIR/bin/python"
PRINT_SYSTEMD="false"
DOCTOR_MODE="false"

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

is_port_in_use() {
  local host="$1"
  local port="$2"
  local python_bin="${PYTHON_BIN:-$(find_python_bin || true)}"
  if [[ -z "$python_bin" ]]; then
    echo "Python is required to inspect port availability."
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

ensure_port_available() {
  local name="$1"
  local host="$2"
  local port="$3"
  if is_port_in_use "$host" "$port"; then
    echo "$name port $port is already in use on $host."
    echo "Stop the existing process or choose a different port with --api-port/--ui-port."
    return 1
  fi
}

ensure_process_running() {
  local pid="$1"
  local name="$2"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "$name failed to stay running. Review the logs above and retry."
    return 1
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

find_ros2_bin() {
  local base="$1"
  if [[ -x "$base/install/bin/ros2" ]]; then
    echo "$base/install/bin"
    return 0
  fi
  local candidate
  candidate="$(find "$base/install" -type f -path '*/bin/ros2' 2>/dev/null | head -n 1 || true)"
  if [[ -n "$candidate" ]]; then
    dirname "$candidate"
    return 0
  fi
  return 1
}

usage() {
  cat <<USAGE
Usage: $0 [--mode standalone|ros2] [--host 0.0.0.0] [--api-port 4000] [--ui-port 3000] [--pi-ip <LAN_IP>] [--topic /race/lap_event] [--ros-setup /path/to/install/setup.bash] [--print-systemd]

Starts race manager backend + frontend and optional bridge.
- standalone: bridge runs demo mode (no ROS 2 required)
- ros2: bridge subscribes to ROS 2 topic (requires a source-built ROS 2 workspace)
- print-systemd: print a ready-to-run systemd setup snippet and exit
- doctor: only run dependency/path checks and print diagnostics
- default UI port is 3000 (Next.js). Use --ui-port 5173 only if you want legacy Vite-style port numbering.
USAGE
}

print_systemd_snippet() {
  local service_user
  service_user="${SUDO_USER:-$(id -un)}"
  cat <<SYSTEMD
sudo tee /etc/systemd/system/racemanager.service >/dev/null <<'EOF'
[Unit]
Description=Race Manager stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${service_user}
WorkingDirectory=${REPO_ROOT}
ExecStart=${REPO_ROOT}/scripts/run_racemanager.sh --mode ${MODE} --host ${HOST} --api-port ${API_PORT} --ui-port ${UI_PORT} --pi-ip ${PI_IP} --topic ${RACE_TOPIC}${ROS_SETUP:+ --ros-setup ${ROS_SETUP}}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now racemanager.service
sudo systemctl status racemanager.service --no-pager
SYSTEMD
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --api-port) API_PORT="$2"; shift 2 ;;
    --ui-port) UI_PORT="$2"; shift 2 ;;
    --pi-ip) PI_IP="$2"; shift 2 ;;
    --topic) RACE_TOPIC="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    --print-systemd) PRINT_SYSTEMD="true"; shift ;;
    --doctor) DOCTOR_MODE="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

for cmd in npm; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing dependency: '$cmd' is not on PATH."
    echo "Install once on Ubuntu Server: sudo apt update && sudo apt install -y nodejs npm python3-venv"
    exit 2
  fi
done

if [[ "$MODE" != "standalone" && "$MODE" != "ros2" ]]; then
  echo "--mode must be standalone or ros2"
  exit 1
fi

apply_defaults

if [[ "${PRINT_SYSTEMD:-false}" == "true" ]]; then
  print_systemd_snippet
  exit 0
fi

if [[ "${DOCTOR_MODE:-false}" == "true" ]]; then
  echo "[run_racemanager] doctor: mode=$MODE host=$HOST api_port=$API_PORT ui_port=$UI_PORT"
  echo "[run_racemanager] doctor: repo_root=$REPO_ROOT"
  echo "[run_racemanager] doctor: python=$(find_python_bin || echo missing) npm=$(command -v npm || echo missing)"
  echo "[run_racemanager] doctor: ros2=$(command -v ros2 || echo missing)"
  if [[ "$MODE" == "ros2" ]]; then
    echo "[run_racemanager] doctor: ros_setup=${ROS_SETUP:-auto}"
  fi
  exit 0
fi

if [[ ! -f "apps/racemanager/service/.env" && -f "apps/racemanager/service/.env.example" ]]; then
  cp apps/racemanager/service/.env.example apps/racemanager/service/.env
fi

PYTHON_BIN="$(find_python_bin || true)"
if [[ -z "$PYTHON_BIN" ]]; then
  echo "Python is required but neither 'python3' nor 'python' was found on PATH."
  exit 2
fi

if [[ ! -d "$VENV_DIR" ]]; then
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

"$VENV_PYTHON" -m pip install -r apps/racemanager/service/requirements.txt >/dev/null

if [[ "$MODE" == "ros2" ]]; then
  ROS_UNDERLAY_SETUP="$HOME/ros2_kilted/install/setup.bash"
  if [[ -z "$ROS_SETUP" ]]; then
    if [[ -f "$HOME/j5/ros_ws/install/setup.bash" ]]; then
      ROS_SETUP="$HOME/j5/ros_ws/install/setup.bash"
    elif [[ -f "ros_ws/install/setup.bash" ]]; then
      ROS_SETUP="ros_ws/install/setup.bash"
    fi
  fi

  if [[ -z "$ROS_SETUP" || ! -f "$ROS_SETUP" ]]; then
    echo "ROS 2 mode requested, but no workspace setup file was found."
    echo "Build ROS 2 from source and provide --ros-setup /path/to/install/setup.bash (for example ~/ros2_kilted/install/setup.bash or ~/j5/ros_ws/install/setup.bash)"
    echo "or run --mode standalone."
    exit 2
  fi

  unset VIRTUAL_ENV
  unset PYTHONPATH
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset COLCON_CURRENT_PREFIX

  source_setup_bash "$ROS_SETUP"

  if ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f "$HOME/ros2_kilted/install/setup.bash" ]]; then
      source_setup_bash "$HOME/ros2_kilted/install/setup.bash"
      source_setup_bash "$ROS_SETUP"
    fi
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "Sourced $ROS_SETUP, but 'ros2' is still missing from PATH."
    echo "Source underlay then overlay in-order:"
    echo "  source ~/ros2_kilted/install/setup.bash"
    echo "  source ~/j5/ros_ws/install/setup.bash"
    ROS2_BIN_DIR="$(find_ros2_bin "$HOME/ros2_kilted" || true)"
    if [[ -n "${ROS2_BIN_DIR:-}" ]]; then
      echo "Found ros2 binary under $ROS2_BIN_DIR, but PATH is still missing it."
      export PATH="$ROS2_BIN_DIR:$PATH"
      if command -v ros2 >/dev/null 2>&1; then
        echo "Added $ROS2_BIN_DIR to PATH for this run."
      fi
    else
      echo "ros2 binary not found under ~/ros2_kilted/install (merged or isolated layouts)."
      echo "Check underlay sources include ros2cli: colcon list | rg '^ros2cli\b'"
      echo "Rebuild from a clean workspace with dependencies, e.g.:"
      echo "  rm -rf build install log"
      echo "  colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3"
    fi
    echo "Your source build may be incomplete; run colcon build and retry."
    exit 2
  fi
fi

cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]]; then kill "$BRIDGE_PID" 2>/dev/null || true; fi
  if [[ -n "${UI_PID:-}" ]]; then kill "$UI_PID" 2>/dev/null || true; fi
  if [[ -n "${API_PID:-}" ]]; then kill "$API_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

ensure_port_available "API" "$HOST" "$API_PORT"
ensure_port_available "UI" "$HOST" "$UI_PORT"

export HTTP_HOST="$HOST"
export HTTP_PORT="$API_PORT"
export NEXT_PUBLIC_API_BASE="http://$PI_IP:$API_PORT"
export NEXT_PUBLIC_WS_URL="ws://$PI_IP:$API_PORT/ws"

"$VENV_PYTHON" -m uvicorn apps.racemanager.service.main:app --host "$HOST" --port "$API_PORT" --env-file apps/racemanager/service/.env &
API_PID=$!

(
  cd apps/racemanager/ui
  if [[ ! -d node_modules ]]; then
    npm install >/dev/null
  fi
  npm run dev -- --hostname "$HOST" --port "$UI_PORT"
) &
UI_PID=$!

sleep 2

ensure_process_running "$API_PID" "Backend"
ensure_process_running "$UI_PID" "Frontend"

if [[ "$MODE" == "ros2" ]]; then
  "$VENV_PYTHON" apps/racemanager/bridge/bridge.py --mode ros2 --service-url "http://localhost:$API_PORT" --topic "$RACE_TOPIC" &
  BRIDGE_PID=$!
else
  "$VENV_PYTHON" apps/racemanager/bridge/bridge.py --mode demo --service-url "http://localhost:$API_PORT" &
  BRIDGE_PID=$!
fi

echo "Race Manager started"
echo "- Mode: $MODE"
echo "- Backend: http://localhost:$API_PORT"
echo "- Frontend (local): http://localhost:$UI_PORT"
echo "- Frontend (LAN):   http://$PI_IP:$UI_PORT"
if [[ "$UI_PORT" != "5173" ]]; then
  echo "- Legacy note:      port 5173 belongs to the older race-master-pro Vite app, not this Next.js UI"
  echo "- To use 5173:      rerun with --ui-port 5173"
fi
echo "- API (LAN):        http://$PI_IP:$API_PORT"
echo "- WS (LAN):         ws://$PI_IP:$API_PORT/ws"
if [[ "$MODE" == "ros2" ]]; then
  echo "- ROS setup:        $ROS_SETUP"
fi

echo "Press Ctrl+C to stop all processes."
wait
