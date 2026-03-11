#!/usr/bin/env bash
set -euo pipefail

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

# Defensive defaults for strict shells/systemd environments.
: "${MODE:=standalone}"
: "${HOST:=0.0.0.0}"
: "${API_PORT:=4000}"
: "${UI_PORT:=3000}"
: "${PI_IP:=}"
: "${RACE_TOPIC:=/race/lap_event}"
: "${ROS_SETUP:=}"
: "${PRINT_SYSTEMD:=false}"

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
USAGE
}

print_systemd_snippet() {
  local service_user
  local service_home
  local service_ros_setup
  local service_pi_ip
  local pi_ip_arg
  service_user="${SUDO_USER:-$(id -un)}"
  service_home="$(eval echo "~${service_user}")"
  service_ros_setup="${ROS_SETUP:-}"
  service_pi_ip="${PI_IP:-}"

  if [[ -z "$service_pi_ip" ]]; then
    service_pi_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  if [[ -n "$service_pi_ip" ]]; then
    pi_ip_arg=" --pi-ip ${service_pi_ip}"
  else
    pi_ip_arg=""
  fi

  if [[ "$MODE" == "ros2" && -z "$service_ros_setup" ]]; then
    if [[ -f "$service_home/j5/ros_ws/install/setup.bash" ]]; then
      service_ros_setup="$service_home/j5/ros_ws/install/setup.bash"
    elif [[ -f "$REPO_ROOT/ros_ws/install/setup.bash" ]]; then
      service_ros_setup="$REPO_ROOT/ros_ws/install/setup.bash"
    fi
  fi

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
Environment=HOME=${service_home}
Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ExecStart=${REPO_ROOT}/scripts/run_racemanager.sh --mode ${MODE} --host ${HOST} --api-port ${API_PORT} --ui-port ${UI_PORT}${pi_ip_arg} --topic ${RACE_TOPIC}${service_ros_setup:+ --ros-setup ${service_ros_setup}}
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
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

# Re-apply critical defaults after argument parsing to survive partial/merged edits.
PRINT_SYSTEMD="${PRINT_SYSTEMD:-false}"
MODE="${MODE:-standalone}"

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

if [[ -z "$PI_IP" ]]; then
  PI_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi

if [[ "${PRINT_SYSTEMD:-false}" == "true" ]]; then
  print_systemd_snippet
  exit 0
fi
if [[ -z "$PI_IP" ]]; then
  PI_IP="localhost"
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

if [[ ! -x "$VENV_PYTHON" ]]; then
  echo "Virtualenv python not found at $VENV_PYTHON"
  exit 2
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

  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u

  if ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f "$ROS_UNDERLAY_SETUP" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "$ROS_UNDERLAY_SETUP"
      # shellcheck disable=SC1090
      source "$ROS_SETUP"
      set -u
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

export HTTP_HOST="$HOST"
export HTTP_PORT="$API_PORT"
export NEXT_PUBLIC_API_BASE="http://$PI_IP:$API_PORT"
export NEXT_PUBLIC_WS_URL="ws://$PI_IP:$API_PORT/ws"

"$VENV_PYTHON" -m uvicorn apps.racemanager.service.main:app --host "$HOST" --port "$API_PORT" --env-file apps/racemanager/service/.env &
API_PID=$!

(
  cd apps/racemanager/ui
  LOCKFILE=""
  if [[ -f package-lock.json ]]; then
    LOCKFILE="package-lock.json"
  elif [[ -f npm-shrinkwrap.json ]]; then
    LOCKFILE="npm-shrinkwrap.json"
  fi

  DEPS_HASH_SOURCE="package.json"
  if [[ -n "$LOCKFILE" ]]; then
    DEPS_HASH_SOURCE="$LOCKFILE"
  fi

  CURRENT_DEPS_HASH="$(sha256sum "$DEPS_HASH_SOURCE" | awk '{print $1}')"
  STORED_DEPS_HASH=""
  if [[ -f node_modules/.deps-hash ]]; then
    STORED_DEPS_HASH="$(cat node_modules/.deps-hash)"
  fi

  if [[ ! -d node_modules || "$CURRENT_DEPS_HASH" != "$STORED_DEPS_HASH" ]]; then
    npm install >/dev/null
    mkdir -p node_modules
    printf '%s\n' "$CURRENT_DEPS_HASH" > node_modules/.deps-hash
  fi

  npm run dev -- --hostname "$HOST" --port "$UI_PORT"
) &
UI_PID=$!

sleep 2

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
echo "- API (LAN):        http://$PI_IP:$API_PORT"
echo "- WS (LAN):         ws://$PI_IP:$API_PORT/ws"
if [[ "$MODE" == "ros2" ]]; then
  echo "- ROS setup:        $ROS_SETUP"
fi

echo "Press Ctrl+C to stop all processes."
wait
