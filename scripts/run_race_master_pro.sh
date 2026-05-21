#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
APP_ROOT="$REPO_ROOT/ros_ws/src/j5_perception/race-master-pro"
BACKEND_ROOT="$APP_ROOT/backend"
FRONTEND_ROOT="$APP_ROOT/frontend"
TRACK_CANVAS_FILE="$FRONTEND_ROOT/src/components/track/TrackCanvas.tsx"
RACE_STORE_FILE="$FRONTEND_ROOT/src/stores/raceStore.tsx"

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

print_source_revision_info() {
  echo "[run_race_master_pro] guard_version=2026-05-21b"
  echo "[run_race_master_pro] script_path=$SCRIPT_DIR/run_race_master_pro.sh"
  echo "[run_race_master_pro] repo_root=$REPO_ROOT"
  if command -v git >/dev/null 2>&1; then
    local head_sha
    head_sha="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || true)"
    if [[ -n "$head_sha" ]]; then
      echo "[run_race_master_pro] git_head=$head_sha"
    fi
  fi
}

frontend_parser_guard() {
  local integration_file="$FRONTEND_ROOT/src/components/vision/IntegrationCheckPanel.tsx"
  local vision_file="$FRONTEND_ROOT/src/components/vision/VisionPanel.tsx"
  local ts_parser="$FRONTEND_ROOT/node_modules/typescript/lib/typescript.js"

  if [[ ! -f "$integration_file" || ! -f "$vision_file" ]]; then
    return 0
  fi

  if [[ ! -f "$ts_parser" ]]; then
    echo "[run_race_master_pro] warning: TypeScript parser not found at $ts_parser"
    echo "[run_race_master_pro] warning: skipping frontend parser guard (run 'npm install' in $FRONTEND_ROOT to enable it)"
    return 0
  fi

  if ! node - "$ts_parser" "$integration_file" "$vision_file" <<'JS'
const fs = require('fs')
const ts = require(process.argv[2])
const files = process.argv.slice(3)
let failed = false

for (const file of files) {
  const source = fs.readFileSync(file, 'utf8')
  const result = ts.transpileModule(source, {
    compilerOptions: {
      jsx: ts.JsxEmit.ReactJSX,
      target: ts.ScriptTarget.ES2020,
    },
    fileName: file,
    reportDiagnostics: true,
  })
  const diagnostics = result.diagnostics || []
  const parseDiagnostics = diagnostics.filter((d) =>
    d.category === ts.DiagnosticCategory.Error,
  )

  if (parseDiagnostics.length > 0) {
    failed = true
    console.error(`[run_race_master_pro] parser errors in ${file}:`)
    const sourceFile = ts.createSourceFile(file, source, ts.ScriptTarget.ES2020, true, ts.ScriptKind.TSX)
    for (const diagnostic of parseDiagnostics) {
      const text = ts.flattenDiagnosticMessageText(diagnostic.messageText, '\n')
      const start = typeof diagnostic.start === 'number' ? diagnostic.start : 0
      const pos = sourceFile.getLineAndCharacterOfPosition(start)
      const lineNo = pos.line + 1
      const colNo = pos.character + 1
      const lineText = source.split('\n')[pos.line] || ''
      console.error(`  TS${diagnostic.code} (${lineNo}:${colNo}): ${text}`)
      if (lineText.trim().length > 0) {
        console.error(`    ${lineText}`)
      }
    }
  }
}

process.exit(failed ? 1 : 0)
JS
  then
    echo "Aborting startup due to frontend parser errors in vision panels."
    echo "Tip: if output does not include 'guard_version=' and 'TS#### (line:col)', your run script is stale."
    echo "      Refresh this checkout (git fetch && git pull) and rerun: ./scripts/run_race_master_pro.sh --doctor"
    echo "      Expected current signature: guard_version=2026-05-21b"
    exit 1
  fi
}

if [[ "$DOCTOR_MODE" == "true" ]]; then
  print_source_revision_info
  frontend_parser_guard
  echo "[run_race_master_pro] doctor: app_root=$APP_ROOT"
  echo "[run_race_master_pro] doctor: python=$PYTHON_BIN"
  echo "[run_race_master_pro] doctor: npm=$(command -v npm || echo missing)"
  echo "[run_race_master_pro] doctor: ros2=$(command -v ros2 || echo missing)"
  echo "[run_race_master_pro] doctor: api_port=$API_PORT ui_port=$UI_PORT host=$HOST"
  exit 0
fi

print_source_revision_info
frontend_parser_guard

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

if [[ -f "$TRACK_CANVAS_FILE" ]]; then
  edit_tool_decl_count="$(grep -c "^type EditTool = 'spline' | 'start' | 'finish' | 'checkpoint'$" "$TRACK_CANVAS_FILE" || true)"
  if [[ "${edit_tool_decl_count:-0}" -gt 1 ]]; then
    echo "Detected duplicate EditTool declarations in $TRACK_CANVAS_FILE; auto-repairing stale file copy."
    "$PYTHON_BIN" - "$TRACK_CANVAS_FILE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
lines = path.read_text(encoding="utf-8").splitlines()
needle = "type EditTool = 'spline' | 'start' | 'finish' | 'checkpoint'"
seen = False
out: list[str] = []
for line in lines:
    if line.strip() == needle:
        if seen:
            continue
        seen = True
    out.append(line)
path.write_text("\n".join(out) + "\n", encoding="utf-8")
PY
  fi
fi

if [[ -f "$RACE_STORE_FILE" ]]; then
  if grep -q "case 'positionUpdate':" "$RACE_STORE_FILE"; then
    echo "Detected legacy switch-based websocket handler in $RACE_STORE_FILE; auto-repairing to parser-safe handler."
    "$PYTHON_BIN" - "$RACE_STORE_FILE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
start_marker = "        function handleWsMessage(msg: { type: string, data: Record<string, unknown> }) {"
end_marker = "\n\n        void refreshRaceState()"
start = text.find(start_marker)
end = text.find(end_marker, start)
if start == -1 or end == -1:
    raise SystemExit(0)
replacement = """        function handleWsMessage(msg: { type: string, data: Record<string, unknown> }) {
            if (msg.type === 'raceUpdate') {
                if (msg.data.race) {
                    setLiveRace(msg.data.race as LiveRaceState)
                }
                return
            }

            if (msg.type === 'positionUpdate') {
                if (!msg.data.racer_profile_id || !msg.data) return
                const nextPosition = extractTrackPosition(msg.data)
                    ?? (msg.data as Partial<LiveRacer>).track_position
                    ?? null
                updateRacerPosition(
                    msg.data.racer_profile_id as string,
                    {
                        ...(msg.data as Partial<LiveRacer>),
                        track_position: nextPosition,
                    },
                )
                if (nextPosition) {
                    void evaluateCheckpointProgress(String(msg.data.racer_profile_id), nextPosition)
                }
                return
            }

            if (msg.type === 'visionDetection') {
                handleVisionDetection(msg.data)
                return
            }

            if (
                msg.type === 'raceStart'
                || msg.type === 'racePause'
                || msg.type === 'raceResume'
                || msg.type === 'raceFinish'
                || msg.type === 'lapComplete'
            ) {
                void refreshRaceState(String(msg.data?.race_id || liveRaceRef.current?.race_id || ''))
                return
            }
        }
"""
path.write_text(text[:start] + replacement + text[end:], encoding="utf-8")
PY
  fi
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
