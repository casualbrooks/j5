#!/usr/bin/env bash
# Single backend process: browser UI, camera timing, API and persistent SQLite.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$ROOT/ros_ws/src/j5_perception/race-master-pro/backend"
STATE="${RACE_DATA_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/j5-racemanager}"
VENV="${RACE_VENV:-$STATE/venv}"
mkdir -p "$STATE"

if [[ "${1:-}" == "--install" ]]; then
  python3 -m venv --system-site-packages "$VENV"
  "$VENV/bin/python" -m pip install -r "$APP/requirements.txt"
  exit
fi

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Install once with: bash scripts/run_race_web.sh --install" >&2
  exit 1
fi

# One underlay only. No inherited mixed-workspace overlays are sourced here.
if [[ -n "${ROS_UNDERLAY:-}" ]]; then
  source "$ROS_UNDERLAY"
elif [[ -f /opt/ros/kilted/setup.bash ]]; then
  source /opt/ros/kilted/setup.bash
elif [[ -f "$HOME/ros2_kilted/install/setup.bash" ]]; then
  source "$HOME/ros2_kilted/install/setup.bash"
fi
export DATABASE_PATH="${DATABASE_PATH:-$STATE/racetracker.db}"
exec "$VENV/bin/python" -m uvicorn app.main:app --app-dir "$APP" \
  --host "${RACE_HOST:-127.0.0.1}" --port "${RACE_PORT:-8080}" --workers 1
