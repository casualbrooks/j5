# Documentation

This folder contains user and developer documentation for J5.

Contents
- BOM.md – Bill of Materials
- SAFETY.md – Safety and risk guidance
- CONTRIBUTING.md – Contribution guidelines
- LICENSES.md – Third‑party licenses
- use_cases/track_lap_counter.md – Vision lap counter appliance use case
- race_manager.md – Web race manager folder layout and lap counter integration

Getting started
- See the top‑level README.md for quick start.
- For ROS 2 build steps, see below.

ROS 2 on Windows (PowerShell)
1) Ensure ROS 2 Kilted Kaiju is installed and added to your environment. In PowerShell:
   - `$env:ChocolateyInstall` should be set if you used Chocolatey.
   - Run the shortcut "ROS 2 <distro> > ROS 2 Developer PowerShell" or dot‑source the setup script.
2) Build
   - `cd j5/ros_ws`
   - `colcon build --symlink-install`
   - `.\install\local_setup.ps1`
3) Run demos
   - Placeholder until packages are implemented.


ROS 2 source-build troubleshooting (Linux)

### Run this first in a fresh shell
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
export Python3_EXECUTABLE=/usr/bin/python3
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
echo "$COLCON_CURRENT_PREFIX"
```

### Fix `No module named catkin_pkg`
```bash
sudo apt update
sudo apt install -y python3-catkin-pkg
```

### Build ROS 2 underlay (default command)
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

### If `ament_cmake` is missing, bootstrap it first
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --packages-up-to ament_cmake --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### If `rmw_zenoh_cpp` fails (GCC13 / zenoh optional-copy errors)
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
command -v ros2
```

### If `local_setup.bash` or `rmw_implementation` reports missing `rmw_zenoh_cpp/package.sh`
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

### If `ros2` is missing after sourcing overlay
```bash
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
command -v ros2
```

### If `command -v ros2` is still empty, rebuild CLI packages
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b'
find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 5

rm -rf build install log
colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

If `ros2cli` fails with missing `install/rmw_zenoh_cpp/.../package.sh`, that means a skipped package is still being referenced by a dependent; use both `--packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp` for ros2cli recovery so the failing package and its dependents are excluded.

### If rosdep fails with `Cannot locate rosdep definition for [ament_python]`
```bash
sudo rosdep init 2>/dev/null || true
rosdep update
source ~/ros2_kilted/install/setup.bash
cd ~/j5/ros_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"
```

### Optional PATH repair when `ros2` exists but is not on PATH
```bash
ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 1)"
export PATH="$(dirname "$ROS2_BIN"):$PATH"
command -v ros2
```

### Quick log triage commands
```bash
rg 'Failed <<<' ~/ros2_kilted/log/latest_build/*
rg 'Aborted <<<' ~/ros2_kilted/log/latest_build/*
rg 'stderr output' ~/ros2_kilted/log/latest_build/*
```


## Copy-paste recovery sequences (Linux)

### 1) Fresh shell + source order check
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
command -v ros2
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
```

### 1b) Build-local ROS 2 path check (no system `/opt` fallback)
```bash
if ! command -v ros2 >/dev/null 2>&1; then
  [ -f ~/ros2_kilted/install/setup.bash ] && source ~/ros2_kilted/install/setup.bash
  [ -f ~/j5/ros_ws/install/setup.bash ] && source ~/j5/ros_ws/install/setup.bash
fi
command -v ros2 || true
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
echo "$COLCON_CURRENT_PREFIX"
```

Use the command above for source-build workflows; do not rely on `/opt/ros/...` or `~/alive/...` paths in this runbook.

### 2) Build underlay while skipping zenoh dependency chain
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 3) If ros2 CLI is still missing
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b'
find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 5

rm -rf build install log
colcon build --symlink-install   --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp   --packages-skip rmw_zenoh_cpp   --packages-skip-by-dep rmw_zenoh_cpp   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

### 4) rosdep for J5 overlay (source-build context)
```bash
sudo rosdep init 2>/dev/null || true
rosdep update
source ~/ros2_kilted/install/setup.bash
cd ~/j5/ros_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"
```


## ARM Ubuntu 24.04 source-build verification plan (step-by-step)

Use this as a strict checklist on a fresh Pi 5 / ARM64 server install. Do not move to the next step until the current step's verification command succeeds.

### Step 0: Host sanity (OS, arch, toolchain)
```bash
uname -a
dpkg --print-architecture
cat /etc/os-release
python3 --version
gcc --version
g++ --version
cmake --version
```

Expected:
- `aarch64` / `arm64`
- Ubuntu `24.04`
- Python 3.12.x
- GCC/G++ available

### Step 1: Install base build + ROS tooling prerequisites
```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  locales \
  software-properties-common \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-rosinstall-generator \
  python3-catkin-pkg \
  python3-empy \
  python3-numpy \
  python3-setuptools \
  python3-pytest \
  python3-pytest-repeat \
  libasio-dev \
  libtinyxml2-dev
```

Verify:
```bash
command -v colcon
command -v vcs
command -v rosdep
python3 -c "import catkin_pkg, em, numpy, setuptools; print('python deps ok')"
```

### Step 2: Initialize rosdep metadata
```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Verify:
```bash
rosdep db > /tmp/rosdep-db.txt
sed -n '1,8p' /tmp/rosdep-db.txt
```

### Step 3: Create clean ROS 2 source workspace
```bash
mkdir -p ~/ros2_kilted/src
cd ~/ros2_kilted
curl -L https://raw.githubusercontent.com/ros2/ros2/kilted/ros2.repos -o ros2.repos
vcs import src < ros2.repos
```

Verify:
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b|^rclcpp\b|^ament_cmake\b' -n
```

### Step 4: Install source workspace system dependencies
```bash
cd ~/ros2_kilted
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted
```

If one key is unresolved, verify and continue:
```bash
rosdep check --from-paths src --ignore-src --rosdistro kilted || true
```

Important: `rosdep install/check` only validates **system** dependencies. It does **not** build ROS packages and does not create `install/.../bin/ros2`.

### Step 5: Clean environment before building
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
export Python3_EXECUTABLE=/usr/bin/python3
```

Verify:
```bash
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
echo "$COLCON_CURRENT_PREFIX"
python3 -c "import sys; print(sys.executable)"
```

### Step 6: Build ROS 2 underlay (first attempt)
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If your log shows `Starting >>> rmw_zenoh_cpp` and then the GCC13 `std::optional<zenoh::CancellationToken>` error, the command above is working as expected and your compiler is not generally broken; this is the known zenoh/rmw incompatibility path on some ARM toolchains.

If your command line does **not** include `--packages-skip rmw_zenoh_cpp`, colcon will still build `rmw_zenoh_cpp`.

If this fails in `rmw_zenoh_cpp`, `rmw_test_fixture_implementation`, **or** `zenoh_security_tools`, use explicit skip package list:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Verify:
```bash
source ~/ros2_kilted/install/setup.bash
command -v ros2
find ~/ros2_kilted/install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
ros2 --help | head -n 5
```

If `command -v ros2` is empty but `find ... '*/ros2cli*/bin/ros2'` returns a path, run the binary directly to prove install success and then fix shell PATH/sourcing:
```bash
ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/ros2cli*/bin/ros2' | head -n 1)"
"$ROS2_BIN" --help | head -n 5
export PATH="$(dirname "$ROS2_BIN"):$PATH"
command -v ros2
```

If some packages were `Aborted` or `not processed`, immediately rerun the same command once more to complete packages that were blocked by the first failure:
```bash
cd ~/ros2_kilted
colcon build --symlink-install \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

If Summary shows `0 packages failed` but you skipped core runtime packages (for example `ros2cli`, `rclpy`, `rcl`, `pluginlib`), run a follow-up build that restores those packages and only excludes the zenoh dependency chain:
```bash
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

Do not keep `ros2cli` in `--packages-skip` for final recovery builds; that prevents the `ros2` command from being installed.

For ros2cli recovery runs, keep both flags together: `--packages-skip rmw_zenoh_cpp` skips the failing package itself, and `--packages-skip-by-dep rmw_zenoh_cpp` skips dependents that still reference it.

If build Summary shows many `stderr output` packages but **no `Failed <<<` lines**, treat the build as successful and continue:
```bash
cd ~/ros2_kilted
source install/setup.bash
command -v ros2
ros2 --help | head -n 5
```

For outputs like `Summary: 132 packages finished ... 33 packages had stderr output` with no failed package line, proceed to underlay sanity checks (this is not a build failure):
```bash
cd ~/ros2_kilted
source install/setup.bash
command -v ros2
ros2 pkg list | rg '^ros2cli$|^rclcpp$' -n
```

If the summary matches the pattern above and `fastdds` / `rmw_fastrtps_*` finished, treat underlay build as complete and move to overlay build:
```bash
cd ~/j5/ros_ws
source ~/ros2_kilted/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"
rm -rf build install log
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

If `source install/setup.bash` succeeds but `command -v ros2` is still empty, run this exact triage block:
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b'
find install -type f -path '*/bin/ros2' | head -n 10
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH"
echo "$COLCON_CURRENT_PREFIX"
```

Interpretation for this triage block:
- `colcon list | rg '^ros2cli\b'` only proves `ros2cli` exists in `src/`; it does **not** prove it was built/installed.
- If `find install -type f -path '*/bin/ros2'` prints nothing, the CLI binary is not installed yet.
- If `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` already contain many overlay paths (for example `~/j5/ros_ws/install/...`) while building underlay, clear them before rebuilding underlay.
- Repeated `WARNING:colcon.colcon_ros.prefix_path.ament:The path '.../install/<pkg>' ... doesn't exist`, `WARNING:colcon.colcon_ros.prefix_path.catkin:The path '.../install/<pkg>' ... doesn't exist`, or bare `AMENT_PREFIX_PATH doesn't exist` output all indicate stale prefix-path entries in the current shell.
- This usually happens when `install/` was deleted (`rm -rf build install log`) in the same shell that previously sourced `install/setup.bash`.
- Stale prefix entries can pollute dependency discovery and cause follow-on `ament_python` failures (for example while building `ros2cli`).

To detect stale prefix entries quickly:
```bash
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; do
  echo "== $var =="
  tr ':' '\n' <<< "${!var}" | sed '/^$/d'
  tr ':' '\n' <<< "${!var}" | sed '/^$/d' | while read -r p; do
    [ -d "$p" ] || echo "MISSING: $p"
  done
done
```

If warnings persist after `unset`, launch a clean build shell that strips inherited path variables before running colcon:
```bash
cd ~/ros2_kilted
env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="${TERM:-xterm}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  /bin/bash --noprofile --norc -lc '
    set -e
    export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
    rm -rf build install log
    colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    source install/setup.bash
    command -v ros2
  '
```

Important: `env -i ... /bin/bash -lc '...'` runs in a child shell. A successful `command -v ros2` inside that block verifies install artifacts, but it does **not** persist environment changes back to your current terminal. After it finishes, run:
```bash
cd ~/ros2_kilted
source install/setup.bash
command -v ros2
```

What your posted long build log means (important):
- `Summary: 132 packages finished ...` and no `Failed <<<` line means the underlay build itself succeeded.
- The repeated `setuptools ... Unbuilt egg for pytest-repeat` lines are warnings, not fatal errors.
- The common reason for `ros2` still appearing missing after this exact `env -i` command is shell scope (child shell vs current shell), not package compile failure.

Use this exact post-build proof in your current shell:
```bash
cd ~/ros2_kilted
source install/setup.bash
ROS2_BIN="$(find install -type f -path '*/bin/ros2' | head -n 1)"
echo "ros2 bin: ${ROS2_BIN:-<not found>}"
[ -n "$ROS2_BIN" ] && "$ROS2_BIN" --help | head -n 5
command -v ros2
```

If `ROS2_BIN` is found but `command -v ros2` is still empty, your install is good and only PATH/sourcing is missing in the current shell.

If `ROS2_BIN` is `<not found>` after sourcing, the issue is not shell PATH; `ros2` was never installed. Check whether `ros2cli` actually built:
```bash
cd ~/ros2_kilted
rg '^(Finished|Failed) <<< ros2cli' log/latest_build/* || true
colcon list --names-only   --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp   --packages-skip rmw_zenoh_cpp   --packages-skip-by-dep rmw_zenoh_cpp | rg '^ros2cli$' -n || true
```

Interpretation:
- If `Finished <<< ros2cli` is missing, no `ros2` entrypoint can exist.
- If the package-selection command does not print `ros2cli`, your skip set excluded it.

If `rg '^(Finished|Failed) <<< ros2cli'` prints nothing, run these exact checks before rebuilding:
```bash
cd ~/ros2_kilted
[ -d log/latest_build ] && rg -n '<<< ros2cli' log/latest_build/* || echo 'no latest_build logs yet'
find install -type f -path '*/ros2cli*/package.sh' | head -n 5
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

How to read these results:
- `ros2cli` listed by `colcon list` only confirms source checkout, not build/install.
- `install/.../ros2cli/package.sh` without `install/.../ros2cli/bin/ros2` means incomplete `ament_python` install step for ros2cli.
- No `log/latest_build` evidence for ros2cli usually means ros2cli was never scheduled in the selected package set for that build invocation.
- If you get `no latest_build logs yet`, either no completed `colcon build` has run since `log/` was cleaned, or `log/latest_build` is stale while `log/latest` now points to a `colcon list` run (`latest_list`).
- If `install/ros2cli` exists while `log/latest_build/ros2cli/` is missing, reconcile with `logger_all.log`: an upstream failure (commonly `ament_cmake_ros`) can abort the graph before ros2cli runs in the *current* build, even though ros2cli artifacts may exist from an older successful build.

When `log/latest_build` is missing or suspicious, run this fallback evidence block:
```bash
cd ~/ros2_kilted
ls -la log || true
readlink -f log/latest 2>/dev/null || true
readlink -f log/latest_build 2>/dev/null || true
[ -f log/latest_build/logger_all.log ] && rg -n 'Summary:|Failed <<<|Aborted <<<|<<< ros2cli' log/latest_build/logger_all.log || true
```

Note: running `colcon list` updates `log/latest` to `latest_list`, so `log/latest/*` may contain only list/debug metadata and no build/package completion data. For build diagnosis, prefer `log/latest_build/*` and especially `log/latest_build/logger_all.log`.

This matches the Kilted source-build flow in the upstream ROS 2 docs: underlay build health must be restored first (ament/rcl layers), then CLI packages (`ros2cli`, `ros2run`, `ros2topic`) are expected to install. In short, treat missing `rcl*` package hooks as the primary blocker and `ros2cli` absence as downstream fallout.

How to use `ros2cli/package.sh` correctly:
- Do **not** execute `install/ros2cli/share/ros2cli/package.sh` directly as a command.
- `package.sh` is an environment hook meant to be sourced by `install/setup.bash` / `install/local_setup.bash`.
- Normal usage is:
```bash
cd ~/ros2_kilted
source install/setup.bash
command -v ros2
ros2 --help | head -n 5
```
- If you need to debug just ros2cli's hook, source it (not execute it):
```bash
cd ~/ros2_kilted
source install/ros2cli/share/ros2cli/package.sh
command -v ros2
```

Recovery for this exact case (`ros2 bin: <not found>`): run a clean build that keeps `ros2cli` in scope and only skips known zenoh-problem packages:
```bash
cd ~/ros2_kilted
env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="${TERM:-xterm}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin   /bin/bash --noprofile --norc -lc '
    set -e
    export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
    rm -rf build install log
    colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    rg "^(Finished|Failed) <<< ros2cli" log/latest_build/* || true
    find install -type f -path "*/ros2cli*/bin/ros2" | head -n 5
  '
```

Then in your current shell:
```bash
cd ~/ros2_kilted
source install/setup.bash
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
command -v ros2
```

Before starting another 50-minute rebuild, run this fast-fail preflight to prove `ros2cli` is in the selected package set:
```bash
cd ~/ros2_kilted
colcon list --names-only   --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp   --packages-skip rmw_zenoh_cpp   --packages-skip-by-dep rmw_zenoh_cpp | rg '^ros2cli$|^ros2run$|^ros2topic$' -n
```

Interpretation:
- If `ros2cli` is missing from this preflight output, the selection/skip combination has excluded it; a full rebuild will not install `ros2`.
- In that case, remove `--packages-skip-by-dep rmw_zenoh_cpp` for the first pass and only keep `--packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation`, then re-check the selected set before building.
- If `log/latest_build/logger_all.log` (or a known build log) shows `Skipping package 'ros2cli'` / `Skipping package 'ros2run'` / `Skipping package 'ros2topic'`, your current skip set has explicitly filtered out the CLI chain.

To avoid blind loops, save a deterministic package-order log before each long build:
```bash
cd ~/ros2_kilted
colcon list --topological-order --names-only > /tmp/colcon-topo-before-build.txt
sed -n '1,120p' /tmp/colcon-topo-before-build.txt
rg -n '^ros2cli$|^ros2run$|^ros2topic$' /tmp/colcon-topo-before-build.txt || true
```

Important: the first 120 lines are only a preview. `ros2cli` often appears later in large workspaces; always `rg '^ros2cli$' /tmp/colcon-topo-before-build.txt` before concluding it is missing.

If `rg '^ros2cli$' /tmp/colcon-topo-before-build.txt` returns nothing, your current source graph does not include ros2cli in this workspace resolution (wrong branch/repos import or unexpected workspace contents), so `log/latest_build/ros2cli` will not exist for that build.

If you are not using `env -i`, do this in a brand-new terminal before running `colcon`:
```bash
# 1) open a new shell (recommended)
# 2) do NOT source ~/ros2_kilted/install/setup.bash before cleaning
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
export AMENT_PREFIX_PATH=""
export CMAKE_PREFIX_PATH=""
export COLCON_CURRENT_PREFIX=""
cd ~/ros2_kilted
rm -rf build install log
colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Then rebuild CLI packages **without** skipping `ros2cli` and with zenoh dependency skipping only:
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

Do **not** skip core ROS graph/runtime packages in this recovery path (for example: `rcl`, `rclpy`, `rcl_action`, `rcl_lifecycle`, `rcl_logging_*`, `pluginlib`, `class_loader`, `ros2cli`). Skipping these often creates new failures (for example `class_loader` failed, `lifecycle_msgs` aborted) and leaves the underlay incomplete.

Common command mistakes to avoid:
```bash
# wrong: missing leading '--' before packages-skip-by-dep
packages-skip-by-dep rmw_zenoh_cpp

# correct
--packages-skip-by-dep rmw_zenoh_cpp
```

If `command -v ros2` is still empty after that, verify ros2cli install artifacts directly:
```bash
find install -type f -path '*/ros2cli*/package.sh' | head -n 5
find install -type f -path '*/bin/ros2' | head -n 10
```

`ros2cli` -> `ros2` mapping (what you should expect):
- `ros2` is the console entrypoint installed by the `ros2cli` package (ament_python).
- In non-merge installs, it usually lands at `~/ros2_kilted/install/ros2cli/bin/ros2`.
- You do **not** manually build a separate `bin/ros2`; it appears automatically when `ros2cli` is successfully installed.

Quick proof commands:
```bash
cd ~/ros2_kilted
find install -type f -path '*/ros2cli*/package.sh' | head -n 5
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
find install -type f -path '*/bin/ros2' | head -n 10
```

If `ros2cli*/package.sh` exists but `ros2cli*/bin/ros2` does not, treat that as a failed/incomplete `ament_python` install and rebuild up-to ros2cli in a clean shell.

If both commands print nothing, `ros2cli` is present in source but not installed in the current install tree. Run this exact recovery (single line command, no missing `--` prefixes):
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp --packages-skip-by-dep rmw_zenoh_cpp --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
find install -type f -path '*/bin/ros2' | head -n 10
```

If `find ... '*/bin/ros2'` finds a path but `command -v ros2` is still empty, export that bin dir and re-source:
```bash
ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 1)"
export PATH="$(dirname "$ROS2_BIN"):$PATH"
source ~/ros2_kilted/install/setup.bash
command -v ros2
```

If there are no `Failed <<<` lines and `command -v ros2` is still empty, run this focused sanity pass to confirm ros2cli dependencies are built and to show where `ros2` landed:
```bash
cd ~/ros2_kilted
colcon list | rg '^ros2cli\b'
rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp \
  --packages-skip-by-dep rmw_zenoh_cpp \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
find install -type f -path '*/ros2cli*/package.sh' | head -n 5
find install -type f -path '*/bin/ros2' | head -n 10
```

If `ros2` is still missing and you suspect `ros2cli` did not finish its install phase, run this focused debug pass (captures ros2cli stdout/stderr directly):
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install   --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3   --event-handlers console_direct+
```

Immediately inspect ros2cli build/install evidence:
```bash
cd ~/ros2_kilted
ls -la log/latest_build/ros2cli || true
sed -n '1,220p' log/latest_build/ros2cli/stdout_stderr.log 2>/dev/null || true
rg -n 'Traceback|ModuleNotFoundError|entry_points|Installing|Failed|ERROR' log/latest_build/ros2cli/* 2>/dev/null || true
find install -type f -path '*/ros2cli*/package.sh' | head -n 5
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

Interpretation for this debug pass:
- If `log/latest_build/ros2cli/` does not exist, ros2cli was never scheduled in that build graph.
- If ros2cli logs show Python traceback/module errors, fix those first (that is the reason `bin/ros2` is absent).
- If ros2cli logs show success but `install/.../ros2cli/bin/ros2` is still absent, inspect install hooks:
```bash
source ~/ros2_kilted/install/setup.bash
python3 -c "import importlib.util as iu; print('ros2cli module:', bool(iu.find_spec('ros2cli')))"
```

If your `install/` tree contains many packages (for example `builtin_interfaces`, `rcutils`, `rmw_*`) but **no `ros2cli/` directory at all**, treat that as a partial underlay where ros2cli and its chain were never installed.

If your logs show many `-- Symlinking: .../install/<pkg>/...` lines (for example `action_msgs`, `ament_*`) that only proves those specific packages installed; it does **not** prove ros2cli ran.

Before cleaning/restarting, check whether the previous build actually reached summary/completion:
```bash
cd ~/ros2_kilted
rg -n 'Summary:|Failed <<<|Aborted <<<' log/latest_build/logger_all.log 2>/dev/null || true
tail -n 40 log/latest_build/logger_all.log 2>/dev/null || true
```

Interpretation:
- If `logger_all.log` has no `Summary:` line, the build was interrupted before completion (timeout/SSH drop/Ctrl-C).
- In that case, resume first **without** deleting `build/ install/ log/`:
```bash
cd ~/ros2_kilted
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_direct+
```

Run this exact proof block:
```bash
cd ~/ros2_kilted
ls -d install/ros2cli 2>/dev/null || echo 'install/ros2cli missing'
ls -d install/rcl install/rclpy install/rcl_action install/rcl_lifecycle 2>/dev/null || true
colcon list --names-only --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp | rg '^ros2cli$|^rcl$|^rclpy$' -n || true
```

If `install/ros2cli` is missing, do not debug PATH yet; rebuild the ros2cli chain first:
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_direct+
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

If your build summary shows this pattern instead:
- `1 package failed: ros2cli`
- aborted packages like `composition_interfaces`, `statistics_msgs`, `std_srvs`

then inspect ros2cli's failure reason first (do not infer from aborted packages):
```bash
cd ~/ros2_kilted
sed -n '1,220p' log/latest_build/ros2cli/stdout_stderr.log 2>/dev/null || true
rg -n 'ERROR|Traceback|ModuleNotFoundError|Failed to find the following files|package.sh' log/latest_build/ros2cli/* 2>/dev/null || true
```

Most common cause in this scenario: missing dependency package scripts in install tree (for example `rmw_implementation`, `rcl`, `rclpy`), which means ros2cli was scheduled but its prerequisites were incomplete.

Upstream dependency reconciliation (ros2cli repo vs ROS docs):
- In `ros2/ros2cli`, the `ros2cli` package is `ament_python` and relies on the ROS client stack via `rclpy` and related `rcl*` runtime packages.
- The official Kilted Ubuntu development setup builds a coherent underlay first, then uses `install/setup.bash`; it does not assume `ros2cli` can succeed if `rcl`, `rclpy`, `rcl_action`, `rcl_lifecycle`, `rcl_logging_interface`, `rcl_yaml_param_parser`, `rcl_logging_noop`, or `rcl_logging_spdlog` are missing from `install/`.
- Therefore, if your missing set is exactly those `rcl*` packages, the discrepancy is underlay completeness, not a PATH-only problem.

Use this prerequisite check before retrying ros2cli:
```bash
cd ~/ros2_kilted
for p in rcl rclpy rcl_action rcl_lifecycle rcl_logging_interface rcl_yaml_param_parser rcl_logging_noop rcl_logging_spdlog; do
  [ -f "install/$p/share/$p/package.sh" ] && echo "OK $p" || echo "MISSING $p"
done
```

If any are missing, build prerequisites first, then ros2cli:
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install   --packages-up-to rclpy rclcpp   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3   --event-handlers console_direct+
colcon build --symlink-install   --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp   --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3   --event-handlers console_direct+
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

If this two-stage command fails early with `1 package failed: ament_cmake_ros` (and aborts `std_msgs` / `lifecycle_msgs` / `composition_interfaces`), bootstrap ROS core CMake packages first, then resume:
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install --packages-up-to ament_cmake_ros --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 --event-handlers console_direct+
sed -n '1,220p' log/latest_build/ament_cmake_ros/stdout_stderr.log 2>/dev/null || true
rg -n 'ERROR|Traceback|ModuleNotFoundError|Failed|not found|package.sh' log/latest_build/ament_cmake_ros/* 2>/dev/null || true
colcon build --symlink-install --packages-up-to rclpy rclcpp --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 --event-handlers console_direct+
colcon build --symlink-install --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 --event-handlers console_direct+
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

If `log/latest_build/ros2cli/` is empty but `ament_cmake_ros` failed, debug `ament_cmake_ros` first; ros2cli is downstream and will not produce logs until this underlay stage succeeds.

Re-check the problematic set after this recovery:
```bash
for p in rcl rclpy rcl_action rcl_lifecycle rcl_logging_interface rcl_yaml_param_parser rcl_logging_noop rcl_logging_spdlog; do
  [ -f "install/$p/share/$p/package.sh" ] && echo "OK $p" || echo "MISSING $p"
done
```

Recovery command for this exact summary pattern:
```bash
cd ~/ros2_kilted
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH VIRTUAL_ENV
rm -rf build install log
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_direct+
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

If your build ends with this pattern:
- `1 package failed: rmw_implementation`
- aborted packages like `composition_interfaces`, `lifecycle_msgs`, `statistics_msgs`

then rerun with `rmw_implementation` temporarily skipped so the CLI chain can finish, then verify `ros2` artifact creation:
```bash
cd ~/ros2_kilted
colcon build --symlink-install \
  --packages-up-to ros2cli ros2run ros2topic demo_nodes_cpp \
  --packages-skip rmw_zenoh_cpp rmw_test_fixture_implementation zenoh_security_tools rmw_implementation \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_direct+
find install -type f -path '*/ros2cli*/bin/ros2' | head -n 5
```

Then pin runtime to Fast DDS in this underlay:
```bash
source ~/ros2_kilted/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
command -v ros2
ros2 --help | head -n 5
```

If `colcon build --packages-select ros2cli` fails with missing `.../package.sh` files (`rmw_implementation`, `rcl`, `rclpy`, etc.), that is expected in a partially built workspace; use the `--packages-up-to ... --packages-skip ... --packages-skip-by-dep ...` command above instead.

### Step 7: Validate middleware/runtime environment
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 doctor --report || true
```

Optional: if you want to force Cyclone DDS for runtime checks, install and verify it before proceeding:
```bash
sudo apt update
sudo apt install -y ros-kilted-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Verify:
```bash
echo "$RMW_IMPLEMENTATION"
ros2 pkg list | rg '^rclcpp$|^ros2cli$' -n
```

Path note (source-build workflow): use `~/ros2_kilted/...` and `~/j5/ros_ws/...` paths for underlay/overlay commands in this guide.

If you now see all of the following, your underlay is good enough and you should move on to overlay:
- `source ~/ros2_kilted/install/setup.bash` succeeds with no errors
- `find ~/ros2_kilted/install -type f -path "*/ros2cli*/bin/ros2" | head -n 1` returns a path
- `command -v ros2` resolves to that underlay path (for example `~/ros2_kilted/install/ros2cli/bin/ros2`)
- `ros2 pkg list | rg "^ros2cli$|^rclcpp$" -n` prints both packages

### Step 8: Build J5 overlay only after underlay passes
```bash
cd ~/j5/ros_ws
source ~/ros2_kilted/install/setup.bash
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y --rosdistro kilted --skip-keys "ament_python"
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
command -v ros2
```

### Step 9: Failure triage commands (compiler vs dependency vs environment)
```bash
cd ~/ros2_kilted
rg 'Failed <<<' log/latest_build/*
rg 'error:' log/latest_build/* | tail -n 40
rg 'No module named|Cannot locate rosdep definition|package.sh|local_setup.bash|Symlinking:' log/latest_build/*
```

Interpretation:
- `No module named ...` => missing Python package / wrong interpreter path.
- `-- Symlinking: .../install/<pkg>/share/<pkg>/local_setup.bash` in `stdout.log` / `streams.log` is positive evidence that package `<pkg>` installed successfully.
- Missing `package.sh` / `local_setup.bash` for downstream package requirements => partial install tree from failed dependency chain; clean rebuild required.
- GCC/C++ template errors in zenoh packages => toolchain/vendor incompatibility; skip affected package(s), continue with Fast DDS/Cyclone.
- `Manually-specified variables were not used by the project: Python3_EXECUTABLE` in vendor packages is usually benign.
- `Unbuilt egg for pytest-repeat` warnings from setuptools are usually non-fatal warnings in this context.

### Step 10: About egg warnings and `pkg_resources is deprecated`
Run:
```bash
python3 - <<'PY'
import sys
print('python:', sys.executable)
try:
    import pkg_resources
    print('pkg_resources import: ok (deprecation warning is non-fatal)')
except Exception as e:
    print('pkg_resources import failed:', e)
PY
```

Notes:
- `Unbuilt egg for pytest-repeat` warnings are usually non-fatal for ROS 2 builds.
- `pkg_resources is deprecated` warnings from `vcs`/`rosdep` are informational unless accompanied by an actual traceback/exception.
