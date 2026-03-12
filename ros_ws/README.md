# J5 ROS 2 Workspace

This is a `colcon` overlay workspace containing J5 ROS 2 packages.

## Linux / Raspberry Pi quick start

### Race manager script from `ros_ws`

If you are already working from `~/j5/ros_ws`, run the existing repo-root script
with a relative path:

```bash
../scripts/run_racemanager.sh --mode ros2 --host 0.0.0.0 --pi-ip <pi-lan-ip>
```

For unattended headless startup on boot (systemd), print and run the generated
service setup snippet:

```bash
../scripts/run_racemanager.sh --mode ros2 --host 0.0.0.0 --pi-ip <pi-lan-ip> --print-systemd
```

If your Pi IP is dynamic or unavailable at setup time, omit `--pi-ip`; the generated
systemd command will omit that flag and let the script auto-detect the current IP each
service start.

If the service restarts/fails, inspect logs with:

```bash
sudo journalctl -u racemanager.service -n 200 --no-pager
```

Before enabling systemd, you can run a non-launch diagnostic check:

```bash
../scripts/run_racemanager.sh --mode ros2 --doctor
```

You only need to edit/regenerate the systemd unit when `ExecStart` arguments change.
For normal script updates, a restart is enough:

```bash
sudo systemctl restart racemanager.service
```

After updating `scripts/run_racemanager.sh`, reload and restart the service to
pick up the new script version:

```bash
sudo systemctl daemon-reload
sudo systemctl restart racemanager.service
sudo systemctl status racemanager.service --no-pager
```

If you still see `PRINT_SYSTEMD: unbound variable`, verify the script on the Pi
contains `PRINT_SYSTEMD="false"` near the top, then restart the service:

```bash
grep -n 'PRINT_SYSTEMD' ~/j5/scripts/run_racemanager.sh
sudo systemctl daemon-reload
sudo systemctl restart racemanager.service
sudo journalctl -u racemanager.service -n 50 --no-pager
```

If you see `apply_defaults: command not found` or `Unknown argument: --doctor`, your local
script is stale or conflict-corrupted. Restore from git and retry:

```bash
cd ~/j5
git fetch origin
git checkout -- scripts/run_racemanager.sh
grep -n "--doctor\|apply_defaults" scripts/run_racemanager.sh
```

1. Verify underlay launch support first (before moving to this overlay):

   ```bash
   source ~/ros2_kilted/install/setup.bash
   ros2 -h | rg -w launch
   ```

2. Build from this workspace root:

   ```bash
   cd ~/j5/ros_ws
   colcon build --symlink-install --allow-overriding fastdds
   ```

3. Source the overlay:

   ```bash
   source install/setup.bash
   ```

4. Launch bringup (voice placeholder node is disabled by default):

   ```bash
   ros2 launch j5_bringup bringup.launch.py
   ```

5. Optional: enable the placeholder voice node explicitly:

   ```bash
   ros2 launch j5_bringup bringup.launch.py start_voice:=true
   ```

## Troubleshooting

### `ros2 launch` is missing (only `run/topic/pkg` shown)

If `ros2 -h` does not list `launch`, you are in a partial ROS environment.
In a fresh shell, source underlay then overlay and verify `launch` is an actual CLI command:

```bash
source ~/ros2_kilted/install/setup.bash
source ~/j5/ros_ws/install/setup.bash
ros2 -h | rg -w launch || true
ros2 pkg list | rg '^j5_bringup$' || true
```

If the first command prints nothing, `launch` is not registered in your active ROS CLI.
That means your underlay is incomplete for launch workflows. Install or build launch CLI
packages in the underlay (`ros2launch` and `launch_ros`), then re-source underlay + overlay.
`j5_bringup` showing up in `ros2 pkg list` does not guarantee `ros2 launch` is installed.

For source-built underlays, ensure `ros2launch` and `launch_ros` are present in `src/` and successfully built.
Do **not** mix ROS distro package sets across workspaces.

If launch packages are installed but `ros2 -h` still does not show `launch`, you are likely invoking
the wrong `ros2` binary (or stale shell command cache). Verify and test directly:

```bash
hash -r
command -v ros2
ros2 extensions | rg 'launch|ros2launch' || true
ROS2_BIN="$(find ~/ros2_kilted/install -type f -path '*/bin/ros2' | head -n 1)"
echo "$ROS2_BIN"
"$ROS2_BIN" -h | rg -w launch || true
```

If `"$ROS2_BIN"` includes `launch` but `ros2` does not, your PATH/shell is pointing to a different
installation. Re-source underlay then overlay in a fresh shell and re-check `command -v ros2`.

If `"$ROS2_BIN"` also lacks `launch`, the source underlay itself is missing launch plugins.
Build launch packages in `~/ros2_kilted` and re-source underlay + overlay:

```bash
cd ~/ros2_kilted
colcon list | rg '^(ros2launch|launch_ros)$' || true
colcon build --symlink-install --packages-up-to ros2launch launch_ros
source install/setup.bash
source ~/j5/ros_ws/install/setup.bash
ros2 -h | rg -w launch
```

### `demo_nodes_py` not found

`demo_nodes_py` is a ROS demo package and may not exist in your source-built underlay.
Use package discovery checks first instead of running nodes:

```bash
ros2 pkg list | rg '^j5_'
ros2 pkg executables j5_voice
```

If `j5_voice` executables exist but crash at runtime (for example allocator/corruption errors),
track that as a package/runtime bug; it does not invalidate launch-extension diagnostics.

### Other common build/runtime issues

- If `ros2` is not found, your underlay is not sourced in the current shell.
- If `fastcdr package NOT found` appears while building `fastdds`, source the ROS 2
  underlay first so dependencies are discoverable.
- If colcon warns that `fastdds` exists in the underlay, use
  `--allow-overriding fastdds` intentionally (as shown above) and keep runtime RMW/distro versions consistent.
- Linker warnings like `libfastdds.so.3.2 ... may conflict with libfastdds.so.3.5` indicate mixed Fast DDS
  libraries between underlay and overlay; keep underlay/overlay builds aligned and treat runtime crashes as ABI mismatch symptoms.
