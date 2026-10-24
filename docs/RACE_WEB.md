# Race Manager on Raspberry Pi

The focused browser workflow is served by the existing Race Master Pro backend
at `/race`. It uses the existing seasons, championships, events, racers, laps and
results tables. No Node server or separate preview process is needed for this
workflow. The larger React dashboard remains available for development.

## Install and run

On Ubuntu 24.04 with Python 3.12, `python3-venv`, and a USB camera:

```bash
bash scripts/run_race_web.sh --install
RACE_HOST=0.0.0.0 bash scripts/run_race_web.sh
```

Open `http://<pi-address>:8080/race` from another device on the trusted LAN or
Tailscale network. This local club application does not implement user accounts;
do not expose its control API directly to the public Internet.

## Camera-only crossing test

The **Camera-only crossing test** works before preparing a race. Connect the
camera, set a vertical line over one straight and limit its top and bottom to
that track segment, then click **Apply test line and reset candidates**. The
orange boxes on the camera preview are moving objects; each directed crossing
adds an anonymous candidate with a frame link. Download the CSV before stopping
the service if you want to retain the observations. Only the latest 500
candidate frames and rows are kept in memory. This test does not insert laps
or championship scores and cannot identify visually similar cars.

For the existing whole-track GoPro recording on the Pi, enter
`video:/home/racetime/videotest.mp4` as the source and click **Use full-track
video line** immediately after connecting. The preset watches the upper
straight around x=61%, y=10-20%, moving right to left. The video plays at its
recorded rate; reconnect it to restart. This preset is specific to that
640x360 recording. It is a diagnostic starting point, not a verified four-car
lap counter. The current 30-fps full-track footage can miss dark cars and
brief occlusions. For live acceptance, compare candidate frames against a
manually checked practice race before relying on the count.

1. Connect `/dev/video0`. If a ROS camera publisher already owns that device,
   connect `ros:/camera/cam1/image_raw` instead. The launcher selects Kilted;
   `ROS_UNDERLAY=/path/to/setup.bash` selects a specific source installation.
   ROS input supports `sensor_msgs/Image` in `rgb8` or `bgr8` encoding.
2. Enter the championship, round, lap target and racer names. Reuse racer names
   across rounds to retain their identities. Choose distinct saturated colors,
   or sample a car's body color in the preview after selecting its swatch.
   Black, white, grey and similarly colored cars require a different detector.
3. Position the vertical finish line over a straight section of track. Select its
   vertical extent and forward direction. Keep colored scenery out of view.
   Multiple blobs of the same car color are ambiguous and are ignored.
4. Save and prepare. Verify that the preview identifies each car correctly.
5. Start. Each car's first forward crossing arms its timer; the next completes
   lap 1. The detector requires an observed approach from at least 6% of frame
   width before the line, rejects stale frames, and enforces the minimum lap time.
6. Finish to commit results and points. Ranking is laps descending, then total
   completed-lap time ascending. Cars below the target are DNF with zero points.
   Only completed main races count toward standings. Finishing twice is safe.

Timing and persistence run on the server even when all browser tabs close.
Pausing discards only the in-progress lap; resuming re-arms at the next crossing.
A service restart pauses active races and retains all committed laps. Camera
failure pauses counting; reconnect and resume explicitly. This initial color
detector does not enforce intermediate checkpoints or resolve occluded/similar
cars. Validate a practice race against manual lap counts at actual racing speed.

## Durable data and service

The launcher keeps data at `~/.local/share/j5-racemanager/racetracker.db`, outside
the checkout. `RACE_DATA_DIR`, `RACE_VENV`, and `DATABASE_PATH` override paths.
Do not run multiple backend workers against the camera.

For an existing database, use the SQLite backup API, not a bare copy while the
source is live:

```bash
python3 scripts/backup_race_db.py /path/to/existing/racetracker.db /path/to/backup.db
```

Back up before upgrading. For a restore, stop the service, validate a backup's
integrity, then point `DATABASE_PATH` at that restored copy. Keep the original
database and its WAL together until the restore has been verified.

For a user service, point `~/j5-racemanager-current` at the chosen release, copy
`deploy/race-manager.service` to `~/.config/systemd/user/`, then run:

```bash
systemctl --user daemon-reload
systemctl --user enable --now race-manager
systemctl --user status race-manager
```

Enable lingering (`sudo loginctl enable-linger "$USER"`) if it should run after
logout and at boot. Camera access requires membership in the `video` group.
Rollback by stopping the service, repointing the release symlink, and starting
it. Application data stays separate from all release directories.

## Verification

```bash
cd ros_ws/src/j5_perception/race-master-pro/backend
python -m pytest test -q
```

The tests cover distinct color identity, ambiguous detections, directed crossings,
occlusion, duplicate/concurrent lap delivery, invalid laps, DNF scoring,
multi-round standings, and database reopen. Hardware acceptance additionally
requires observing each car under the actual camera placement and lighting.
