# Off-Pi race video review

The Pi remains the source of truth for the recording, race markers, racer roster, lap ledger, and championship. A stronger computer fetches a saved race video, analyzes sequences around directed finish-line crossings, and sends a **proposal** back. A proposal never changes laps or standings by itself. The browser compares recorded and proposed totals, and **Apply complete proposal** atomically backs up and replaces laps. Finished races are rescored in the same database transaction.

The exchange uses race-scoped endpoints:

| Endpoint | Purpose |
| --- | --- |
| `POST /api/quick/review/{race_id}/request` | Queue a paused or finished replay for compute review |
| `GET /api/quick/review/jobs` | Let workers discover requested races |
| `GET /api/quick/review/{race_id}/manifest` | Return video hash, markers, gate, roster, lap target, and ledger hash |
| `GET /api/quick/review/{race_id}/video` | Stream the original MP4 for offline analysis |
| `POST /api/quick/review/{race_id}/proposal` | Submit attributed crossings with confidence and surrounding-frame timestamps |
| `POST /api/quick/review/{race_id}/apply` | Apply a complete proposal after viewing its summary |

The server rejects proposals for a different video, unknown racer, out-of-range timestamp, implausibly short lap, or too many laps. A proposal stays incomplete if any crossing is unassigned, any proposed identity is below 0.9 confidence, or any entered racer has fewer than two crossings. The first crossing starts that car's timing; each later crossing completes one lap. The server saves the original laps and results under `review_backup:{race_id}:{proposal_id}` before replacement. A changed ledger invalidates an older proposal.

On Windows, use `scripts/run_nemoclaw_review.ps1 -RaceId <race-id> -Limit 20` from PowerShell. It downloads the video into `%USERPROFILE%\j5-reviews`, detects candidate crossings, builds seven-frame temporal strips, uploads those strips to the existing NemoClaw sandbox, runs the NVIDIA vision model, downloads its resumable classifications, and submits the proposal to the Pi. Run the same command again to review the next batch; labels already returned are kept on the host. The NVIDIA key remains in the sandbox. The Pi never receives it. For another compute provider, `scripts/offboard_review.py` accepts `NVIDIA_API_KEY` locally and can run with `--watch --submit` to process queued races.

The model is a visual assistant, not proof of identity. On the current 640x360 GoPro replay, a seven-frame test showed a moving car, but the model reversed its direction and could not name the racer. In the first automated batch it missed a visible car altogether. The worker therefore checks direction through computer vision, checks plausible lap spacing, and leaves unclear identities unassigned. An incomplete proposal cannot overwrite the ledger. Better footage, multiple views, or validated per-car reference imagery may be needed before this recording can produce four dependable lap totals. [NVIDIA's model page](https://build.nvidia.com/nvidia/nemotron-3.5-super-vl-120b-a12b) documents the hosted vision endpoint used by the optional worker.
