
# Safety & Public Demo Checklist

- Dual e-stop: one wired, one wireless; test before every run.
- Bumper switches: around track guards; verify debounce handling.
- Force/velocity limits: exposed as ROS params; unit tests enforce caps.
- Consent: ask before recording/touch; enable face blurring by default.
- Environment: set perimeter, spotters, and clear signage for public demos.
- QuirkPolicy: never active in motion/force control loops.
- Play-dead: limp/neutral mode on E-stop or major fault.
