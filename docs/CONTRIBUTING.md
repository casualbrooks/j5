# Contributing

Thanks for your interest in J5! Please follow these guidelines.

- Use feature branches and pull requests.
- Follow conventional commits: feat, fix, docs, chore, refactor, test.
- Keep changes small and focused.
- Add or update docs and tests when applicable.
- CI should pass before merge.

## Branch targets
- Use **main** for core J5 platform changes (ROS, hardware, base docs).
- Use **race-manager** for the race manager app/bridge/service/UI work so it stays isolated from the core platform.
- When opening a PR in GitHub, choose the right base branch up front. If you already opened one against `main`, you can edit the
  PR to retarget `race-manager` as long as there are no merge conflicts. To retarget: open the PR, click **Edit**, change the
  **base** dropdown to `race-manager`, and confirm the rebase warning. If conflicts appear, rebase locally onto `race-manager`
  before requesting review.
- The PR template includes a branch-targeting checklist—keep the `race-manager` checkbox ticked for race app changes so they do
  not land on `main` by accident.
