# Fork provenance

This is a fork of [`blackcoffeerobotics/vector_pursuit_controller`](https://github.com/blackcoffeerobotics/vector_pursuit_controller),
the nav2 controller plugin implementing the Vector Pursuit
path-tracking algorithm (Wit, 2006). Vector Pursuit is the geometric
extension of Pure Pursuit that takes the path heading at the lookahead
point into account, gated by a single shaping parameter `k`.

| Field           | Value                                                                  |
|-----------------|------------------------------------------------------------------------|
| Upstream remote | `https://github.com/blackcoffeerobotics/vector_pursuit_controller` (`upstream`) |
| Fork remote     | `git@github.com:jcfurey/vector_pursuit_controller.git` (`origin`)      |
| Tracked branch  | `jazzy` (upstream); `cam_devel` (fork)                                 |
| Last sync base  | `45e3dd6` — *2.0.0*                                                    |

The fork is **upstream-trackable** — every patch lives as a commit on
top of the upstream `jazzy` history with no rewrites or squashes, so a
`git rebase` against a refreshed upstream is the supported sync path.
Verify with:

```bash
git rev-list --count upstream/jazzy..HEAD   # commits ahead of upstream
git rebase upstream/jazzy                   # pick up new upstream work
```

## Local divergence — hardening pass 2026-04-27 (twelve commits)

A focused review-and-fix series after the rovermax workspace adopted
the upstream 2.0.0 release as a candidate ThetaStar replacement. The
review turned up two latent bugs that would fire in production
(infinite loop on near-zero commands; commanded-vs-measured velocity
mix-up in slew limits), several short-plan / divide-by-zero footguns,
and a pile of stale RPP-lineage cruft.

Ordering is intentional: chores first (no behavior change), then
cheap defensive guards, then the two semantic bugs, then exception
typing.

| Commit    | Category | Subject                                                              |
|-----------|----------|----------------------------------------------------------------------|
| `280fb2a` | chore    | gitignore coverage artifacts and remove committed `.info` file       |
| `09fa684` | chore    | fix stale RPP doc comments and typos                                 |
| `339b005` | chore    | drop dead `is_reversing_` member and duplicate param declaration     |
| `1ce6683` | chore    | dedupe `approach_velocity_scaling_dist` in sample params             |
| `cd7d356` | fix      | guard `getCuspDist` and `getLookAheadPoint` against short plans      |
| `86c660d` | fix      | guard divide-by-zero in approach scaling and curvature limit         |
| `dfce509` | fix      | avoid infinite loop in `isCollisionImminent` for near-zero commands  |
| `998295f` | fix(yaw) | use shortest-path interpolation between path headings                |
| `e5035a6` | fix      | track measured velocity in `last_cmd_vel_`, not commanded twist      |
| `5ae2fd2` | fix      | throw `nav2_core::ControllerException` instead of `PlannerException` |
| `e85a282` | fix      | downgrade FATAL log in `costAtPose` to throttled WARN                |
| `e1f6fdf` | docs     | explain why `desired_linear_vel` updates the base setpoint too       |

Each commit message contains the *why* in detail; consult `git log`
rather than restating it here.

## Workspace integration notes

- Wired up via `src/settings/params/navigation/nav2/behavior_trees/navigate_route_with_recovery.xml`
  (BT description) but not yet selected as the active controller in
  `src/settings/config/navigation/nav2/nav2.yaml` — the active stack
  uses Graceful (see `project_nav2_collision_investigation.md`).
- If/when you switch to it on the real platform, set
  `min_linear_velocity: 0.0` rather than the upstream sample's `0.05`
  — see review note (item #10): a non-zero floor short-circuits the
  approach-velocity scaling and prevents the robot from braking to a
  stop at the goal.
- The `last_cmd_vel_` measured-velocity fix (`e5035a6`) matters most
  on a saturated chassis. The Rover MAX platform commonly hits the
  velocity ceiling on the 1.0 m/s cap from
  `project_nav2_collision_investigation.md`; with the upstream
  commanded-velocity logic the slew limits would steadily drift
  upward, then snap when collision detection fires.

## Known follow-ups, not addressed

These were called out in the review but explicitly deferred — track
if you re-open the file:

1. **`circleSegmentIntersection` can still produce NaN** if a caller
   ever calls it with a fully-disjoint segment/circle (no real root).
   `getLookAheadPoint` is the only in-tree caller and only invokes it
   with one endpoint inside, one outside the circle, so the
   discriminant is non-negative by construction. Worth adding an
   explicit check + nearest-endpoint fallback if the helper ever
   becomes public-facing.
2. **`shouldRotateToGoalHeading` checks the lookahead point**, not
   the actual end of the global plan. Fine when the lookahead has
   clipped to the end of the path (the common case), but pathological
   short-path situations could trigger it spuriously. Better signal
   would be `transformed_plan.poses.size() == 1 && dist_to_back <
   goal_dist_tol_` or pulling state from the goal_checker.
3. **`global_plan_` is written by `setPlan` without taking `mutex_`**,
   while `transformGlobalPlan` reads it under the lock. Safe under
   nav2's default single-threaded executor; would need a fix if
   `controller_server` is ever run with `use_realtime_priority` or a
   multi-threaded executor.
4. **`min_linear_velocity` floor overrides approach scaling.** Item
   #10 in the review. Fixing it cleanly requires deciding whether
   the floor should bypass *all* deceleration or only the
   curvature/cost terms — a behavior change that needs operator
   sign-off, so deferred. Mitigated for now by the workspace
   integration note above (set it to 0).

## Sync procedure

```bash
# Inside src/packages/navigation/vector_pursuit_controller:
git fetch upstream
git log --oneline upstream/jazzy..HEAD          # what we have on top
git log --oneline HEAD..upstream/jazzy          # what's new upstream
git rebase upstream/jazzy                       # rebase the fork
# Resolve conflicts, then update the "Last sync base" line above.
git push origin cam_devel --force-with-lease    # publish to the fork
```

After syncing, update the **Last sync base** line in this file with
the new merge-base commit and a one-line label, and commit alongside
the rebase.
