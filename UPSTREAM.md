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

## Local divergence — Lyrical audit pass 2026-06-09

A multi-angle review after the Lyrical migration (`157a4fb`), with
fixes verified against a from-source nav2 `main` build inside the
Lyrical container (the `docker/` infrastructure added on this branch).

| Commit    | Category | Subject                                                              |
|-----------|----------|----------------------------------------------------------------------|
| `80888f7` | infra    | reproducible ROS 2 Lyrical Docker build (nav2 from source)           |
| `9e68fcf` | infra    | scope rosdep to closure; apply toolchain patches after deps          |
| `37980a2` | infra    | pin nav2 commit; build-only proxy CAs; layer split (~36s iteration)  |
| `4cb3a8e` | fix      | path-integrated lookahead via nav2_util; min-velocity floor before approach scaling; turning-radius pole guard; atomic param rejection; publisher gating |
| `(docs)`  | docs     | README defaults aligned with code; follow-ups below updated          |

`4cb3a8e` resolves known follow-ups #1 and #4 below.

## Upstream comparison & the calcTurningRadius geometry fix

Upstream (`blackcoffeerobotics`) has not moved since the last sync:
their `jazzy` (2.0.0) still equals our sync base `45e3dd6` (a clean
ancestor of this branch), and their default branch `master` is the
diverged 1.0.x line. That line carries one substantive fix the 2.0.0
lineage never received: `5886046` *"Fixed angle calculations and out
of bounds errors"*, touching `calcTurningRadius`.

Investigating it showed **both** formulations are geometrically wrong.
`phi` must be the rotation angle of the pure-pursuit translation screw
— the arc swept along the circle tangent to the robot heading through
the target — because the screw blend must reduce to pure pursuit
(`radius = d²/2y`) whenever the target heading equals the arc's natural
end heading, for any `k`. By the tangent-chord (inscribed angle)
identity that sweep is simply `2 * atan2(y, x)`. Numerically:

| case (x, y, θ_t = sweep)  | true R | 2.0.0 formula | master `5886046` | chord fix |
|---------------------------|--------|---------------|------------------|-----------|
| (2, 2, π/2)               | 2.000  | 2.867         | 2.182            | 2.000     |
| (2, −2, −π/2)             | 2.000  | 2.867         | 2.400 (asymmetric!) | 2.000  |
| (1, 1, π/2)               | 1.000  | 1.600         | 1.091            | 1.000     |

Worse, in the near-straight regime (the dominant operating mode) the
2.0.0 formula produced `phi ≈ −π` where geometry gives `phi ≈ 0`, so
the target-heading term entered the screw blend with an effectively
inverted sign — the heading-awareness that distinguishes Vector
Pursuit from Pure Pursuit was working backwards precisely where the
controller spends most of its time. The existing unit tests could not
catch any of this: with an identity carrot orientation `term_1`
collapses to `k/(k−1)` for *any* nonzero `phi`.

The fix also replaces the raw `(k−1)·phi + θ_t` denominator with
`k·phi + shortest_angular_distance(phi, θ_t)` — algebraically equal
where `|θ_t − phi| ≤ π` but wrap-safe at ±π (a reversed target heading
no longer gives two different radii depending on the sign of the ±π
representation). The zero-total-rotation pole (pure-translation screw)
degrades to straight-line motion, preserving the `4cb3a8e` guard.
New unit tests pin pure-pursuit consistency, mirror symmetry, the
near-straight heading response, and the pole.

A follow-up audit (numeric verification in `/tmp` during review;
results summarised here) confirmed the `phi` fix three ways: it equals
the actual pure-pursuit arc heading change to machine precision; it is
the unique value making the blend self-consistent (`term1 == 1` iff
`θ_t == phi`, for every `k`); and the old second term reduced to
`atan(R_pp)` — the arctangent of a length, i.e. a mis-derivation. The
blend's screw-theory structure also checks out (correct `k → ∞` pure
pursuit limit; curvature decomposes as `(k-1)/k·κ_pp +
heading-correction/k`). The authoritative thesis (DTIC ADA468928) is
403-blocked, so this verifies internal consistency and limits rather
than byte-matching eq. 3.52.

That audit also found a separate latent bug: `calcTurningRadius`
returned `std::abs(...)` and the caller re-derived the turn direction
from `sign(y)` alone, discarding `sign(term_1)`. When the heading
correction dominates the arc (`term_1 < 0` — a near-straight carrot
with a strongly opposing target heading), the screw center flips to
the far side but the robot kept turning toward the carrot, commanding
the wrong angular direction. Reachable mainly with
`use_heading_from_path: true`; computed headings keep `term_1 > 0`.
Fixed by returning a **signed** turning radius (sign = turn direction)
and dividing by it directly. When the signed radius lands on the far
side from the target (`radius * y < 0`), arcing away is degenerate, so
the controller rotates toward the target heading instead. Unit test
`calcTurnRadius` now pins the signed convention and the heading-
dominated (far-side) case.

The 1.0.x line's other unique commit (`4e5c79f`, Ackermann constraint
test) targets the pre-2.0 test fixture API and was not ported; the
min-turning-radius clamp it exercises is covered by the existing
`calcTurnRadius` "directly behind" case.

## Zero-turn differential-drive target

The intended deployment is a zero-turn differential-drive base
(`min_turning_radius: 0.0`, which auto-enables `use_rotate_to_heading`).
That platform exposed an asymmetry the Ackermann-flavoured defaults
hid: the rotate-to-heading paths acceleration-limit `angular_vel`
against `max_angular_accel`, but the main curvature-tracking path
(`angular_vel = linear_vel / turning_radius`) did not. With a
near-zero `min_turning_radius` the radius clamp bounds nothing, so a
sharp lookahead produced a large, cycle-to-cycle-discontinuous angular
command — harmless on a car-like base that can't execute it, but real
jerk/slip on a base that can.

`applyAngularAccelerationLimit()` now factors the clamp (shared by
`rotateToHeading`) and is applied on the curvature path too.
`config/diffdrive_nav2_params.yaml` is a diff-drive starting config;
README "Platform notes" documents the choice.

## ROS 2 distro support

Verified 2026-06-09 in the official `ros:<distro>-ros-base` images
(`docker/check_distros.sh` re-runs the humble/jazzy/kilted checks):

| Distro  | Status | How to use                                                        |
|---------|--------|-------------------------------------------------------------------|
| humble  | ✅ binaries | `apt install ros-humble-vector-pursuit-controller` (1.0.2, upstream `master`) |
| iron    | ❌ EOL  | never released; iron reached end-of-life 2024-11                  |
| jazzy   | ✅ binaries | `apt install ros-jazzy-vector-pursuit-controller` (2.0.0, `jazzy` branch) |
| kilted  | ✅ source | no release, but the `jazzy` branch (2.0.0) builds cleanly against kilted's nav2 binaries |
| lyrical | ✅ source | no nav2 binaries exist for lyrical at all — this branch (`cam_devel` lineage) + `./docker/build.sh` builds nav2 `main` from source |

Caveats:

- The pre-lyrical rows ship the **upstream** controller. The fork's
  hardening-pass and audit-pass fixes live on `cam_devel`/this branch
  on top of the lyrical migration; backporting them to a
  jazzy/kilted-compatible branch is straightforward (the fixes
  predate-API-agnostic except for the lookahead `nav2_util` reuse,
  which kilted/jazzy's nav2_util lacks — keep the path-integrated
  selection loop local there).
- Kilted support relies on source-building an unreleased branch; if
  the `jazzy` branch moves, re-run `docker/check_distros.sh`.

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

1. ~~`circleSegmentIntersection` can still produce NaN~~ — resolved
   in `4cb3a8e`: the local helper was removed outright;
   `getLookAheadPoint` now selects by path-integrated distance and
   interpolates with `nav2_util::linearInterpolation` (guarding
   degenerate duplicate-pose segments), matching nav2 main semantics.
2. **`shouldRotateToGoalHeading` checks the lookahead point**, not
   the actual end of the global plan. Fine when the lookahead has
   clipped to the end of the path (the common case), but pathological
   short-path situations could trigger it spuriously. Better signal
   would be `transformed_plan.poses.size() == 1 && dist_to_back <
   goal_dist_tol_` or pulling state from the goal_checker.
3. ~~`global_plan_` is written by `setPlan` without taking `mutex_`~~ —
   moot since the Lyrical migration (`157a4fb`): `global_plan_` was
   removed; plan handling lives in the controller server's path
   handler and `newPathReceived()` is a no-op.
4. ~~`min_linear_velocity` floor overrides approach scaling~~ —
   resolved in `4cb3a8e`: the floor is applied before approach
   scaling, so it bounds the curvature/cost/accel terms but the
   goal-approach deceleration can go below it (bounded by
   `min_approach_linear_velocity`). The workspace mitigation
   (`min_linear_velocity: 0.0`) is no longer required, though still
   harmless.

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
