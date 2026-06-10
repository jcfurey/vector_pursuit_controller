# Jazzy backport of the fork's controller fixes

This branch (`claude/jazzy-backport-rd2g2p`) is **upstream `jazzy` (2.0.0,
commit `45e3dd6`)** plus a backport of the fork's controller fixes that were
originally developed on the Lyrical-targeted branch
(`claude/pull-latest-fork-rd2g2p`, on top of the nav2 `main`/`nav2_ros_common`
API). The Lyrical branch does not build on Jazzy (no `nav2_ros_common`, no
`nav2_controller::feasible_path_handler`, pre-Kilted `ament_auto_package`), so
the fixes are re-applied here against the Jazzy nav2 API
(`rclcpp_lifecycle::LifecycleNode`, `nav2_util::declare_parameter_if_not_declared`).

## What was backported

- **`calcTurningRadius` screw geometry.** `phi` is the pure-pursuit
  translation-screw rotation — the tangent-arc sweep `2*atan2(y, x)` — so the
  blend reduces to pure pursuit when the target heading matches the arc, for
  any `k`. The denominator uses `k*phi + shortest_angular_distance(phi,
  target_angle)` (wrap-safe), and the zero-total-rotation pole degrades to
  straight-line motion. (Fixes a formula that was 15-45% off and inverted the
  heading term near straight.)
- **Angular-acceleration limit on the curvature path.** Factored into
  `applyAngularAccelerationLimit()` (shared with `rotateToHeading`) and applied
  to `angular_vel = linear_vel / turning_radius`, which was previously
  unbounded — relevant for zero-turn differential drive, which can execute the
  resulting angular jerk.
- **`min_linear_velocity` floor before approach scaling**, so the goal-approach
  deceleration is not overridden by the floor.
- **Atomic rejection of invalid `inflation_cost_scaling_factor`** in the dynamic
  parameter callback (`result.successful = false` + reason) instead of warning
  and silently keeping the old value.
- **Subscription guards** on the three debug publishers (received plan,
  lookahead point, collision arc), which published every control cycle.

## Not backported (deferred)

- The path-integrated `getLookAheadPoint` rewrite (and the `nav2_util`
  `circleSegmentIntersection`/`linearInterpolation` reuse). Jazzy's `nav2_util`
  lacks those helpers and the Jazzy lineage also predates the fork's short-plan
  / shortest-angle-interpolation fixes, so a faithful port is larger; the
  Euclidean selection is left as upstream.

## Verification

Built and tested in the official `ros:jazzy-ros-base` image against Jazzy nav2
binaries. All gtest cases pass (including the updated `calcTurnRadius` and the
new `angularAccelerationLimit` test). The only remaining lint failures
(cpplint/xmllint on files not touched here) are pre-existing in upstream
`jazzy` — identical to a clean `origin/jazzy` baseline run.
