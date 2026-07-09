# MoveIt Servo (GreenForge Labs fork)

This is a fork of the [`moveit_servo`](https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo)
package from [MoveIt 2](https://github.com/moveit/moveit2), extracted as a standalone
repository so it can be consumed via `vcs`/`dependencies.repos` alongside binary MoveIt
packages.

**Base:** `moveit_servo` 2.12.4, imported from
[moveit/moveit2@1ade0e9](https://github.com/moveit/moveit2/commit/1ade0e9dcf50dbbbc3a984b995c786cf12736235)
(the `2.12.4` tag), path `moveit_ros/moveit_servo`.

## Fork delta

Kept intentionally minimal — one reviewable patch on top of the pristine import:

1. **`check_singularity` parameter** (default `true`, i.e. upstream behaviour).
   When `false`, the singularity condition-number check and its velocity scaling/halting
   are skipped entirely in `jointDeltaFromTwist` and `jointDeltaFromPose`.

   Rationale: `velocityScalingFactorForSingularity` indexes the Jacobian SVD with
   `dims = target_delta_x.size()` (always 6), but for a move group with fewer than
   6 joints the thin SVD of the 6×N Jacobian has only N singular values and a 6×N U
   matrix — `singularValues()(dims - 1)` and `matrixU().col(dims - 1)` read out of
   bounds (undefined behaviour in release builds; garbage condition numbers that can
   spuriously scale or halt motion regardless of threshold configuration). Setting the
   thresholds high does **not** avoid this, because the computation runs before the
   comparison. This has been reported upstream.

2. **Launch-based integration tests off by default** (`MOVEIT_SERVO_LAUNCH_TESTS=OFF`).
   The `launch_testing`-based tests error at collection in environments that disable
   pytest plugin autoloading or ship pytest ≥ 9. Re-enable with
   `-DMOVEIT_SERVO_LAUNCH_TESTS=ON`.

## Updating to a new upstream release

```bash
# 1. Fetch the new upstream tag and extract the package
curl -sL https://github.com/moveit/moveit2/archive/refs/tags/<TAG>.tar.gz | tar -xz
# 2. On a branch, replace the tree with moveit2-<TAG>/moveit_ros/moveit_servo
#    (keep this README), commit as "Import moveit_servo <TAG> from moveit/moveit2@<sha>"
# 3. Re-apply the fork delta (cherry-pick the patch commit; resolve if upstream moved)
# 4. If upstream has fixed the sub-6-DOF singularity indexing, drop patch 1 and plan
#    retirement of this fork.
```

## License

BSD-3-Clause, unchanged from upstream. See `package.xml` for authors and maintainers
of the original package.
