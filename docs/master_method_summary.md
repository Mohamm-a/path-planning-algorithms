# Master method summary

A high-level view of the core. Implementation details are in the companion method documents.

## 1. System architecture

The core is a deterministic five-stage pipeline. Each control cycle receives one immutable snapshot and produces one command.

| Stage | Role |
|---|---|
| 1. Odometry | Fuses encoder speed and gyro yaw rate into a predicted motion. |
| 2. Map manager | On a new LiDAR sweep, marks free and occupied cells, rebuilds the inflation layer, and composes the costmap. A repeated sweep reuses the existing map. |
| 3. Global planner | A* over the composed grid when the goal, map, or replan period changes. It produces a waypoint guide, not a trajectory. |
| 4. TEB local planner | Optimizes speed and steering over a short horizon under steering, speed, acceleration, and clearance limits. |
| 5. Output | Converts the first plan step into the next pose, speed and steering commands, and a status. |

The order is fixed. A stage that is not `Valid` stops the cycle and reports its status; no stage result is replaced by a fallback.

## 2. Simplified pseudocode

```text
initialize the core
if the configuration is invalid:
    stop with Fault

for each control cycle:
    read the pose, goal, LiDAR, encoder, steering, gyro, and timestamps

    check sensor freshness
    if any required sample is too old:
        return StaleSensor

    motion = odometry(snapshot)

    if the snapshot contains a new LiDAR sweep:
        update the map, inflation, and costmap
    else:
        reuse the existing map

    if the goal, map, or replan period changed:
        guide = A_star(start, goal, costmap)
    else:
        reuse the existing guide

    plan = TEB(motion, guide, costmap, limits)
    if no collision-free local plan exists:
        return NoPath

    return next pose, speed command, steering command, Valid
```

The snapshot timestamp is the only clock. The pipeline does not read wall time or hidden state.

## 3. Placeholder and guard system

The 17 physical values are compile-time `#define`s in `placeholders.hpp`: wheelbase and steering limits, speed and acceleration limits, wheel radius, LiDAR geometry and range, map resolution and inflation, robot radius, stale timeout, and planner periods. They are approved and locked in; every row is `RESOLVED` in `UNKNOWNS.md`.

The `initialize()` guard validates the values before operation. Zero and negative values are rejected, non-finite mount values are rejected, and each quantity must otherwise lie in its permitted range. A failure returns `Fault`, and `step()` refuses to run until initialization succeeds. This keeps an unsafe configuration from reaching the planners.

## 4. Coordinate frame

`base_link` is the centre of the rear axle on the ground plane. +x is forward, +y is left, and heading is positive counter-clockwise. The host supplies the rear-axle pose in the `map` frame; sensor mount offsets are measured from that origin.
