# Kinematics

## Ackermann bicycle model

The robot is modeled as a kinematic bicycle: the two front wheels are merged into one front wheel, and the two rear wheels into one rear wheel. Steering acts on the front wheel only.
[ASSUMED: standard Ackermann kinematics; AGENTS.md does not give a dynamics or slip model.]

State in the map frame: position `(x, y)` and heading `θ`.

```text
x'     = v * cos(θ)
y'     = v * sin(θ)
θ'     = v * tan(δ) / WHEELBASE_M
```

- `v` is the forward speed in m/s.
- `δ` is the steering angle in rad. Positive is a left turn.
- `WHEELBASE_M` is the distance between the front and rear axles, in m. It is placeholder `PH-0001`.
[FROM: src/teb_local_planner.cpp, AGENTS.md §5]

## Yaw rate relation

From the model above, the yaw rate is:

```text
yaw_rate = (speed * tan(steering)) / WHEELBASE_M
```

Two limits apply:

- The steering magnitude must be below a right angle, because `tan(δ)` is unbounded at `±π/2`. The guard requires `0 < MAX_STEER_RAD < π/2`. `MAX_STEER_RAD` is `PH-0002`.
- At `v = 0`, the model predicts no yaw change, so a stationary robot cannot rotate in place. [FROM: the model equation, not a measured property.]

The turn radius that follows from this relation is `WHEELBASE_M / tan(δ)`. The minimum turn radius is therefore `WHEELBASE_M / tan(MAX_STEER_RAD)`. It is derived, not stored, so the two values cannot disagree. The old placeholder `PH-0003` was dropped for this reason. [FROM: UNKNOWNS.md PH-0003]

## Origin at the rear axle

The physical origin of `base_link` is the centre of the rear axle on the ground plane. [FROM: AGENTS.md §5]

This is required by the model, not a convention. The bicycle equations describe the motion of the rear axle centre: the heading rate depends on `tan(δ)/L`, where `L` is the distance from that centre to the front axle. If the origin were at another point, the `x'` and `y'` equations would describe that point's motion, and the `tan(δ)/L` term would no longer give the correct yaw rate.

The core uses the pose as given, in the map frame. The pose is treated as the rear-axle centre. The host must supply it that way. The core cannot check this.
[ASSUMED: the host pose refers to the rear-axle centre; the core has no way to verify it.]

Sensor mount offsets (for example `LIDAR_MOUNT_X_M`) are measured from this origin. The LiDAR beams are placed with those offsets before ray marking. [FROM: src/map_manager.cpp]

## How the local planner avoids impossible turns

The TEB stage never accepts a steering command outside the physical limits. It enforces this in three places:

1. **Steering box.** Every steering value is clamped to `[-MAX_STEER_RAD, +MAX_STEER_RAD]` in the projection step.
2. **Steering rate.** Each steering value is clamped to a window around the previous value, of half-width `MAX_STEER_RATE_RADPS * dt`. The robot cannot turn its wheel faster than the servo allows.
3. **Rollout.** The trajectory is simulated with the same bicycle equation as above. A plan that would cross an obstacle fails the final clearance check, and the cycle returns `NoPath`.

Because every candidate is projected onto these limits before its cost is computed, the optimizer only searches over feasible steering histories. [FROM: src/teb_local_planner.cpp, `project()`]

The speed is limited in the same way: `|v| ≤ MAX_SPEED_MPS`, and the speed change per step is limited by `MAX_ACCEL_MPS2 * dt`.

> **Assumption:** The model has no slip, tire force, or actuator delay. Real motion can differ from the model at high speed or on low-friction surfaces. [ASSUMED: AGENTS.md states no dynamics.]
