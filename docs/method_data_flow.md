# Data flow

## Scalar types

The core uses one alias, `real = float`, for every spatial and kinematic quantity: coordinates, angles, speeds, map resolution and planner weights. Timestamps and durations (`timestamp`, `dt`, the per-sensor sample times, `TebConfig::step_dt`) stay `double`. A binary32 timestamp cannot resolve small intervals at Unix-epoch magnitude, and the stale check subtracts two timestamps, so time must not narrow.
[FROM: `include/core/core.hpp`, user approval Q1]

The conversion is done once at each boundary use: for example odometry takes `snapshot.dt` as `double` and converts it to `real` for the integration step.

## Frames and units

[FROM: AGENTS.md section 5; README "Coordinate frame"]

- The API uses SI units only: m, rad, s, rad/s. `encoder_rpm` is the single documented exception.
- The `base_link` origin is the centre of the rear axle on the ground plane. +x is forward, +y is left.
- `theta` is CCW-positive and normalized to (-pi, pi] at the boundary.
- LiDAR: `r` is the beam length in m; `alpha` is the beam angle in rad, CCW from the lidar's +x axis. Every beam is cast from `snapshot.pose`, so a sweep captured while the robot moves must be motion-compensated by the host.
- `LIDAR_MOUNT_X_M`, `LIDAR_MOUNT_Y_M` and `LIDAR_MOUNT_THETA_RAD` are measured from the `base_link` origin.
- All five sensor timestamps and `timestamp` share one epoch, in seconds.

## Inputs

Each cycle the host builds one immutable `InputSnapshot` ([FROM: include/core/core.hpp]).

| Field | Frame / unit | Meaning |
|---|---|---|
| `pose` | `map`, m, m, rad | Localized pose (`Pose2D`), already in the map frame. |
| `goal` | `map`, m, m | Goal position (`Point2D`). NaN in either component selects no-goal mode. |
| `lidar` | `lidar`, m, rad | Non-owning view of beam returns (`LidarPoint`: `r`, `alpha`). |
| `is_new_sweep` | bool | True when `lidar` holds a sweep captured since the previous cycle. |
| `encoder_rpm` | RPM | Drive-wheel rotation rate. Documented exception to the rad/s rule. |
| `steering_angle` | rad | Steering angle; positive is a left turn. |
| `gyro_yaw_rate` | rad/s | Yaw rate; positive is CCW. |
| `global_planner` | enum | Validated planner in effect: `AStar`, `DStarLite`, or `RRT`. Only `AStar` has an implementation. |
| `timestamp` | s | Current cycle time, in the host's epoch. |
| `dt` | s | Cycle period, supplied by the host. |
| `pose_timestamp`, `lidar_timestamp`, `encoder_timestamp`, `gyro_timestamp`, `steering_timestamp` | s | Sample time of each sensor, in the same epoch as `timestamp`. |

The `LidarView` is a pointer and a count. The caller owns the storage, and the core does not allocate for it.

## Output

`step()` returns an `Output` ([FROM: include/core/core.hpp]):

| Field | Meaning |
|---|---|
| `next_pose` | Next pose (`Pose2D`) in `map`. `theta` is normalized to (-π, π]. |
| `v_cmd` | Speed for the first horizon step, in m/s. |
| `delta_cmd` | Steering for the first horizon step, in rad; positive is left. |
| `dt` | Cycle period, copied from the snapshot. |
| `timestamp` | Cycle time, copied from the snapshot. |
| `frame_id` | Always `Frame::Map`. |
| `status` | One of the codes below. |

The commands come from the local plan, not from a conversion of `next_pose`. They describe the same first step, so a host may use either. When `status` is not `Valid`, `next_pose`, `v_cmd` and `delta_cmd` are all `NaN`.
[FROM: user approval Q5, `src/output_converter.cpp`]

## Status codes

| Code | Set when |
|---|---|
| `Valid` | Every stage succeeded. `next_pose` and the commands are usable. |
| `NoPath` | The global guide or the local plan has no collision-free route. |
| `StaleSensor` | A checked sensor sample is older than `SENSOR_STALE_TIMEOUT_S`. |
| `PlannerTimeout` | The global planner exhausts `GlobalPlannerConfig::max_expansions`. |
| `Fault` | Core not initialized, a non-finite pose or sensor time in the future, `teb_config.step_dt` not equal to `snapshot.dt`, a placeholder not replaced, no sweep ever mapped, or any invalid stage input. |

Stage order determines which code wins. Freshness is checked first, so a stale or future sensor is reported before any stage runs. `Fault` takes priority over `StaleSensor` when both occur. [FROM: src/core.cpp]

`Status` has no silent default. `Output::status` defaults to `Fault`, so an unset output is never read as valid. [FROM: include/core/core.hpp]

## Sensor timestamp staleness

`sensor_freshness()` runs before any stage. For the pose, encoder, gyro and steering timestamps:

1. Compute `age = timestamp - sample_time`.
2. If `age` is not finite or `age < 0`, return `Fault`. A negative age means the sample is from the future.
3. If `age > SENSOR_STALE_TIMEOUT_S`, mark the snapshot stale.

The LiDAR timestamp is checked only when `is_new_sweep` is true. A sweep takes much longer than a control cycle, so the host repeats the latest complete sweep and its original timestamp. Treating that repeat as stale would stop the core between sweeps, so the repeated sweep is not re-age-checked and the map manager is not asked to mark the same beams again.
[FROM: user approval Q3, `src/core.cpp`]

After the checks, return `StaleSensor` if any checked sensor was stale, otherwise `Valid`. The threshold `SENSOR_STALE_TIMEOUT_S` is a placeholder (`PH-0005`).

> **Note:** With `is_new_sweep == false`, `step()` reuses the map built by the last sweep. If no sweep has been mapped yet, `step()` returns `Fault` rather than planning on caller-initialized cells.

## No-goal mode

A `goal` with a NaN component selects no-goal mode. `step()` skips the global planner and the plan cache, writes a straight-line guide ahead of the robot, and passes it to the local planner. The guide covers the farthest distance one TEB horizon can travel at `MAX_SPEED_MPS`, so the guide cost does not pull the robot back at the end of the horizon. The status remains `Valid` while all other checks pass.
[FROM: user approval Q4, `src/core.cpp`]

> **Assumption:** The host sets both goal components to NaN together. A goal with one NaN component is treated as no-goal, and a finite but unreachable goal still returns `NoPath` or `Fault` from the planner.
