# Data flow

## Inputs

Each cycle the host builds one immutable `InputSnapshot` ([FROM: include/core/core.hpp]).

| Field | Frame / unit | Meaning |
|---|---|---|
| `pose` | `map`, m, m, rad | Localized pose (`Pose2D`), already in the map frame. |
| `goal` | `map`, m, m | Goal position (`Point2D`). |
| `lidar` | `lidar`, m, rad | Non-owning view of beam returns (`LidarPoint`: `r`, `alpha`). |
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
| `dt` | Cycle period, copied from the snapshot. |
| `timestamp` | Cycle time, copied from the snapshot. |
| `frame_id` | Always `Frame::Map`. |
| `status` | One of the codes below. |

When `status` is not `Valid`, `next_pose` is `NaN` in every field.

## Status codes

| Code | Set when |
|---|---|
| `Valid` | Every stage succeeded. `next_pose` is usable. |
| `NoPath` | The global guide or the local plan has no collision-free route. The TEB stage returns this when any trajectory point is within `ROBOT_RADIUS_M` of a blocked cell. |
| `StaleSensor` | A sensor sample is older than `SENSOR_STALE_TIMEOUT_S`. |
| `PlannerTimeout` | The global planner reports a timeout. The status is defined and propagated; the core has no wall-clock timeout. |
| `Fault` | Core not initialized, sensor time in the future or non-finite, `teb_config.step_dt` not equal to `snapshot.dt`, a placeholder not replaced, or any invalid stage input. |

Stage order determines which code wins. Freshness is checked first, so a stale or future sensor is reported before any stage runs. `Fault` takes priority over `StaleSensor` when both occur. [FROM: src/core.cpp]

`Status` has no silent default. `Output::status` defaults to `Fault`, so an unset output is never read as valid. [FROM: include/core/core.hpp]

## Sensor timestamp staleness

`sensor_freshness()` runs before any stage. For each of the five sensor timestamps:

1. Compute `age = timestamp - sample_time`.
2. If `age` is not finite or `age < 0`, return `Fault`. A negative age means the sample is from the future, and a non-finite age means the data is corrupt.
3. If `age > SENSOR_STALE_TIMEOUT_S`, mark the snapshot stale.

After all five are checked, return `StaleSensor` if any was stale, otherwise `Valid`. The threshold `SENSOR_STALE_TIMEOUT_S` is a placeholder (`PH-0005`). The guard rejects zero, so no sensor is treated as always stale.
[FROM: src/core.cpp, UNKNOWNS.md PH-0005]

> **Assumption:** The host supplies all five sample times in the same epoch as `timestamp`. [ASSUMED: AGENTS.md does not state the epoch; the header says "caller-defined epoch".]
