# UNKNOWNS.md

Authoritative registry of every unknown variable (placeholder) in the codebase.
Defined by AGENTS.md section 6. Rows are append-only; never delete a row.
A placeholder without a row, or a row without a placeholder, is a defect.

## Row format

```text
| ID | Placeholder | Type / Unit | Sentinel | Affects | Guard | Added by | Status | Resolution |
```

| Field | Meaning |
|---|---|
| `ID` | Stable short ID, e.g. `PH-0001`. Never reused. |
| `Placeholder` | Exact `#define` name used in code. |
| `Type / Unit` | e.g. `double / m`, `enum`, `Pose2D`. |
| `Sentinel` | Sentinel value used and why it cannot be valid. |
| `Affects` | Pipeline stage(s) or module(s). |
| `Guard` | Guard that fires if the placeholder is unreplaced. |
| `Added by` | Task or commit that introduced it. |
| `Status` | `PROPOSED`, `OPEN`, `RESOLVED`, or `DROPPED` (proposed, then withdrawn before code; user approval). |
| `Resolution` | `RESOLVED` only: value supplied, by whom, when. |

## Registry

| ID | Placeholder | Type / Unit | Sentinel | Affects | Guard | Added by | Status | Resolution |
|---|---|---|---|---|---|---|---|---|
| PH-0001 | `CORE_PH_WHEELBASE_M` | double / m | 0.0; zero is not a valid axle distance | TEB local planner | `initialize()` returns `Status::Fault` | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0002 | `CORE_PH_MAX_STEER_RAD` | double / rad | 0.0; zero steering limit cannot steer | TEB local planner | `initialize()` returns `Status::Fault` | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0003 | `CORE_PH_MIN_TURN_RADIUS_M` | double / m | 0.0; zero turn radius is not physical | TEB local planner (Ackermann) | None (define and guard removed) | Map/guide and Ackermann/runtime-limit task | DROPPED | [FROM: user answer, TEB Phase A approval] Derived as `CORE_PH_WHEELBASE_M / tan(CORE_PH_MAX_STEER_RAD)` instead of stored independently, so the two values cannot disagree. Define removed from `placeholders.hpp`; guard removed from `initialize()`. |
| PH-0004 | `CORE_PH_MAX_SPEED_MPS` | double / m/s | 0.0; zero speed limit cannot move | TEB local planner | `initialize()` returns `Status::Fault` | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0005 | `CORE_PH_SENSOR_STALE_TIMEOUT_S` | double / s | 0.0; zero timeout marks all data stale | Status (`StaleSensor`) | `initialize()` returns `Status::Fault` | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0006 | `CORE_PH_GLOBAL_PLANNER_TIMEOUT_S` | double / s | 0.0; zero timeout fails every plan | Status (`PlannerTimeout`), global planner manager | None (define and guard removed) | Map/guide and Ackermann/runtime-limit task | DROPPED | [FROM: user answer, PH-0006 open question] Replaced by the deterministic `GlobalPlannerConfig::max_expansions` budget, because the core may not read wall-clock time. Define removed from `placeholders.hpp`; guard removed from `initialize()`. |
| PH-0007 | `CORE_PH_INFLATION_RADIUS_M` | double / m | -1.0; 0.0 is valid (no inflation), negative is not | Map manager (Inflation layer) | `initialize()` returns `Status::Fault` while value < 0.0 | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0008 | `CORE_PH_MAP_RESOLUTION_M` | double / m per cell | 0.0; zero cell size is undefined | Map manager (`GridMap`, `kCellResolution_m`) | `initialize()` returns `Status::Fault` | Map/guide and Ackermann/runtime-limit task | OPEN | - |
| PH-0009 | `CORE_PH_WHEEL_RADIUS_M` | double / m | 0.0; zero radius is not physical | Odometry (forward speed) | `initialize()` and `EncoderGyroOdometry::predict()` return `Status::Fault` | Odometry task | OPEN | - |
| PH-0010 | `CORE_PH_LIDAR_MOUNT_X_M` | double / m | NaN; zero is a valid position, so NaN is used | Map manager (LiDAR frame) | `initialize()` returns `Status::Fault` while not finite | Map manager task | OPEN | - |
| PH-0011 | `CORE_PH_LIDAR_MOUNT_Y_M` | double / m | NaN; zero is a valid position, so NaN is used | Map manager (LiDAR frame) | `initialize()` returns `Status::Fault` while not finite | Map manager task | OPEN | - |
| PH-0012 | `CORE_PH_LIDAR_MOUNT_THETA_RAD` | double / rad | NaN; zero is a valid angle, so NaN is used | Map manager (LiDAR frame) | `initialize()` returns `Status::Fault` while not finite | Map manager task | OPEN | - |
| PH-0013 | None | - | - | - | - | - | DROPPED | [FROM: user instruction, audit approval] Dropped before implementation (X/Y lever arm not needed for 2D yaw). No `#define` was ever written. |
| PH-0014 | None | - | - | - | - | - | DROPPED | [FROM: user instruction, audit approval] Dropped before implementation (X/Y lever arm not needed for 2D yaw). No `#define` was ever written. |
| PH-0015 | `CORE_PH_IMU_MOUNT_THETA_RAD` | double / rad | NaN; zero is a valid angle, so NaN is used | IMU yaw sign (only if the IMU is inverted) | Not yet in code (PROPOSED) | Odometry task | DROPPED | [FROM: user instruction, removed before code]. Withdrawn before any placeholder was written; row kept for history. |
| PH-0016 | `CORE_PH_LIDAR_MAX_RANGE_M` | double / m | 0.0; zero range cannot observe anything | Map manager (ray length and occupied-endpoint marking) | `initialize()` returns `Status::Fault` while not > 0.0 | Map manager task | OPEN | - |
| PH-0017 | `CORE_PH_GLOBAL_PLANNER_REPLAN_PERIOD_S` | double / s | 0.0; zero would replan every cycle, which is not a chosen policy | Global planner manager (`needs_replan`) | `initialize()` returns `Status::Fault` while not > 0.0 | Global planner task | OPEN | - |
| PH-0018 | `CORE_PH_ROBOT_RADIUS_M` | double / m | 0.0; zero footprint radius is not physical | TEB local planner (obstacle cost, circumscribed circle) | `initialize()` returns `Status::Fault` while not > 0.0 | TEB local planner task | OPEN | - |
| PH-0019 | `CORE_PH_MAX_ACCEL_MPS2` | double / m/s^2 | 0.0; zero would forbid any speed change | TEB local planner (acceleration limit) | `initialize()` returns `Status::Fault` while not > 0.0 | TEB local planner task | OPEN | - |
| PH-0020 | `CORE_PH_MAX_STEER_RATE_RADPS` | double / rad/s | 0.0; zero would freeze the steering | TEB local planner (steering-rate limit, servo speed) | `initialize()` returns `Status::Fault` while not > 0.0 | TEB local planner task | OPEN | - |
| PH-0021 | `CORE_PH_PATH_SMOOTH_MARGIN_M` | double / m | -1.0; 0.0 is valid (footprint only), negative is not | Global planner (A* path smoothing clearance) | `initialize()` returns `Status::Fault` while value < 0.0; `AStarPlanner::plan()` also returns `Status::Fault` while value < 0.0 (via `footprint_radius_cells()`) | Path smoothing task | OPEN | - |
| PH-0022 | `CORE_PH_PATH_SMOOTH_MAX_SEGMENT_M` | double / m | 0.0; a zero cap allows no jump | Global planner (`smooth_chain`, max length of one smoothed segment) | `initialize()` returns `Status::Fault` while not > 0.0; `AStarPlanner::plan()` also returns `Status::Fault` while not finite, <= 0.0, or < one cell diagonal (via `segment_cap_cells()`) | Path smoothing segment-cap task | OPEN | - |
