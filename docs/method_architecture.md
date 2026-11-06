# Architecture

## Purpose

The core is a drop-in C++ library for an Ackermann-constrained autonomous robot. It exposes one deterministic entry point, `step()`, and treats everything outside its boundary as unknown: sensor drivers, hardware I/O, logging, clock, map loading, e-stop, and localization. The host system supplies every input as an immutable snapshot and receives one output per cycle.
[FROM: AGENTS.md §1]

## Blackbox design

- One public header (`include/core/core.hpp`) and one facade (`Core`).
- The host includes one header and calls `initialize()` once, then `step(InputSnapshot)` each cycle.
- No globals, singletons, or hidden state. All grids and planner buffers are caller-owned views that must outlive the `Core`.
- No file I/O, network I/O, threads, or wall-clock reads. The snapshot timestamp is the only time source.
- The same inputs produce the same outputs.
[FROM: include/core/core.hpp, src/core.cpp, AGENTS.md §4–5]

## Five-stage pipeline

`Core::step()` runs these stages in this fixed order:

| # | Stage | Source | Runs | Role |
|---|---|---|---|---|
| 1 | Odometry | `src/odometry.cpp` | Every cycle | Predicts the next pose from encoder speed and gyro yaw rate. |
| 2 | Map manager | `src/map_manager.cpp` | On a new sweep | Updates the SLAM overlay from LiDAR, rebuilds inflation, composes the costmap. A repeated sweep reuses the existing map. |
| 3 | Global planner manager | `src/global_planner_manager.cpp` | On demand | Produces a topological guide (waypoints) when the goal, map generation, or replan period changes. |
| 4 | TEB local planner | `src/teb_local_planner.cpp` | Every cycle | Optimizes speed and steering over a horizon under Ackermann limits. |
| 5 | Output converter | `src/output_converter.cpp` | Every cycle | Builds the `Output` from the local plan. |

Each stage returns a `Status`. A stage that is not `Valid` stops the cycle. The output is then reported with a `NaN` pose and the stage's status. No stage result is replaced by a fallback.
[FROM: src/core.cpp, AGENTS.md §2]

Step 0, before odometry, checks sensor freshness (see `method_data_flow.md`). The facade also checks that `teb_config.step_dt` equals `snapshot.dt`. A mismatch returns `Fault`.

A repeated LiDAR sweep (`is_new_sweep == false`) is not age-checked, because it is the same measurement and not new data. Its beams are not re-marked: the map stage is skipped and the composed grid is reused. If no sweep has ever been mapped, `step()` returns `Fault` rather than planning on caller-initialized cells. [FROM: user approval, Q3]

## Placeholder system

Values the project has not supplied are compile-time placeholders, not defaults.

- Each placeholder is a `#define` in `include/core/placeholders.hpp`, named in uppercase (for example `WHEELBASE_M`).
- Each has a sentinel that cannot be a valid physical value: zero for quantities where zero is invalid, a negative value where zero is valid but negative is not (`INFLATION_RADIUS_M`, `PATH_SMOOTH_MARGIN_M`), and NaN for mount poses where zero is a valid position.
- Each placeholder can be overridden with `-D` on the compiler command line or by defining it before the include.
- Each placeholder has a row in `UNKNOWNS.md` (`PH-xxxx`). Rows are append-only.
[FROM: include/core/placeholders.hpp, UNKNOWNS.md, AGENTS.md §6]

Runtime tuning (planner weights, iteration counts, horizon) is passed as config structs. These have no defaults. The caller must supply every field.
[FROM: include/core/stages/teb_local_planner.hpp]

## Startup guard

`initialize()` returns `Status::Valid` only when all placeholders have been replaced:

- `Fault` if any of these is not met: `WHEELBASE_M > 0`, `MAX_STEER_RAD > 0`, `MAX_SPEED_MPS > 0`, `SENSOR_STALE_TIMEOUT_S > 0`, `INFLATION_RADIUS_M >= 0`, `MAP_RESOLUTION_M > 0`, `WHEEL_RADIUS_M > 0`, the three LiDAR mount values are finite, `LIDAR_MAX_RANGE_M > 0`, `GLOBAL_PLANNER_REPLAN_PERIOD_S > 0`, `ROBOT_RADIUS_M > 0`, `MAX_ACCEL_MPS2 > 0`, `MAX_STEER_RATE_RADPS > 0`, `PATH_SMOOTH_MARGIN_M >= 0`, `PATH_SMOOTH_MAX_SEGMENT_M > 0`.
- `Core::initialize()` stores the result. `step()` returns `Fault` until a `Valid` result is stored.

The guard only checks that a value was supplied and is in range. It cannot check whether the value is correct.
[FROM: include/core/core.hpp, src/core.cpp]

> **Note:** `PH-0003` (`MIN_TURN_RADIUS_M`) and `PH-0006` (`GLOBAL_PLANNER_TIMEOUT_S`) were dropped. The minimum turn radius is derived from `WHEELBASE_M / tan(MAX_STEER_RAD)`, and the planner budget is `GlobalPlannerConfig::max_expansions`. [FROM: UNKNOWNS.md]
