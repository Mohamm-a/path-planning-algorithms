# wall_sim

End-to-end wall simulation for `core::Core`. It is an opt-in developer tool.
It is not part of the core library and is not built by default.

## Scenario

| Item | Value |
| --- | --- |
| Start pose | (1.0 m, 1.0 m, 0.0 rad) |
| Goal | (8.0 m, 8.0 m) |
| Goal tolerance | 0.2 m |
| Wall | segment (5.0, 2.0) to (5.0, 8.0) |
| LiDAR | 180 beams over -1.57 to 1.57 rad |
| Cycle period | 0.1 s |
| Max cycles | 500 |
| Wall clearance | 0.05 m beyond robot radius |

The plant model assumes the robot tracks `next_pose` exactly each cycle, with
no actuator or sensor noise. Sensor values are derived from the applied motion,
so the core's odometry reproduces the simulated pose.

## Build

The sim is off by default. Enable it with the CMake option `ROBOT_CORE_BUILD_SIM`:

```bash
cmake -S . -B build -DROBOT_CORE_BUILD_SIM=ON \
      -DCMAKE_CXX_FLAGS="-DWHEELBASE_M=<value> -DMAX_STEER_RAD=<value> \
        -DMAX_SPEED_MPS=<value> -DSENSOR_STALE_TIMEOUT_S=<value> \
        -DINFLATION_RADIUS_M=<value> -DMAP_RESOLUTION_M=<value> \
        -DWHEEL_RADIUS_M=<value> -DLIDAR_MOUNT_X_M=<value> \
        -DLIDAR_MOUNT_Y_M=<value> -DLIDAR_MOUNT_THETA_RAD=<value> \
        -DLIDAR_MAX_RANGE_M=<value> -DGLOBAL_PLANNER_REPLAN_PERIOD_S=<value> \
        -DROBOT_RADIUS_M=<value> -DMAX_ACCEL_MPS2=<value> \
        -DMAX_STEER_RATE_RADPS=<value> -DPATH_SMOOTH_MARGIN_M=<value> \
        -DPATH_SMOOTH_MAX_SEGMENT_M=<value>"
cmake --build build --target wall_sim
```

> **Note:** The placeholders must be supplied in `CMAKE_CXX_FLAGS` so the
> core libraries and the sim compile with the same values. Passing them as
> plain `-D` cache variables does not work. The sim does not provide values. If a placeholder is unset, `initialize()` refuses
> to run and the sim prints `RESULT FAIL initialize`. See
> [`UNKNOWNS.md`](../UNKNOWNS.md) for the full placeholder list.

## Run

```bash
./build/sim/wall_sim [speed_mps] [w_guide] [w_obstacle]
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `speed_mps` | 1.0 | Target speed, must be > 0 |
| `w_guide` | 1.0 | TEB weight for the global guide, must be >= 0 |
| `w_obstacle` | 10.0 | TEB weight for obstacles, must be >= 0 |

Invalid arguments print `RESULT FAIL invalid <name> argument '<value>'`.

## Output

The first line is a header. Each following line is one control cycle:

```
cycle t x y theta status dist_goal clearance
```

| Column | Unit | Meaning |
| --- | --- | --- |
| `cycle` | count | Control cycle index |
| `t` | s | Simulated time |
| `x`, `y` | m | Map-frame position |
| `theta` | rad | Heading |
| `status` | enum | `Status` value from `core.hpp` (0 = `Valid`, 1 = `NoPath`, ...) |
| `dist_goal` | m | Distance to goal |
| `clearance` | m | Distance to the wall |

The run ends with exactly one `RESULT` line:

- `RESULT SUCCESS cycles=<n> final <x> <y> <theta>`: goal reached.
- `RESULT FAIL max cycles (500) reached without goal, ...`: goal not reached.
- `RESULT FAIL wall clearance at cycle ...`: the robot came inside the wall clearance.
- `RESULT FAIL status <n> at cycle ...`: the core returned a non-`Valid` status.
- `RESULT FAIL initialize`: the core refused to initialize (see the note above).

## Known limitation: goal not reached near the wall corner

The robot does not reach the goal in the runs observed so far. The outcome
depends on the placeholder values:

- One run with placeholder values not recorded in this repo ended at the cycle
  limit (`RESULT FAIL max cycles (500) reached without goal`). The core stayed
  `Valid` with near-zero forward speed, near the wall corner.
- A run with other throwaway values returned `RESULT FAIL status 1 at cycle 74`
  (`NoPath`), near (4.3 m, 5.5 m).

The coordinates above are not reproducible from this repo, because the
values that produced them are not recorded. [UNKNOWN: the placeholder values
used for the first run.] Re-run the sim with the approved placeholder values
before relying on either result.
