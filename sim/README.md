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
./build/sim/wall_sim [speed_mps] [w_guide] [w_obstacle] [astar|dstar|rrt]
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `speed_mps` | 0.4 | Target speed, must be > 0 and <= `MAX_SPEED_MPS` |
| `w_guide` | 0.3 | TEB weight for the global guide, must be >= 0 |
| `w_obstacle` | 10.0 | TEB weight for obstacles, must be >= 0 |
| `planner` | `astar` | Global planner: `astar`, `dstar` or `rrt` |

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
| `clearance` | m | Distance to the wall minus `ROBOT_RADIUS_M` |

The run ends with exactly one `RESULT` line:

- `RESULT SUCCESS cycles=<n> final <x> <y> <theta>`: goal reached.
- `RESULT FAIL max cycles (500) reached without goal, ...`: goal not reached.
- `RESULT FAIL wall clearance at cycle ...`: the robot came inside the wall clearance.
- `RESULT FAIL status <n> at cycle ...`: the core returned a non-`Valid` status.
- `RESULT FAIL initialize`: the core refused to initialize (see the note above).

## Results with the approved values

Command: `wall_sim 0.4 0.3 10 <planner>` (0.4 m/s target, TEB weights 0.3 and
10, 500-cycle limit). These are the reproducible outcomes; no budget was raised
to obtain them.

| Planner | Result | Outcome |
| --- | --- | --- |
| `astar` | `RESULT SUCCESS cycles=314 final 7.957602024 7.826906681 1.297009230` | Reaches the 0.2 m goal tolerance |
| `dstar` | `RESULT SUCCESS cycles=322 final 7.922506332 7.837221622 1.169487238` | Reaches the 0.2 m goal tolerance |
| `rrt` | `RESULT SUCCESS cycles=340 final 7.944367409 7.836927891 1.236014605` | Reaches the 0.2 m goal tolerance |

The `dstar` run uses the goal-progress extraction tie, the D*-Lite-only
two-cell smoothing allowance, and the soft clearance preference documented in
`docs/method_algorithms.md`; the physical clearance check remains
`ROBOT_RADIUS_M = 0.30 m`. The `rrt` run uses the straight-line shortcut,
reserved exact-goal slot, and backward line-of-sight pull. A* at 0.5 m/s still
fails before reaching the goal: `RESULT FAIL wall clearance at cycle 87 pose
5.013302326 1.664465666 0.473239183`.

`clearance` is distance to the wall minus `ROBOT_RADIUS_M`, so a positive value
means the robot circle is still outside the wall. The harness fails when it
falls below the separate 0.05 m margin. A `NoPath` result is a pipeline status,
not a wall-clearance failure.

## TEB weight diagnostic

With `w_obstacle = 10`, raising `w_guide` does not improve the D* result:

| Planner | `w_guide = 1.0` | `w_guide = 3.0` |
| --- | --- | --- |
| `dstar` | wall clearance at cycle 377 | 500-cycle limit, final (1.0255, 1.0000) |
| `rrt` | success, 419 cycles | 500-cycle limit, final (4.7299, 1.0794) |

The default `w_guide = 0.3` run is the accepted baseline; no TEB weight was
changed in the harness for the passing D*/RRT results.
