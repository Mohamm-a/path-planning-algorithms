# Hardware integration: response to the lidcar fit and gap review

Answers to `algo-fit-gap.md` (ESP32-S3, Zephyr 4.4, 10 Hz navigation thread).
Sources below are marked `[FROM: ...]` or `[ASSUMED: ...]` as required by
AGENTS.md. Nothing here is measured on the target: all run-time numbers are
host measurements or the lidcar team's own estimates.

> **Status:** Q1–Q5 and Q9 are implemented. Q6 and Q7 are answered with
> constraints, not values. The §6 float check **does not meet the proposed
> one-cell criterion in a closed-loop run**; the measurements and the reason
> are in "Float regression" below.

## What changed since your review

| Topic | Before your review | Now |
|---|---|---|
| Scalar width | `double` throughout | `using real = float` for geometry, kinematics and costs; `double` for time |
| `step()` allocation | `std::vector` inside TEB | caller-owned `TebWorkspace`; no allocation in `step()` |
| Slow LiDAR | every cycle age-checked, re-marked | `InputSnapshot::is_new_sweep` gates both |
| No goal | not supported | NaN goal selects a straight reactive guide |
| Output | pose only | `v_cmd` and `delta_cmd` added |
| Pin you reviewed | `43025eb` | `0884adc` (three commits ahead: the refactor, the decisions document, and this response) |

## Q1 — `float` through a single alias

Yes. `include/core/core.hpp` defines `using real = float;` and every spatial or
kinematic field uses it: `Pose2D`, `Point2D`, `LidarPoint`, the motion inputs,
`TebConfig` speeds and weights, the A* `g_cost` array and
`map::kCellResolution_m`.

Time stays `double`: `timestamp`, `dt`, the five sensor sample times,
`TebConfig::step_dt` and the plan-cache timestamp. A binary32 significand has 24
bits, so at Unix-epoch magnitude its spacing is 256 s; the stale check subtracts
two timestamps, so time must not narrow. Geometry is converted once at each
boundary, for example `snapshot.dt` is narrowed for the odometry integration
step.

`CORE_REAL_DOUBLE` exists for the §6 regression only. It is a verification
configuration, not a second supported ABI: it changes every struct layout, so
all translation units in a target must agree or the program is ill-formed.

## Q2 — no allocation in `step()`

`step()` allocates nothing. Every per-call buffer is caller-owned:

| Buffer | Contents |
|---|---|
| `teb::TebWorkspace` | decision vector, gradient, probe, candidate (`2 * horizon_steps` reals each) and `horizon_steps + 1` trajectory poses |
| `global_planner::PlannerWorkspace` | `g_cost`, `parent`, `closed`, `heap`, `heap_slot` |
| `global_planner::WaypointBuffer` | global guide store |
| four `map` grids | fixed borders, SLAM overlay, inflation, composed costmap |

A null, undersized or aliased workspace returns `Status::Fault`; it is never
silently allocated or truncated. `Core` allocates once in its constructor
(`std::unique_ptr<Impl>`), which is setup, not control-loop time.

## Q3 — a 5 s sweep

Pass the latest complete sweep with its original timestamp and set
`is_new_sweep = true` only on the cycle it is new. On every later cycle set
`is_new_sweep = false`, either repeating the same view or passing an empty one.

When `is_new_sweep` is false:

- `lidar_timestamp` is **not** age-checked, so the repeated sweep cannot make
  the core `StaleSensor`;
- the map manager is not called, so beams are not re-marked and the inflation
  layer is not rebuilt;
- `map_generation()` does not change, so the cached global plan stays valid.

> **Important:** on the arrival cycle (`is_new_sweep == true`) the timestamp
> **is** checked. **Stamp the completion time of the sweep.** The core processes
> the sweep on delivery, so completion time leaves the age at about one control
> cycle and `SENSOR_STALE_TIMEOUT_S` can stay small. Stamping the start time
> would make a 5 s sweep appear 5 s old on arrival and force the threshold
> above the sweep duration.

Two further consequences for your platform:

1. Every beam is cast from the current `snapshot.pose`. The core has no
   per-beam timestamp and no motion compensation, so a sweep collected while
   the car moves smears along the direction of travel unless the adapter
   compensates it (or the car is stationary during the sweep).
2. If the very first cycle reports `is_new_sweep == false`, no map exists yet
   and the core returns `Status::Fault` rather than planning on uninitialized
   cells. Send one real sweep first.

## Q4 — no goal

A `goal` with a NaN component selects no-goal mode. The core skips A* and the
plan cache and writes a straight guide from the current pose along the current
heading, long enough for one TEB horizon at `MAX_SPEED_MPS`. TEB still enforces
the Ackermann limits, the acceleration limit and obstacle clearance, so the
local planner acts as a reactive obstacle avoider. The cycle stays `Valid`
while every other check passes.

The host chooses the mode per cycle; a finite goal returns to normal planning
on the next call. The guide waypoints must fit in the caller's
`WaypointBuffer`.

## Q5 — commands in the output

`Output` now contains:

```cpp
real v_cmd;      // m/s; speed for the first horizon step
real delta_cmd;  // rad; steering for the first horizon step; + = left
```

They are copied from the same local-plan step that produces `next_pose`, so
the pose and the commands always describe one consistent command. When
`status` is not `Valid`, `next_pose`, `v_cmd` and `delta_cmd` are all NaN.

## Q6 — map size and cell size

The core imposes no minimum size and no fixed cell size. What it requires is
that the grid contains the start, the goal, the robot footprint plus
`INFLATION_RADIUS_M` around every obstacle, and the TEB horizon reach. Beyond
that, larger cells reduce memory and A* work but coarsen the inflation
boundary.

With `real = float`, a 32-bit planner workspace is

```text
sizeof(real) + 3 * sizeof(size_t) + sizeof(bool) = 4 + 12 + 1 = 17 B/cell
```

plus four 1-byte grid layers, so **21 B/cell** in total. That is 4 B/cell less
than the 21 B/cell workspace estimate in your §4.1 (which assumed `double`).
Recalculating your table:

| Area | Cell | Cells | Workspace (17 B) | Grids (4 B) | Total |
|---|---|---|---|---|---|
| 10 × 10 m | 5 cm | 40 000 | 680 KB | 160 KB | ≈ 840 KB |
| 10 × 10 m | 10 cm | 10 000 | 170 KB | 40 KB | ≈ 210 KB |
| 5 × 5 m | 10 cm | 2 500 | 43 KB | 10 KB | ≈ 53 KB |

All three fit your 8 MB PSRAM; the 5 cm map drops from your ≈1.0 MB to
≈840 KB. The core does not care which row you choose; we have no scenario that
sets a minimum.

## Q7 — step time and TEB settings

**No timing is claimed.** We have not measured `step()` on the ESP32-S3, and the
12 ms desktop figure is yours, not ours. Whether the 20–50 ms target is met is
exactly what your step 2 must measure. We accept 20–50 ms as the target.

What the refactor removes is specific and structural:

- `real = float` removes the software-double path on the FPU-less-for-double
  LX7;
- the zero-allocation TEB removes per-cycle heap work and its fragmentation.

Neither is a measured speedup, and neither alone proves the budget.

The knobs you may reduce are runtime configuration, not code changes:
`TebConfig::horizon_steps` and `TebConfig::iterations` dominate the cost, which
is `O(iterations × 2N × N)` cost evaluations for a horizon of `N` steps. The
global planner already runs on demand, gated by
`GLOBAL_PLANNER_REPLAN_PERIOD_S`, not every cycle. `step_dt` must equal
`snapshot.dt`; it is not a free parameter.

## Q8 — odometry-only pose

The core treats `snapshot.pose` as the localized rear-axle pose in `map` and
cannot verify or correct it. Odometry supplies only the initial speed and yaw
rate for TEB; it never corrects the pose. There is no scan matching or
localization inside the core.

So dead-reckoned poses are accepted as input, but drift is yours to bound. A
drifting pose also smears the SLAM overlay, because beams are marked from the
supplied pose, and a 5 s sweep gives the car more time to move during one scan.

## Q9 — sharing rules and the float check

**Section 5 (west project), accepted.** We agree to all five rules:

- the lidcar repository never edits files under `algorithms/robot_core`;
- the car pins an exact commit and moves it by a reviewed `west.yml` change;
- we need no `zephyr/module.yml`, Kconfig or Zephyr CMake;
- our CMake, tests and `wall_sim` keep working;
- the repository stays readable to everyone who builds the car.

Your pin `43025eb` is an ancestor of `0884adc`. **Pin `0884adc`:** it contains
the `float` geometry, the zero-allocation TEB, the sweep gating and the
`v_cmd`/`delta_cmd` output, and it is ready for your step 2 ESP32 timing test.
The only commit after it is documentation. If the timing test forces code
changes, they will land as a new reviewed commit rather than a moving branch.

**Section 6 (float check):** procedure accepted and executed; the result is
below and **fails the proposed criterion**.

## Float regression

**Procedure.** Same commit, same compiler, same 17 placeholder values, same TEB
configuration (100 iterations, horizon 10, `fd_epsilon = 1e-3`, `dt = 0.1 s`),
same wall scenario. The only difference is the scalar alias, selected by
`-DROBOT_CORE_REAL_DOUBLE=ON` on the `core` interface target. `sizeof(real)` was
verified as 4 bytes in the float build and 8 bytes in the double build, and
`-DCORE_REAL_DOUBLE` was confirmed in the compile flags. Poses are logged at the
start of each cycle.

| Metric | `double` | `float` |
|---|---|---|
| `RESULT` | `SUCCESS` | `SUCCESS` |
| Cycles to goal | 121 | 125 |
| Final pose (x, y, θ) | 7.877749444, 7.871491671, 0.791384438 | 7.984554768, 7.826812744, 1.310380220 |
| Status differs on a common cycle | 0 of 121 | 0 of 121 |
| Wall-clearance guard | passed every cycle | passed every cycle |

| Deviation over the 121 common cycles | Value |
|---|---|
| Maximum position difference | **1.233705441 m at cycle 98** |
| Maximum heading difference | 0.685379568 rad |
| First cycle above 1e-6 m | 2 |
| First cycle above 0.01 m | 7 |
| First cycle above 0.1 m | 11 |

**Verdict against §6:** same `RESULT` and same status every cycle, but the
position difference exceeds one map cell (0.1 m) for most of the run. The check
**fails** the proposed criterion. The peak is about 12 cells, and the float run
takes four extra cycles (0.4 s at 10 Hz).

**Interpretation.** The divergence begins at cycle 2 and grows smoothly; there
is no jump, no status change, no collision and no invalid plan. Both runs go
around the same end of the wall, but the float run turns east earlier and
tracks roughly 0.9 m to the right through the middle of the run before
converging at the goal. This is consistent with route-level sensitivity: the
map is rebuilt from float beams, A* breaks near-equal-cost ties differently,
and each cycle's pose becomes the next cycle's input, so an initial rounding
difference is amplified by feedback. [ASSUMED: amplification mechanism; not
isolated by experiment.]

**Why this is expected in a closed loop.** The §6 criterion is an open-loop
metric: it assumes both builds follow the same physical path. The wall
simulation is closed-loop — each cycle's pose becomes the next cycle's LiDAR
origin and planner input — so it has sensitive dependence on initial
conditions. A difference of about 1e-6 m at cycle 2 is enough to change the
steering command slightly; the robot then occupies a slightly different pose,
sees the wall from a slightly different angle, and the two runs settle into
different but equally safe routes. The 1.23 m peak is the distance between two
routes, not an error accumulated in the arithmetic: the separation collapses
to 0.31 m by cycle 120 as both converge, and neither run ever reports a
non-`Valid` status or violates the clearance guard. Bitwise-identical
trajectories would require identical arithmetic, not merely stable arithmetic.
An open-loop replay of one sensor stream through both builds would isolate the
arithmetic if you want that number as well.

We did **not** tune the tolerance, the scenario or the TEB settings to make the
check pass. If you want a different comparison — open-loop replay of one sensor
stream, another scenario, or a different tolerance — specify it and we will run
it and publish the numbers.

## Adapter Requirements

Conventions the adapter must honour. [FROM: AGENTS.md section 5, README
"Coordinate frame", `include/core/core.hpp`.]

### Units

| Quantity | Core unit | From lidcar | Conversion |
|---|---|---|---|
| Position | m | cm | divide by 100 |
| Angle, steering | rad | centi-degrees | multiply by π / 18000 |
| Time, durations | s | ms | divide by 1000 |
| Speed | m/s | — | from your plant |
| Encoder | RPM | — | documented exception; no conversion |

### Frames

- Frame names: `map`, `odom`, `base_link`, `lidar`, `imu`.
- `base_link` origin: centre of the rear axle on the ground plane.
- `base_link` axes: **+x forward, +y left**.
- Heading: CCW positive, normalized to (-π, π] at the boundary.
- The input pose is the rear-axle pose in `map`, not the body centre and not
  the LiDAR position.

Your framework uses +y forward and +x right. That is a −90° rotation about z:

```text
x_base_link =  y_lidcar
y_base_link = -x_lidcar
```

Apply the same rotation to heading and to every vector before filling
`InputSnapshot`. [ASSUMED: derived from your §2 table; verify the heading sign
against your encoder convention.]

### LiDAR

- `r` in m; `alpha` in rad, CCW from the **lidar's** +x axis.
- Your sweep angle (0 = right, 90 = ahead, 180 = left) maps to
  `alpha = (angle_deg − 90) × π / 180`.
- A beam with no return inside `LIDAR_MAX_RANGE_M` must be reported with
  `r > LIDAR_MAX_RANGE_M`; the map manager then treats it as no return.
- Beams are cast from `snapshot.pose`. Motion-compensate a moving sweep.
- The mount offsets `LIDAR_MOUNT_X_M`, `LIDAR_MOUNT_Y_M`,
  `LIDAR_MOUNT_THETA_RAD` are measured from `base_link`.

### Timestamps and cycle

- `timestamp` and all five sensor times are `double` seconds in one epoch.
- `snapshot.dt` must equal `TebConfig::step_dt`; both are 0.1 s at 10 Hz.
- Set `is_new_sweep` as described in Q3, and stamp the sweep as you will
  interpret it (start or completion).
- A negative age (a sample from the future) is a `Fault`, not a stale sensor.

### Goal and output

- Finite `goal` → normal A* + TEB.
- NaN in either goal component → no-goal reactive mode.
- Consume `v_cmd` (m/s) and `delta_cmd` (rad, positive left) directly; do not
  invert `next_pose`.
- Any status other than `Valid` means all three numeric outputs are NaN.

### Buffers and placeholders

- Every grid, waypoint buffer and workspace is a non-owning view. Keep the
  storage alive for the lifetime of `Core`, and do not alias the four grids or
  the TEB arrays.
- The four grids must share `origin`, `width` and `height`.
- Supply all 17 placeholders as `-D` flags; `initialize()` returns `Fault`
  while any sentinel remains, and `step()` refuses to run before a `Valid`
  `initialize()`.
