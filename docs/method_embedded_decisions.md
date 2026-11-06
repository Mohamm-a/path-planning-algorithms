# Embedded decisions

This document records the interface decisions taken for the embedded deployment
of the core, the ESP-32 [FROM: user request], and answers the nine questions from
the algorithm team. It describes the code as it stands after commit `32061c0`.

The core contains no ESP-32-specific code. The target changes which limits are
affordable, not the contract.

## Q1 — float or double?

The core splits the two.

| Kind | Type | Fields |
|---|---|---|
| Spatial and kinematic | `real` (`float`) | `Pose2D`, `Point2D`, `LidarPoint`, the motion inputs in `InputSnapshot`, `TebConfig` speeds/steering/weights, `PlannerWorkspace::g_cost`, `kCellResolution_m` |
| Time | `double` | `timestamp`, `dt`, the five sensor sample times, `TebConfig::step_dt`, `PlanCache::planned_at` |

**Why float for geometry.** One alias means a target can retarget the arithmetic
in one place, and binary32 halves the storage of every spatial array, including
the planner's cost-to-come array and the TEB buffers. The deployment reason is
FPU cost per operation [FROM: user approval, Q1].

**Why double for time.** A binary32 has a 24-bit significand, so at Unix-epoch
magnitude (about 2³¹ s) its spacing is 256 s [FROM: IEEE-754 binary32].
`sensor_freshness()` subtracts two timestamps, so that quantisation would make
the stale check meaningless. Time therefore never narrows.

The split is applied at the boundary only: odometry reads `snapshot.dt` as
`double` and converts it once for the integration step. Geometry precision is
not assessed here; whether the accuracy meets the vehicle is part of Q6/Q7 and
is deferred.

## Q2 — zero allocation in the control loop

`TebWorkspace` holds every per-call buffer: the decision vector, the gradient,
the perturbation probe, the candidate step, and the rollout trajectory. The first
four hold `2 * horizon_steps` reals; the trajectory holds `horizon_steps + 1`
poses.

`solve()` constructs no containers. A null, undersized or aliased buffer returns
`Status::Fault` rather than allocating or truncating. The same rule already
applied to the global planner (`PlannerWorkspace`, `WaypointBuffer`) and to every
map layer, which are non-owning views.

**Why.** A dynamic allocation in an RTOS control loop adds an unbounded worst-case
latency and fragments the heap; the deployment cannot accept either
[FROM: user approval, Q2].

> **Note:** `Core` still allocates once, in its constructor (`std::unique_ptr<Impl>`).
> That is setup time, not `step()` time. After construction, `step()` allocates
> nothing.

## Q3 — slow LiDAR sweeps

`InputSnapshot::is_new_sweep` tells the core whether `lidar` holds a sweep
captured since the previous cycle. When it is false:

1. `lidar_timestamp` is not age-checked, so a repeated sweep is not reported as
   stale.
2. `GridMapManager::update()` is not called: the beams are not re-marked, the
   inflation layer is not rebuilt, and the composed grid is reused unchanged.
3. `map_generation()` does not change, so the cached global plan stays valid.

**Why.** A sweep can take seconds while the control loop runs at tens of hertz.
Re-marking identical beams every cycle spends CPU to produce the same map, and
age-checking the same measurement would stop the core between sweeps
[FROM: user approval, Q3].

If the first cycle reports `is_new_sweep == false`, no map exists yet, so
`step()` returns `Fault` instead of planning on caller-initialized cells. The
repeated-sweep behaviour is covered by a standalone verification harness that is
kept outside the repository, not by the wall simulation.

**Host responsibility.** Every beam in the view is cast from `snapshot.pose`.
The core has no per-beam timestamp and no motion compensation, so a sweep
captured while the robot moved must be compensated outside the core, or the map
will smear along the direction of travel. [FROM: `src/map_manager.cpp`]

## Q4 — no-goal mode

A goal with a NaN component selects no-goal mode. `step()` bypasses the global
planner and the plan cache, and writes a straight-line guide ahead of the robot:
ten waypoints along the current heading, covering the farthest distance one TEB
horizon can travel at `MAX_SPEED_MPS` [FROM: user approval, Q4].

TEB still enforces the Ackermann limits, the acceleration limit and obstacle
clearance, so the local planner becomes a pure reactive obstacle avoider. The
cycle status stays `Valid` while every other check passes.

The guide is rebuilt from the current pose every cycle, so it always starts at
the robot. The ten waypoints must fit in the caller's `WaypointBuffer`; a smaller
capacity returns `Fault`.

## Q5 — commands in the output

`Output` now carries `v_cmd` (m/s) and `delta_cmd` (rad) alongside `next_pose`.
They are copied from the local plan's first horizon step, which is the same step
that produces `next_pose`, so the two always describe one consistent command.
On any non-`Valid` status both are NaN, matching `next_pose`.

**Why.** Converting `next_pose` back into speed and steering on the host repeats
the bicycle-model inversion and loses the steering-rate and acceleration context
that TEB already enforced [FROM: user approval, Q5].

## Q6 — map size and cell size

Deferred to physical testing. The core imposes no fixed size: every grid is a
caller-owned view with its own width, height and origin.

The constraints that decide the size are fixed, even though the numbers are not:
the grid must contain the start, the goal, the robot footprint plus
`INFLATION_RADIUS_M` around every obstacle, and the TEB horizon reach. Memory is
linear in cells: four grid layers at one byte each, plus the planner workspace
at roughly 29 bytes per cell with `real` costs on a 64-bit host
[FROM: the struct definitions].

## Q7 — step time and TEB settings

Deferred. No timing has been measured, so no budget is claimed.

The knobs already exist in `TebConfig` and need no code change: `horizon_steps`
and `iterations` dominate the cost, which is `O(iterations × 2N × N)` cost
evaluations for a horizon of `N` steps. `step_dt` is not free: it must equal
`snapshot.dt`, so the horizon duration is `horizon_steps × dt`.

## Q8 — pose source

Deferred to the host. The core treats `snapshot.pose` as the localized
rear-axle pose and cannot verify it. Odometry contributes only the initial speed
and yaw rate for TEB; it never corrects the pose, and there is no scan matching
or localisation inside the core.

If the host supplies odometry-only poses, drift is the host's to bound. The core
will not detect it, and a drifted pose also smears the SLAM overlay, because
beams are marked from the supplied pose.

## Q9 — sharing rules and the float check

The sections this question refers to are not part of this repository, so their
text could not be reviewed. What the code enforces is:

| Rule | Where |
|---|---|
| Every grid, waypoint buffer and planner buffer is a non-owning view. | `GridMap`, `MutableGridMap`, `WaypointBuffer`, `PlannerWorkspace`, `TebWorkspace` |
| The caller keeps that storage alive for the lifetime of the `Core`. | `Core` constructor contract |
| Writable grids must not share storage; a map update with aliased grids returns `Fault`. | `GridMapManager::update()` |
| TEB decision arrays and the trajectory must not alias; an aliased workspace returns `Fault`. | `TebLocalPlanner::solve()` |
| No globals, no singletons, one `Core` per host. | `Core` is non-copyable and owns all per-cycle state |
| Geometry is `real`; time is `double`. | Q1 above |

If the original sections define different sharing rules, send their text and this
table can be checked against them.
