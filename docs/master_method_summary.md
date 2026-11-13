# Master method summary

This document gives a formal, implementation-independent overview of the core. It covers the data flow, the kinematic model, the four algorithmic stages, the embedded constraints, the configuration guard, and the verification method. Detailed derivations and edge cases are in the companion method documents.

## 1. System architecture and data flow

### 1.1 Input snapshot

The host supplies one immutable snapshot per control cycle. The snapshot is the complete input to the core; the core reads no sensors, clock, or global state directly.

| Input | Representation | Meaning |
|---|---|---|
| Pose | `(x, y, theta)` in the map frame | Localized rear-axle pose. `theta` is positive counter-clockwise. |
| Goal | `(x, y)` in the map frame | Desired position. A NaN goal selects no-goal mode, where the local planner follows a straight reactive guide. |
| LiDAR | Sequence of `(r, alpha)` | Beam length in metres and beam angle in radians, measured counter-clockwise from the LiDAR +x axis. The host owns the storage. |
| Encoder | Signed RPM | Rotation rate of the drive wheel. The encoder is mounted wheel-side. |
| Steering | Radians | Measured steering angle; positive is a left turn. |
| Gyro | Radians per second | Measured yaw rate; positive is counter-clockwise. |
| Timing | Timestamp, `dt`, five sensor sample times | One epoch, in seconds. `dt` is the control period and must equal the local planner's horizon step. |
| Sweep flag | Boolean | True only when the LiDAR view contains a sweep captured since the previous cycle. |
| Planner kind | Enumeration | Requests A*, D* Lite, or RRT. Only A* is currently implemented. |

The LiDAR is deliberately allowed to be slower than the control loop. A complete sweep is delivered once and then repeated with `is_new_sweep = false` until the next sweep is ready. The sweep should be stamped at completion time, so its age at delivery is approximately one control cycle.

### 1.2 Output

| Output | Representation | Meaning |
|---|---|---|
| Next pose | `(x, y, theta)` in the map frame | Pose after the first horizon step. `theta` is normalized to (-pi, pi]. |
| Speed command | Metres per second | Speed for the first horizon step. |
| Steering command | Radians | Steering for the first horizon step; positive is a left turn. |
| Timing and frame | `dt`, `timestamp`, `frame_id` | Cycle period, cycle time, and map-frame identifier. |
| Status | Enumeration | `Valid`, `NoPath`, `StaleSensor`, `PlannerTimeout`, or `Fault`. |

The commands are taken from the same local-plan step that produces the next pose, so pose and commands are always consistent. When the status is not `Valid`, the next pose and both commands are NaN. A consumer therefore cannot accidentally execute a partial or failed plan.

### 1.3 Five-stage pipeline

The pipeline order is fixed:

| Stage | Runs | Function |
|---|---|---|
| 0. Freshness check | Every cycle | Rejects future, non-finite, or stale sensor samples before any planning. The LiDAR age is checked only when `is_new_sweep` is true. |
| 1. Odometry | Every cycle | Converts encoder speed and gyro yaw rate into a predicted motion and forward speed. |
| 2. Map manager | On a new sweep | Marks the SLAM overlay, rebuilds the inflation layer, and composes the costmap. A repeated sweep reuses the previous map without re-marking beams. |
| 3. Global planner | On demand | Runs A* when the goal, map generation, or replan period changes. Produces a topological waypoint guide. |
| 4. TEB local planner | Every cycle | Optimizes speed and steering over a fixed horizon under the Ackermann limits and obstacle clearance. |
| 5. Output converter | Every cycle | Publishes the first plan step as the next pose and the two commands. |

Each stage returns a status. A non-`Valid` status halts the cycle immediately: later stages do not run, the output carries that status, and the numeric fields are NaN. There is no silent fallback to a previous plan, zero command, or alternative algorithm. This fail-stop behaviour is what makes a failed map update, a missing path, or an invalid sensor sample visible to the host.

The freshness check has one deliberate exception. When `is_new_sweep` is false, the LiDAR timestamp is not re-aged, because the same complete sweep is being reused and is not new data. The map manager is also skipped, so the costmap remains unchanged. If no sweep has ever been mapped, the cycle returns `Fault` rather than planning on uninitialized cells.

## 2. Kinematics and physics

### 2.1 Ackermann bicycle model

The robot is modelled as a kinematic bicycle: the two front wheels are merged into one steered wheel and the two rear wheels into one driven wheel. The state is position `(x, y)` and heading `theta` in the map frame. The model is:

```text
x'     = v * cos(theta)
y'     = v * sin(theta)
theta' = v * tan(delta) / L
```

where `v` is the signed forward speed, `delta` is the steering angle, and `L` is the wheelbase. The corresponding yaw-rate relation is:

```text
Yaw Rate = (Speed * tan(Steering)) / Wheelbase
```

The steering angle must remain strictly below a right angle, because `tan(delta)` is unbounded at `±pi/2`. The configured limit is `pi/4`, which gives a finite tangent of 1. With a wheelbase of 0.245 m, the minimum rear-axle turn radius is:

```text
R_min = L / tan(delta_max) = 0.245 / tan(pi/4) = 0.245 m
```

The model is kinematic: it assumes ideal rolling, no tyre slip, no actuator delay, and perfect tracking of the commanded speed and steering. These assumptions are appropriate for the low-speed planning layer; they are not a dynamic vehicle model.

### 2.2 Why the origin is the rear axle

The `base_link` origin is defined as the centre of the rear axle on the ground plane. This is a mathematical requirement, not a naming convention. The bicycle equations describe the motion of the rear-axle centre: the heading rate depends on the distance from that centre to the front axle. If the origin were moved to the body centre or the front axle, the `x'` and `y'` equations would describe that point's motion, and the `tan(delta)/L` term would no longer produce the correct yaw rate.

The host must therefore supply the pose of the rear-axle centre, not the geometric centre of the chassis. Sensor mount offsets are measured from the same origin. A LiDAR mounted on a mast directly above the rear axle has zero x and y offset in this frame; its height is not represented because the map is two-dimensional.

## 3. Algorithmic logic

The following pseudocode is intentionally implementation-independent. It shows the logical flow and the decisions that affect behaviour, not the C++ syntax.

### 3.1 Odometry

Odometry uses the encoder for signed forward speed and the gyro for yaw rate. The steering angle is not used in this stage; it is consumed later by the local planner.

```text
function ODOMETRY(snapshot):
    if wheel radius is not positive, or any input is non-finite:
        return Fault

    wheel_angular_rate = encoder_rpm * (2*pi / 60)
    speed              = wheel_angular_rate * wheel_radius
    yaw_rate           = gyro_yaw_rate

    mid_heading = theta + yaw_rate * dt / 2
    x_next      = x + speed * cos(mid_heading) * dt
    y_next      = y + speed * sin(mid_heading) * dt
    theta_next  = wrap_to_pi(theta + yaw_rate * dt)

    return Valid with (x_next, y_next, theta_next), speed, yaw_rate
```

The midpoint heading integrates half of the yaw change before propagating position, which gives a second-order update over one control cycle. The wheel radius is the effective radius of the drive wheel; because the encoder is wheel-side, no gear ratio is applied.

### 3.2 Map manager

The map manager is the only stage that writes the occupancy representation. It runs only when a new sweep is delivered.

```text
function UPDATE_MAP(pose, lidar, is_new_sweep):
    if not is_new_sweep:
        if no map has ever been built:
            return Fault
        return the existing map

    for each beam in the sweep:
        origin = pose position + rotate(lidar mount offset, pose.theta)
        angle  = pose.theta + lidar mount yaw + beam.alpha
        length = min(beam.r, lidar_max_range)
        endpoint = origin + length * (cos(angle), sin(angle))

        clip the segment from origin to endpoint to the grid
        if the segment is entirely outside:
            continue

        for each cell crossed by the clipped segment:
            mark the cell Free

        if the return is within range and the endpoint is inside:
            mark the final cell Occupied

    rebuild the inflation layer:
        set every inflation cell to Unknown
        for each Occupied cell in the fixed or SLAM layer:
            mark every cell whose centre is within the inflation radius

    compose the layers:
        Occupied if any layer is Occupied
        else Free if any layer is Free
        else Unknown

    increment the map generation if the composed map changed
    return Valid
```

Two properties matter for a slow LiDAR:

1. A repeated sweep is not re-marked. The map generation and the global plan therefore remain stable between sweeps.
2. The repeated sweep is not age-checked. A five-second sweep does not make the core report `StaleSensor` on every cycle; the host controls when a sweep is genuinely new.

The costmap is conservative. Cells that have never been observed are `Unknown`, and both planners treat `Unknown` as blocked. This prevents planning through unseen space, at the cost of requiring the map to be explored before a route can be found.

### 3.3 Global planner: A* with line-of-sight smoothing

The global planner runs on demand: when the goal changes, when the map generation changes, or when the replan period has elapsed. It searches the composed costmap, not the raw LiDAR frame.

```text
function PLAN_A_STAR(start, goal, composed_map):
    if start or goal is outside the grid, or the grid is unusable:
        return Fault
    if start or goal is Occupied:
        return Fault
    if start or goal is Unknown:
        return NoPath

    open_list = min-heap ordered by f = g + h
    g[start] = 0
    expansions = 0

    while open_list is not empty:
        current = pop the cell with the lowest f
        if current is the goal:
            reconstruct the path through the parent links
            break
        if expansions reached the configured budget:
            return PlannerTimeout

        expansions += 1
        for each of the eight neighbours:
            skip if the neighbour is closed
            skip if any cell in the clearance disk around it is not Free
            skip a diagonal move if either orthogonal neighbour is not Free
            tentative = g[current] + step_cost
            if tentative improves g[neighbour]:
                update parent and g; insert or decrease the key

    if the goal was not reached:
        return NoPath

    guide = smooth the cell path by greedy line-of-sight jumps
    return Valid with guide waypoints
```

The heuristic is octile distance, which is consistent for the eight-connected move costs. Ties are broken by the lower cell index, so the search is deterministic.

The clearance disk is sized as:

```text
clearance_cells = ceil((robot_radius + half_cell_diagonal) / map_resolution)
```

This matches the clearance that the local planner enforces. At a robot radius of 0.30 m, a map resolution of 0.05 m, and a half-diagonal of about 0.035 m, the search uses seven cells.

Smoothing shortens the grid path by jumping to the farthest cell that is visible along a straight line. Every cell along the candidate segment must have a free footprint disk, and the jump length is capped. The result is a topological guide: a short polyline that tells the local planner where to go, not a full trajectory.

### 3.4 Local planner: projected finite-difference TEB

The local planner runs every cycle. It optimizes a sequence of `N` speeds and `N` steering angles over a horizon of `N` steps. The decision vector has `2N` entries. The first speed and first steering angle become the commands for the current cycle.

The cost function has five terms:

```text
J = sum over trajectory points of:
        w_guide    * (distance to the guide polyline)^2
      + w_obstacle * max(0, robot_radius - clearance)^2
  + sum over horizon steps of:
        w_speed        * (speed - target_speed)^2
      + w_accel        * (speed_k - speed_{k-1})^2
      + w_steer_change * (steering_k - steering_{k-1})^2
```

The guide term pulls the trajectory toward the global route. The obstacle term penalizes any point closer than the robot radius to a blocked cell. The speed term tracks the reference speed, while the acceleration and steering-change terms discourage abrupt commands. All weights are runtime configuration; the core supplies no defaults.

The optimizer is a projected finite-difference descent:

```text
function TEB(pose, initial_speed, initial_steering, guide, costmap, config):
    x = [initial_speed repeated N, initial_steering repeated N]
    x = project_onto_limits(x)
    J = cost(x)
    step = config.step_size

    repeat config.iterations times:
        for each component i of x:
            probe = x with component i increased by config.fd_epsilon
            gradient[i] = (cost(probe) - J) / config.fd_epsilon

        candidate = project_onto_limits(x - step * gradient)
        candidate_cost = cost(candidate)
        if candidate_cost < J:
            x = candidate
            J = candidate_cost
        else:
            step = step / 2

    poses = rollout(x)
    if any pose is closer than robot_radius to a blocked cell:
        return NoPath

    return Valid with first speed, first steering, and the trajectory
```

Projection enforces the physical limits before every cost evaluation:

| Limit | Configured value |
|---|---|
| Wheelbase | 0.245 m |
| Maximum steering magnitude | pi/4 rad |
| Maximum speed | 0.5 m/s |
| Maximum acceleration | 1.0 m/s² |
| Maximum steering rate | pi/3 rad/s |
| Robot radius | 0.30 m |

Speeds are clamped to the speed box and to an acceleration window around the previous speed. Steering angles are clamped to the steering box and to a rate window around the previous angle. Because projection happens after every candidate step, the optimizer only searches over feasible histories; the output cannot violate the limits even if the unconstrained gradient points outside them.

The final rollout is checked point by point. If any point on the horizon is inside the robot radius of a blocked cell, the cycle returns `NoPath` rather than publishing a colliding command.

## 4. Embedded system constraints

### 4.1 Single precision for geometry, double precision for time

The core defines one scalar alias for spatial and kinematic quantities: positions, angles, speeds, map resolution, and planner weights are single-precision `float`. Timestamps and durations remain `double`.

The reason is specific to the target class of microcontroller. The ESP32-S3 has a single-precision floating-point unit; double-precision arithmetic is performed in software and is substantially more expensive. Geometry at the scale of a small robot, with a map resolution of 0.05 m, does not need binary64 precision. Using `float` for geometry removes the software-double path from the control loop.

Time is different. At Unix-epoch magnitude, a binary32 value has a spacing of roughly 256 seconds, so a `float` timestamp cannot represent sub-second sensor ages. The staleness check subtracts two timestamps; if time were narrowed, a fresh sample could appear old or a stale sample could appear fresh. Timestamps and `dt` therefore stay `double`, and geometry is converted once at each boundary.

A verification-only build can switch the geometry alias to `double` for regression testing. That build is not a second supported ABI: it changes every structure layout, so all translation units in a target must use the same setting.

### 4.2 Zero allocation in the control loop

The core is designed for a deterministic real-time loop on Zephyr. `step()` performs no heap allocation. Every buffer is caller-owned and supplied through a non-owning view:

| Buffer | Contents |
|---|---|
| TEB workspace | Decision vector, gradient, probe, candidate, and trajectory poses |
| A* workspace | Cost array, parent array, closed flags, heap, and heap slots |
| Grid views | Fixed borders, SLAM overlay, inflation layer, composed costmap |
| Waypoint buffer | Global guide output |

The `Core` object allocates its private implementation once during construction, which is setup rather than control-loop work. A null, undersized, or aliased workspace returns `Fault`; the core never allocates a replacement or silently truncates a buffer.

This design avoids heap fragmentation, unpredictable allocator latency, and out-of-memory failures during motion. It also makes memory use statically analysable. At 32 bits, the A* workspace is 17 bytes per cell (4-byte cost, three 4-byte indices/flags, and one byte for the closed flag), plus four one-byte grid layers, for 21 bytes per cell. A 10 m by 10 m map at 0.05 m resolution has 40,000 cells and requires approximately 840 KB, which fits the target's PSRAM.

## 5. Placeholder and guard system

Every physical dimension, limit, calibration, and timing constant is a compile-time `#define` in `placeholders.hpp`. This keeps the physical configuration out of runtime state and guarantees that every translation unit in a build sees the same value.

The 17 values are approved and recorded as `RESOLVED` in `UNKNOWNS.md`:

| Quantity | Placeholder | Value |
|---|---|---|
| Wheelbase | `WHEELBASE_M` | 0.245 m |
| Maximum steering | `MAX_STEER_RAD` | pi/4 rad |
| Maximum speed | `MAX_SPEED_MPS` | 0.5 m/s |
| Stale timeout | `SENSOR_STALE_TIMEOUT_S` | 1.0 s |
| Inflation radius | `INFLATION_RADIUS_M` | 0.05 m |
| Map resolution | `MAP_RESOLUTION_M` | 0.05 m per cell |
| Wheel radius | `WHEEL_RADIUS_M` | 0.056 m |
| LiDAR x offset | `LIDAR_MOUNT_X_M` | 0.0 m |
| LiDAR y offset | `LIDAR_MOUNT_Y_M` | 0.0 m |
| LiDAR yaw | `LIDAR_MOUNT_THETA_RAD` | 0.0 rad |
| LiDAR maximum range | `LIDAR_MAX_RANGE_M` | 8.0 m |
| Replan period | `GLOBAL_PLANNER_REPLAN_PERIOD_S` | 0.1 s |
| Robot radius | `ROBOT_RADIUS_M` | 0.30 m |
| Maximum acceleration | `MAX_ACCEL_MPS2` | 1.0 m/s² |
| Maximum steering rate | `MAX_STEER_RATE_RADPS` | pi/3 rad/s |
| Path smoothing margin | `PATH_SMOOTH_MARGIN_M` | 0.0 m |
| Maximum smoothed segment | `PATH_SMOOTH_MAX_SEGMENT_M` | 0.5 m |

The `initialize()` guard validates the configuration before the first cycle. It rejects zero or negative values where a positive quantity is required, negative values where zero is valid, and non-finite mount poses. A failure returns `Fault`; `step()` refuses to run until initialization succeeds. The guard cannot judge whether a value is physically correct, so overrides still require a reviewed change and a rebuild.

The guard is intentionally retained after resolution. The values are no longer missing, but a parent build can still override them with `-D`, and an accidental zero or negative value must fail loudly rather than reach the planners.

## 6. Simulation and verification

The architecture is exercised end to end by `sim/wall_sim.cpp`, a two-dimensional closed-loop simulation. A virtual robot moves from a start pose to a goal in a 10 m by 10 m world containing one vertical wall. A virtual LiDAR produces 180 beams over a 180-degree forward span. The plant tracks the commanded next pose exactly, and the encoder, steering, and gyro values for the next cycle are derived from that motion, so the core's odometry reproduces the simulated pose.

The simulation runs at 10 Hz with a 0.1 s control period. It uses the approved physical values: 0.30 m robot radius, 0.05 m map resolution, 0.05 m inflation, 0.5 m/s target speed, and the A* and TEB stages described above. The wall is sensed through the LiDAR rather than loaded into the fixed layer, so the map manager, inflation layer, and global planner all participate.

A run succeeds only when all of the following hold:

1. The robot reaches the goal within the configured tolerance.
2. The distance from the wall, minus the robot radius, never falls below the simulation's clearance margin.
3. Every cycle returns `Valid`; `NoPath`, `StaleSensor`, `PlannerTimeout`, and `Fault` terminate the run.
4. The run completes within the cycle limit.

The unmodified simulation, with a target speed of 0.5 m/s and the approved configuration, reaches the goal in 240 cycles (24 s). The final pose is approximately `(7.963, 7.807, 1.379)`, and the run reports `RESULT SUCCESS`. The same run is built with `-Wall -Wextra -Wpedantic -Werror`; compilation, the startup guard, and the closed-loop run all pass.

The simulation validates the interaction of the stages, the map/inflation pipeline, the guide smoothing, the local planner's feasibility checks, and the output contract. It is not a hardware test: the plant is ideal, there is no sensor or actuator noise, and the ESP32-S3 execution time is not measured. Those measurements remain part of the physical commissioning step.
