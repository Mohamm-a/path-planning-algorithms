# Algorithms

Pseudocode below is a logical description, not C++. Names match the code.

## 1. Odometry

Source: `src/odometry.cpp`. [FROM: src/odometry.cpp header]

Odometry uses the encoder for forward speed and the gyro for yaw rate. It does not use `steering_angle`. The steering value stays in the snapshot for other stages.

```text
function predict_odometry(snapshot):
    if WHEEL_RADIUS_M is not positive, or any input is not finite:
        return Fault with NaN fields
    v     = encoder_rpm * (2π / 60) * WHEEL_RADIUS_M      # m/s, signed
    omega = gyro_yaw_rate                                  # rad/s
    theta_mid = theta + omega * dt / 2
    x_next = x + v * cos(theta_mid) * dt
    y_next = y + v * sin(theta_mid) * dt
    theta_next = wrap_to_(-π, π](theta + omega * dt)
    return Valid with (x_next, y_next, theta_next), forward_speed = v
```

- `WHEEL_RADIUS_M` is configured as `PH-0009` (0.056 m). The guard rejects zero and negative values.
- The midpoint heading halves the yaw change, which gives a second-order position update over one cycle. [ASSUMED: midpoint integration is the standard choice; AGENTS.md does not specify the scheme.]
- Sign: positive `encoder_rpm` is forward, negative is reverse. [ASSUMED: from odometry header comment.]

## 2. Map manager

Source: `src/map_manager.cpp`. Three layers, composed with priority Occupied > Free > Unknown. [FROM: src/map_manager.cpp]

`Core::step()` runs this stage only when `is_new_sweep` is true. On a repeated sweep it reuses the composed grid unchanged and skips the LiDAR age check, so a slow sensor does not stop the core between sweeps. [FROM: user approval, Q3]

### 2a. LiDAR ray marking (SLAM overlay)

```text
function cast_beam(slam_grid, pose, beam):
    sensor_origin = pose position + rotate(LIDAR mount offset, pose.theta)
    beam_angle    = pose.theta + LIDAR_MOUNT_THETA_RAD + beam.alpha
    length        = min(beam.r, LIDAR_MAX_RANGE_M)
    endpoint      = sensor_origin + length * (cos(beam_angle), sin(beam_angle))

    clip the segment (origin, endpoint) to the grid box
    if the clip is empty:
        return                       # beam lies outside the grid: no change

    hit = beam.r <= LIDAR_MAX_RANGE_M
          and the endpoint was not clipped
          and the endpoint is inside the grid

    for each cell the clipped segment passes through:
        set the cell to Free
    if hit:
        set the final cell to Occupied
```

- A return beyond `LIDAR_MAX_RANGE_M` marks every traversed cell Free and does not mark an endpoint. The beam is truncated at the range.
- Cells the beam passes through are Free. The cell where a real return lands is Occupied.
- The mount offsets are configured values (`PH-0010` to `PH-0012`): x = 0.275 m, y = 0.0 m, yaw = 0.0 rad. They are measured from `base_link`.

### 2b. Inflation layer (safety buffer)

```text
function build_inflation(fixed_grid, slam_grid, inflation_grid):
    set every cell of inflation_grid to Unknown
    for each cell c in fixed_grid or slam_grid that is Occupied:
        for each cell n within INFLATION_RADIUS_M of c (cell-center distance):
            set inflation_grid[n] = Occupied
```

- Cell distance uses cell centers: an offset `(dx, dy)` is inflated when `(dx² + dy²) * res² ≤ INFLATION_RADIUS_M²`. [ASSUMED: metric radius, from map_manager.cpp comment.]
- `INFLATION_RADIUS_M` is configured as `PH-0007` (0.05 m). A value of `0` is valid and means no inflation; a negative value is rejected.
- Inflation is a separate layer, not a branch inside a planner. [FROM: AGENTS.md §2]

### 2c. Composition

```text
function compose(fixed, slam, inflation):
    for each cell i:
        if any layer is Occupied: composed[i] = Occupied
        else if any layer is Free: composed[i] = Free
        else: composed[i] = Unknown
```

## 3. Global planners

Source: `src/global_planner_manager.cpp`. `GlobalPlannerKind::AStar`,
`DStarLite` and `RRT` are implemented. `make_global_planner()` selects one; an
unknown enum value returns a planner that always reports `Fault`, never a
fallback to another algorithm. `Core::step()` calls the selected planner only
when `needs_replan()` says the cached guide is stale (goal, map generation or
replan period).

### 3a. Shared clearance boundary

All three planners and TEB use `map::clearance()`. It returns the signed
distance from a map-frame point to the nearest blocked cell (Occupied or
Unknown) or to the grid edge, minus half a cell diagonal. A segment is clear
when every cell visited by a Bresenham walk has `clearance >= ROBOT_RADIUS_M`.
There is one physical boundary: the rear-axle-centred `ROBOT_RADIUS_M` circle.
The old fixed seven-cell search disk is gone.

```text
function line_is_clear(from_cell, to_cell, radius):
    for each cell (x, y) on the Bresenham line from_cell -> to_cell:
        if clearance(cell_centre(x, y), radius) < radius:
            return false
    return true
```

### 3b. A*: grid search

```text
function plan_astar(start, goal, composed_grid):
    if start or goal is outside the grid, or the planner inputs are invalid: return Fault
    if start or goal is Occupied: return Fault
    if start or goal is Unknown: return NoPath
    open list = min-heap keyed on f = g + h, ties broken by lower cell index
    g[start] = 0
    expansions = 0
    while open list is not empty:
        cur = pop lowest f
        if cur == goal: reconstruct path via parent links; break
        expansions += 1
        if expansions == max_expansions: return PlannerTimeout
        mark cur closed
        for each of 8 neighbours n of cur:
            skip if n is closed
            skip if line_is_clear(cur, n, ROBOT_RADIUS_M) is false
            tentative = g[cur] + (1 if orthogonal, sqrt(2) if diagonal)
            if tentative < g[n]:
                parent[n] = cur; g[n] = tentative; push or decrease-key n
    if goal was not reached: return NoPath
```

- The heuristic is octile distance: the larger axis difference plus `(sqrt(2) − 1)` times the smaller one. It is consistent for these move costs, so the first time the goal is popped gives a shortest path.
- Every edge is checked with the shared `line_is_clear` boundary above. A* therefore uses the same physical radius as TEB, with no separate cell-disk approximation.
- `ROBOT_RADIUS_M` is configured as `PH-0018` (0.30 m). The expansion budget is `GlobalPlannerConfig::max_expansions`, which replaces the wall-clock timeout (`PH-0006`, dropped).

### 3c. Line-of-sight smoothing (A*, D*-Lite and RRT)

The grid path is a chain of cells. Smoothing shortens it with greedy string pulling.

```text
function smooth_chain(chain):
    kept = [chain[0]]
    cur = 0
    while cur is not the last cell:
        next = cur + 1                       # always a valid grid step
        for j from the last cell down to cur + 2:
            if segment(chain[cur], chain[j]) is no longer than PATH_SMOOTH_MAX_SEGMENT_M
               and line_is_clear(chain[cur], chain[j], clearance radius):
                next = j
                break                        # farthest clear jump wins
        append chain[next] to kept
        cur = next
    return kept
```

- `line_is_clear` walks the segment cell by cell (Bresenham) and requires `clearance >= ROBOT_RADIUS_M + PATH_SMOOTH_MARGIN_M` at every visited cell. D*-Lite and RRT call this same smoother, so their guides have the same clearance boundary.
- `PATH_SMOOTH_MAX_SEGMENT_M` (`PH-0022`, 0.5 m) caps the length of one jump. It must be at least one cell diagonal, or the smoother cannot take a single grid step.
- `PATH_SMOOTH_MARGIN_M` (`PH-0021`, 0.0 m) is the extra clearance. `0` is valid (footprint only).

The result is a guide polyline. It is a topological guide, not a trajectory. [FROM: AGENTS.md §2]

### 3d. D*-Lite

`DStarWorkspace` keeps `g`, `rhs`, priority keys, heap membership, a previous
map snapshot and accumulated start-motion cost `km`. Arrays are supplied by the
caller and never allocated by `plan()`.

```text
function plan_dstar(start, goal, grid):
    validate config, grid, start/goal cells and clearance
    if grid geometry or goal changed:
        initialize every g = rhs = infinity
        rhs[goal] = 0; queue = {goal}; km = 0
        previous_map = grid
    else:
        km += octile(old_start, new_start)
        for each cell whose occupancy changed from previous_map:
            mark a square of radius ceil((ROBOT_RADIUS_M + half_diag) / res)
        for each dirty cell once:
            recompute rhs from successor g + step with line_is_clear
            push, decrease-key or remove that cell
        previous_map = grid
    compute_shortest_path up to max_expansions
    follow finite-g successors from start to goal:
        minimise step + g, then smaller octile distance to the goal
        break any remaining tie with the lower cell index
    smooth the cell chain with the shared clearance
    return Valid, or NoPath / PlannerTimeout
```

- `km` keeps the heuristic consistent when the robot moves without a new map.
  Moving the start never reinitializes the cost field.
- The goal is the search root. A goal change reinitializes because the old
  cost field is rooted at the old goal.
- `compute_shortest_path` stops when the start is consistent or the expansion
  budget is exhausted. A timeout preserves the queue and `g`/`rhs` so the next
  call continues; it is reported, not hidden.
- A blocked start or goal, insufficient start/goal clearance, or an
  unreachable start returns `NoPath`. Extraction can also return `NoPath` if no
  finite successor has a clear edge.
- The extraction tie-break deliberately includes goal progress. Lower-index
  ties alone selected a path that stayed too low at the wall corner and TEB
  rejected the final rollout. Goal progress first reproduced A*'s lower route;
  the lower index remains the final deterministic tie-break.
- D*-Lite smoothing allows two extra cell steps over `PH-0022` because the
  extracted chain is expressed at cell centres. This is a D*-Lite-only path
  shaping allowance, tagged `[ASSUMED]`; the clearance check itself still uses
  exactly `ROBOT_RADIUS_M`, and A* keeps the unchanged 0.5 m cap.

### 3e. RRT

`RrtWorkspace` holds bounded `nodes`, `parents` and `path` arrays. Runtime
values come from `GlobalPlannerConfig`: `max_nodes`, `rrt_step_size`,
`rrt_goal_tolerance`, `rrt_goal_bias` and `rrt_seed`.

```text
function plan_rrt(start, goal, grid):
    validate config, workspace, start/goal cells and clearance
    if line_is_clear(start, goal, ROBOT_RADIUS_M):
        return straight guide [start, goal]
    tree = {start}; rng_state = rrt_seed
    reserve one tree slot for the exact goal
    for attempt in 1 .. max_nodes-2:
        sample = goal with probability rrt_goal_bias, else uniform over grid
        nearest = linear scan of the tree
        next = nearest + min(step_size, distance) toward sample
        if line_is_clear(nearest, next, ROBOT_RADIUS_M):
            append next with parent nearest
            if distance(next, goal) <= goal_tolerance:
                keep the cheapest goal-reaching node found so far
    if no goal-reaching node: return PlannerTimeout
    append exact goal when that final edge is clear
    pull from the goal back toward the root, keeping the farthest ancestor
        visible from the current anchor
    reverse the kept chain; deduplicate cells; smooth with shared clearance
    return Valid
```

- Nearest-neighbour search is linear over the bounded tree; there is no k-d
  tree and no allocation.
- xorshift32 supplies deterministic samples and is reset to `rrt_seed` on each
  call, so the same inputs produce the same guide.
- The first goal-reaching node is not accepted blindly. Sampling continues to
  the node budget and retains the lowest-cost chain that reached the tolerance.
- A full sampling budget without a goal-reaching node returns `PlannerTimeout`.
  It does not restart with a larger tree or fall back to A*.
- The backward line-of-sight pull removes random intermediate tree nodes
  before the cell-chain smoother sees them. The straight start-to-goal check
  short-circuits an open map.

## 4. TEB local planner

Source: `src/teb_local_planner.cpp`. [FROM: src/teb_local_planner.cpp header]

Every per-call buffer (decision vector, gradient, probe, candidate and trajectory) is caller-owned through `TebWorkspace`. `solve()` constructs no containers and performs no heap allocation. A missing, too-small or aliased buffer returns `Fault`. [FROM: user approval, Q2]

The optimizer is **projected finite-difference descent**. It is not an analytic gradient method. Each gradient component comes from a forward difference of the cost.

### 4a. Decision variables

- `N = horizon_steps`, `dt = step_dt`.
- `x` holds `N` speeds followed by `N` steering angles, total `2N`.
- Initial guess: current `forward_speed` and `steering_angle`, held constant over the horizon, then projected.

### 4b. Rollout (bicycle model)

```text
function rollout(pose0, x):
    poses[0] = pose0
    for k in 0 .. N-1:
        v = x[k]; delta = x[N + k]
        poses[k+1].x     = poses[k].x + v * cos(theta_k) * dt
        poses[k+1].y     = poses[k].y + v * sin(theta_k) * dt
        poses[k+1].theta = wrap(theta_k + v * tan(delta) / WHEELBASE_M * dt)
    return poses
```

The steering model is in `method_kinematics.md`.

### 4c. Cost

```text
J(x) = sum over trajectory points k = 1..N of:
           w_guide    * (distance from point to global guide polyline)²
         + w_obstacle * max(0, ROBOT_RADIUS_M - clearance(point))²
     + sum over k = 0..N-1 of:
           w_speed        * (v_k - target_speed)²
         + w_accel        * (v_k - v_{k-1})²
         + w_steer_change * (delta_k - delta_{k-1})²
```

- `clearance(point)` is the distance to the nearest blocked cell (Occupied or Unknown) or to the grid edge. It is conservative: it subtracts a cell half-diagonal from the center distance. Outside the grid it is negative.
- `v_{-1}` is the initial speed and `delta_{-1}` is the initial steering.
- Weights are runtime config (`TebConfig`), with no defaults.

### 4d. Projection onto limits

Every candidate is projected before its cost is evaluated, so the output is always feasible.

```text
function project(x):
    for k in 0 .. N-1:
        lo = max(-MAX_SPEED_MPS, v_{k-1} - MAX_ACCEL_MPS2 * dt)
        hi = min( MAX_SPEED_MPS, v_{k-1} + MAX_ACCEL_MPS2 * dt)
        v_k = clamp(v_k, lo, hi)
    for k in 0 .. N-1:
        lo = max(-MAX_STEER_RAD, delta_{k-1} - MAX_STEER_RATE_RADPS * dt)
        hi = min( MAX_STEER_RAD, delta_{k-1} + MAX_STEER_RATE_RADPS * dt)
        delta_k = clamp(delta_k, lo, hi)
```

The initial speed and steering must already lie inside their limits. If not, `solve()` returns `Fault`.

### 4e. Optimization loop

```text
function solve(x):
    x = project(initial guess)
    J = cost(x)
    eta = step_size
    repeat for a fixed number of iterations:
        for each component i of x:
            grad[i] = (cost(x + fd_epsilon * e_i) - J) / fd_epsilon     # forward difference
        candidate = project(x - eta * grad)
        if cost(candidate) < J:
            x = candidate; J = cost(candidate)      # accept
        else:
            eta = eta / 2                           # reject and shrink the step
    return x
```

- The iteration count is fixed (`TebConfig::iterations`), not run to convergence.
- Each accepted step lowers the cost. A rejected step halves `eta`.
- Because every candidate is projected, the limits hold at every iteration, not only at the end.

### 4f. Final feasibility check

```text
function final_check(x):
    poses = rollout(x)
    for each trajectory point k = 1..N:
        if clearance(point_k) < ROBOT_RADIUS_M:
            return NoPath
    return Valid with first speed x[0] and first steering x[N]
```

The first speed and first steering are the commands for the current cycle. The output converter turns them into the next pose.
