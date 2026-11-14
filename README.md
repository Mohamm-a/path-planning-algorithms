# core

> **Status:** Core architecture complete and verified. Ready for physical robot integration.
> All 17 physical values are approved and compiled in; see `UNKNOWNS.md`.

A header-only-facade C++17 core for an Ackermann-constrained robot. The core is
a deterministic blackbox with one `step()` call per control cycle. It has no
hardware, ROS, logging, clock, or file I/O dependencies. All storage is
caller-owned.

Pipeline per `step()`: odometry → map manager → global planner (on demand) →
TEB local planner → output converter → status.

The global planner is selectable per snapshot: A*, D*-Lite or RRT. All three
return the same topological guide format and use the same `ROBOT_RADIUS_M`
clearance boundary as TEB.

## Build

Requires CMake 3.16+ and a C++17 compiler. Every library is built with
`-Wall -Wextra -Wpedantic -Werror`.

```bash
cmake -S . -B build
cmake --build build
cmake --install build --prefix <install-prefix>
```

## Using the core from another CMake project

After `cmake --install`, an external project uses the exported package:

```cmake
find_package(robot_core REQUIRED)
target_link_libraries(my_robot PRIVATE robot_core::core_facade)
```

Configure the consumer with `-DCMAKE_PREFIX_PATH=<install-prefix>`. Headers are
installed under `<install-prefix>/include` and the package config under
`<install-prefix>/lib/cmake/robot_core`. The installed package has no version
file yet.

Targets (exported as `robot_core::<name>`):

| Target | Contents |
| --- | --- |
| `core` | Interface target: include path and C++17 requirement only |
| `core_facade` | `Core` facade (`src/core.cpp`) and links every stage library |
| `core_odometry`, `core_map`, `core_global_planner`, `core_teb_local_planner`, `core_output_converter` | Stage implementations |
| `core_compile_check` | Compile-only check that includes every header. Not a test |

Link your application against `robot_core::core_facade`. Without CMake, add
`include/` to the include path and link every `libcore_*.a` in `lib/`. The
archives depend on each other.

## Configured physical values

Every physical value is a compile-time value in `include/core/placeholders.hpp`
(named `WHEELBASE_M`, `MAX_STEER_RAD`, and so on). The approved configuration is
compiled in and recorded as `RESOLVED` in `UNKNOWNS.md`.

The installed copy of `placeholders.hpp` ships with those values. Override one
without editing the header by passing it on the compiler command line, for
example `-DWHEELBASE_M=<value>`. Each define is guarded by `#ifndef`.
The guard in `initialize()` rejects out-of-range overrides; it cannot check that
an override is physically correct.

### Setting overrides from a parent CMake project

When the core is built as part of a parent project with `add_subdirectory`,
set the overrides on the `core` interface target. Every stage library links to
`core` publicly, so all of them receive the same values:

```cmake
add_subdirectory(path/to/core)
target_compile_definitions(core INTERFACE
    WHEELBASE_M=<value>
    MAX_STEER_RAD=<value>)
```

> **Note:** A define set only on your own application target does not reach the
> core's compiled archives. Set overrides on `core` (or on the core build
> itself) so every translation unit uses the same value. The installed package
> has no CMake hook for this, so pass overrides when building the core.

`initialize()` validates all 17 configured values and returns `Status::Fault`
if any is outside its physical domain. `Core::step()` refuses to run before a
`Valid` `initialize()`. The approved values are compiled in; override them only
through a reviewed change.

## Instantiation

`Core` takes configuration by value and buffers by non-owning view. The views
must stay valid for the lifetime of the `Core`. `Core` is not copyable.

| Argument | Type | Purpose |
| --- | --- | --- |
| `teb_config` | `teb::TebConfig` | TEB tuning. Must be supplied in full |
| `planner_config` | `global_planner::GlobalPlannerConfig` | A*/D*-Lite expansion limit and RRT limits |
| `fixed_borders` | `map::GridMap` | Static map layer (read-only) |
| `slam_overlay` | `map::MutableGridMap` | SLAM layer. Updated in place each cycle |
| `inflation` | `map::MutableGridMap` | Inflation layer, written each cycle |
| `composed` | `map::MutableGridMap` | Costmap the planners read, written each cycle |
| `teb_workspace` | `teb::TebWorkspace` | TEB scratch: four arrays of `2 * horizon_steps` reals, plus `horizon_steps + 1` poses |
| `workspace` | `global_planner::PlannerWorkspace` | Caller-owned A*, D*-Lite and RRT scratch |
| `waypoints` | `global_planner::WaypointBuffer` | Output storage for the global guide |

The four grids must share the same `origin`, `width` and `height`. They must
not alias each other's storage. `map_manager` returns `Status::Fault`
otherwise.

## Buffer sizing

Let `W` and `H` be the grid extents in cells. `N = W * H`. The caller owns every
buffer; `plan()` and `step()` perform no heap allocation.

| Buffer | Requirement | Bytes |
| --- | --- | --- |
| Each grid (`fixed`, `slam`, `inflation`, `composed`) | `N` cells | `N * sizeof(map::CellState)` (1 byte each) |
| `PlannerWorkspace` A* arrays (`g_cost`, `parent`, `closed`, `heap`, `heap_slot`) | all five non-null, `capacity >= N` | `N * (sizeof(real) + 3 * sizeof(std::size_t) + sizeof(bool))` |
| `PlannerWorkspace::dstar` | all eight arrays non-null, `capacity >= N`; A* `parent` and `heap` are reused for the dirty list and extracted chain | `N * (4 * sizeof(real) + 2 * sizeof(std::size_t) + 2)` |
| `PlannerWorkspace::rrt` | `capacity >= max_nodes`; the A* `heap` must cover `N` for the smoothed cell chain | `max_nodes * (sizeof(Point2D) + 2 * sizeof(std::size_t))` |
| `WaypointBuffer` | `capacity >= N` | `N * sizeof(Point2D)` |
| `teb::TebWorkspace` | four arrays of `2 * horizon_steps` reals, `horizon_steps + 1` poses | `4 * 2 * horizon_steps * sizeof(real) + (horizon_steps + 1) * sizeof(Pose2D)` |

With `real = float`, the A* portion of `PlannerWorkspace` is 17 B per cell on a
32-bit target and 29 B per cell on a 64-bit host. The D*-Lite and RRT arrays are
only needed for those planners; filling `PlannerWorkspace` with zeroes and
leaving them empty is valid for A*. Each of the four grid layers adds one byte
per cell. [FROM: the struct definitions]

`N` is a worst-case bound. A planner path visits each cell at most once. A
smaller workspace, RRT array or waypoint capacity returns `Status::Fault` and
writes nothing. A degree of freedom in `GlobalPlannerConfig` is validated by
the selected planner only.

## Contract

- `InputSnapshot` is immutable per cycle. Units are m, rad, s, rad/s. The
  encoder is RPM, the one documented exception.
- `encoder_rpm` is **effective rear-wheel RPM**, not raw motor RPM. The lidcar
  encoder is mounted before the differential; the adapter must divide its raw
  reading by the combined gearbox and differential ratio before filling the
  snapshot. The core applies the wheel-radius conversion directly.
- Each sensor has its own sample time (`pose_timestamp`, `lidar_timestamp`,
  `encoder_timestamp`, `gyro_timestamp`, `steering_timestamp`), in the same
  epoch as `timestamp`.
- `snapshot.is_new_sweep` marks a LiDAR sweep captured since the previous
  cycle. When it is false, the lidar timestamp is not age-checked and the map
  is not updated. When it is true, the timestamp must be fresh.
- `teb_config.step_dt` must equal `snapshot.dt`. Otherwise `step()` returns
  `Status::Fault`.
- `Output` always carries `dt`, `timestamp`, `frame_id` (`Frame::Map`) and
  `status`. If any stage is not `Valid`, `next_pose` is NaN.
- `Output` also carries `v_cmd` (m/s) and `delta_cmd` (rad, positive left) for
  the first horizon step. Both are NaN when `status` is not `Valid`.
- The core is deterministic. It reads no wall clock, and it holds no hidden
  state beyond the `Core` object.

## Coordinate frame

- The `base_link` frame origin is located at the center of the rear axle on the
  ground plane. The +x axis points forward, and the +y axis points left.
- Sensor mounts (`LIDAR_MOUNT_X_M`, `LIDAR_MOUNT_Y_M`) are measured from this
  origin.

## Status

| Status | Meaning |
| --- | --- |
| `Valid` | Stage output is usable |
| `NoPath` | No collision-free path from the global guide or costmap |
| `StaleSensor` | A sensor age exceeds `SENSOR_STALE_TIMEOUT_S` (PH-0005) |
| `PlannerTimeout` | A*/D*-Lite exceeded `max_expansions`, or RRT exhausted `max_nodes` without reaching the goal tolerance |
| `Fault` | Invalid input, uninitialized core, placeholder guard, or undersized buffer. Also a negative or non-finite sensor age |

There are no silent fallbacks. A non-`Valid` status is always reported.

## Known limitations

- **Corner stall.** The TEB local planner may stall (returning `Valid` status
  with zero velocity) at tight corners if the physical turning radius is too
  large for the maneuver. [ASSUMED: turning radius cause, not verified in code;
  observed stall in `sim/README.md`]. While the speed bounds technically allow
  negative values (reversing), the current gradient descent implementation does
  not explicitly optimize for reverse maneuvers to escape local minima.
  [FROM: `src/teb_local_planner.cpp`, fixed-iteration finite-difference projected
  descent in `solve()`]
- **D*-Lite path-shape allowance.** D*-Lite smoothing permits two extra cell
  steps over `PH-0022` for its cell-centre extraction. The clearance check is
  unchanged, but this is a D*-Lite-specific shaping assumption. See
  `sim/README.md` for the passing runs.
- **No version file.** The installed package has no version file yet.
- **Physical values.** All 17 values are resolved and compiled in. Overriding
  them requires a rebuild; `initialize()` rejects out-of-range values.

## Example

The example below compiles against this repository. It uses only stationary
inputs and the approved configuration, so it returns `Valid`.

```cpp
#include <cstddef>
#include <cstdio>
#include <memory>
#include <vector>

#include "core/core.hpp"
#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/teb_local_planner.hpp"

int main() {
    using namespace core;

    constexpr std::size_t N = 200;       // grid is N x N cells; 10 m at 0.05 m/cell
    constexpr double kOrigin = -5.0;     // m; map-frame position of cell (0, 0)
    constexpr double kDt = 0.2;          // s
    constexpr std::size_t kHorizon = 10; // TEB horizon steps

    // Caller-owned storage, sized per "Buffer sizing".
    std::vector<map::CellState> fixed(N * N, map::CellState::Free);
    std::vector<map::CellState> slam(N * N, map::CellState::Unknown);
    std::vector<map::CellState> inflation(N * N, map::CellState::Unknown);
    std::vector<map::CellState> composed(N * N, map::CellState::Unknown);
    std::vector<real> g_cost(N * N);
    std::vector<std::size_t> parent(N * N);
    std::unique_ptr<bool[]> closed(new bool[N * N]());
    std::vector<std::size_t> heap(N * N);
    std::vector<std::size_t> heap_slot(N * N);
    std::vector<Point2D> waypoints(N * N);
    std::vector<real> teb_x(2 * kHorizon);
    std::vector<real> teb_grad(2 * kHorizon);
    std::vector<real> teb_probe(2 * kHorizon);
    std::vector<real> teb_cand(2 * kHorizon);
    std::vector<Pose2D> teb_trajectory(kHorizon + 1);

    teb::TebConfig teb_cfg{};
    teb_cfg.horizon_steps = kHorizon;
    teb_cfg.step_dt = kDt;
    teb_cfg.target_speed = 0.5;  // m/s; must not exceed MAX_SPEED_MPS
    teb_cfg.iterations = 100;
    teb_cfg.step_size = 0.1;
    teb_cfg.fd_epsilon = 1e-6;
    teb_cfg.w_guide = 1.0;
    teb_cfg.w_obstacle = 10.0;
    teb_cfg.w_speed = 0.1;
    teb_cfg.w_accel = 0.1;
    teb_cfg.w_steer_change = 0.1;

    global_planner::GlobalPlannerConfig planner_cfg{};
    planner_cfg.max_expansions = 100000;

    // This example allocates only the A* scratch. To use D*-Lite, also fill
    // PlannerWorkspace::dstar; to use RRT, set max_nodes/rrt_step_size/
    // rrt_goal_tolerance/rrt_goal_bias/rrt_seed and fill PlannerWorkspace::rrt.
    // See "Buffer sizing" for the array requirements.

    Core core(teb_cfg, planner_cfg,
              map::GridMap{Point2D{kOrigin, kOrigin}, N, N, fixed.data()},
              map::MutableGridMap{Point2D{kOrigin, kOrigin}, N, N, slam.data()},
              map::MutableGridMap{Point2D{kOrigin, kOrigin}, N, N, inflation.data()},
              map::MutableGridMap{Point2D{kOrigin, kOrigin}, N, N, composed.data()},
              global_planner::PlannerWorkspace{N * N, g_cost.data(), parent.data(),
                                               closed.get(), heap.data(), heap_slot.data()},
              teb::TebWorkspace{teb_x.size(), teb_x.data(), teb_grad.data(),
                                teb_probe.data(), teb_cand.data(),
                                teb_trajectory.size(), teb_trajectory.data()},
              global_planner::WaypointBuffer{waypoints.data(), waypoints.size()});

    if (core.initialize() != Status::Valid) {
        std::printf("refusing to run: core configuration is invalid\n");
        return 1;
    }

    // Stationary robot, no LiDAR returns, all samples taken at the snapshot time.
    InputSnapshot s{};
    s.pose = Pose2D{0.0, 0.0, 0.0};
    s.goal = Point2D{3.0, 0.0};
    s.lidar = LidarView{nullptr, 0};
    s.is_new_sweep = true;
    s.encoder_rpm = 0.0;
    s.steering_angle = 0.0;
    s.gyro_yaw_rate = 0.0;
    s.global_planner = GlobalPlannerKind::AStar;
    s.timestamp = 1.0;
    s.dt = kDt;
    s.pose_timestamp = s.timestamp;
    s.lidar_timestamp = s.timestamp;
    s.encoder_timestamp = s.timestamp;
    s.gyro_timestamp = s.timestamp;
    s.steering_timestamp = s.timestamp;

    const Output out = core.step(s);
    std::printf("status=%d next_pose=(%.3f, %.3f, %.3f) dt=%.3f\n",
                static_cast<int>(out.status), out.next_pose.x, out.next_pose.y,
                out.next_pose.theta, out.dt);
    return out.status == Status::Valid ? 0 : 2;
}
```
