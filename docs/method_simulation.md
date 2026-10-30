# Simulation

## Purpose

`sim/wall_sim.cpp` is an end-to-end test of `core::Core`. It runs the core in a closed loop against a simulated plant: a robot that moves in a 2D world with one wall. It is opt-in and is not part of the core. It is enabled by the CMake option `ROBOT_CORE_BUILD_SIM`. [FROM: sim/wall_sim.cpp header, sim/README.md]

## Scenario

The scenario uses these named constants. The values are not written here.

| Name | Meaning |
|---|---|
| `kStart` | Starting position of the robot. |
| `kGoal` | Goal position. |
| `kWallA`, `kWallB` | End points of one straight wall segment (the wall is sensed by the virtual LiDAR; the fixed map layer is all Free). |
| `kGoalExtent_m` | Goal coordinate used to size the grid. |
| `kMapMargin_m` | Margin beyond the goal, so inflation and clearance fit inside the grid. |
| `kN` | Grid cells per side, computed from `MAP_RESOLUTION_M` and the goal. |
| `kDt` | Cycle period, which must equal the core's `dt`. |
| `kBeams`, `kAlphaMin`, `kAlphaMax` | Number of LiDAR beams and their angular span. |
| `kGoalTol` | Distance to the goal that counts as reached. |
| `kClearanceMargin` | Required clearance beyond `ROBOT_RADIUS_M`. |
| `kMaxCycles` | Upper bound on control cycles. |

The grid size is derived from `MAP_RESOLUTION_M`, so the goal and margin stay inside the map at any resolution. [FROM: sim/wall_sim.cpp]

## Test loop

Each cycle does the following, in order:

```text
initialize the core (fail with RESULT FAIL initialize if the guard refuses)
pose = kStart, sensors = at rest

for cycle k from 0 to kMaxCycles - 1:
    check the success and failure criteria on the current pose
    if a criterion is met: stop and report RESULT

    1. Generate virtual LiDAR:
         for each beam i in the sweep:
             cast a ray from pose at angle (theta + alpha_i) against the wall
             return the hit distance, or a value above LIDAR_MAX_RANGE_M if no hit
    2. Build InputSnapshot:
         pose, goal, lidar view, the sensor values from the last motion,
         all five sample times = t = k * kDt, global planner = AStar
    3. out = core.step(snapshot)
         if out.status is not Valid: report RESULT FAIL status and stop
    4. Move the virtual robot:
         the plant sets pose = out.next_pose exactly (no noise)
    5. Derive the sensor values for the next cycle from the applied motion:
         omega = wrapped heading change / kDt
         v     = forward component of the displacement / kDt
         rpm   = v converted with WHEEL_RADIUS_M
         delta = atan(WHEELBASE_M * omega / v), or 0 when v is near zero
```

- The plant is ideal. The robot tracks `next_pose` exactly each cycle. [ASSUMED: no actuator or sensor noise; sim/README.md]
- Sensor values are derived from the applied motion, so the core's odometry reproduces the simulated pose. The steering value is derived from the same bicycle relation as `method_kinematics.md`.
- The LiDAR uses the same mount convention as the core. [FROM: sim/wall_sim.cpp]

## Success criteria

The run ends with one of these results, checked in this order each cycle:

| Result | Condition | Exit |
|---|---|---|
| `RESULT FAIL wall clearance` | Distance to the wall minus `ROBOT_RADIUS_M` is below `kClearanceMargin`. | Non-zero |
| `RESULT SUCCESS` | Distance to the goal is at most `kGoalTol`. | Zero |
| `RESULT FAIL status` | The core returns a status other than `Valid`. | Non-zero |
| `RESULT FAIL max cycles` | `kMaxCycles` is reached without reaching the goal. | Non-zero |
| `RESULT FAIL initialize` | The placeholder guard refuses to run. | Non-zero |

The wall check runs before the goal check. A pose that is both clear of the goal and too close to the wall is a failure. [FROM: sim/wall_sim.cpp]

> **Note:** The sim does not supply placeholder values. `sim/README.md` says they must come from the build's `CMAKE_CXX_FLAGS`. If a placeholder is unset, the guard fails the run. [FROM: sim/README.md]
