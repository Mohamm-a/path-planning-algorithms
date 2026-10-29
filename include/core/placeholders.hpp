// Placeholder header for the drop-in core.
//
// Every unknown value is a compile-time #define named NAME (for example WHEELBASE_M).
// Each one is tagged [UNKNOWN] at its definition, has a row in UNKNOWNS.md,
// and has a sentinel that cannot be a valid value. initialize() in core.hpp
// refuses (Status::Fault) while any placeholder still holds its sentinel.
//
// Sentinels are chosen per quantity:
//   - 0.0 for quantities where zero is never a valid physical value.
//   - -1.0 for PH-0007, where 0.0 is a valid value (no inflation) and
//     negative values are not [FROM: user answer, Phase A].
//   - NaN for mount poses, where zero is a valid position or angle.
// No real values are supplied. See AGENTS.md section 6.
//
// Each placeholder is overridable: define NAME before including this
// header, or pass -DNAME=<value> on the compiler command line. The
// #ifndef guards below keep the supplied value. The guard in initialize() still
// rejects sentinel values, but it cannot judge whether an override is correct.

#pragma once

#include <limits>

// [UNKNOWN] PH-0001: distance between front and rear axles. Unit: m.
// Affects: TEB local planner. Guard: initialize() -> Fault.
#ifndef WHEELBASE_M
#define WHEELBASE_M 0.0
#endif

// [UNKNOWN] PH-0002: maximum steering angle magnitude. Unit: rad.
// Affects: TEB local planner. Guard: initialize() -> Fault.
#ifndef MAX_STEER_RAD
#define MAX_STEER_RAD 0.0
#endif

// [UNKNOWN] PH-0004: maximum forward speed. Unit: m/s.
// Affects: TEB local planner. Guard: initialize() -> Fault.
#ifndef MAX_SPEED_MPS
#define MAX_SPEED_MPS 0.0
#endif

// [UNKNOWN] PH-0005: age beyond which a sensor reading is stale. Unit: s.
// Affects: status (StaleSensor). Guard: initialize() -> Fault.
#ifndef SENSOR_STALE_TIMEOUT_S
#define SENSOR_STALE_TIMEOUT_S 0.0
#endif

// [UNKNOWN] PH-0007: obstacle inflation radius. Unit: m.
// Sentinel -1.0: 0.0 is a valid value (no inflation); negative is not.
// Affects: map manager (Inflation layer). Guard: initialize() -> Fault while < 0.0.
#ifndef INFLATION_RADIUS_M
#define INFLATION_RADIUS_M -1.0
#endif

// [UNKNOWN] PH-0008: edge length of one map grid cell. Unit: m per cell.
// Affects: map manager (GridMap). Guard: initialize() -> Fault.
#ifndef MAP_RESOLUTION_M
#define MAP_RESOLUTION_M 0.0
#endif

// [UNKNOWN] PH-0009: effective radius of the encoder-measured drive wheel. Unit: m.
// Affects: odometry (forward speed). Guard: initialize() -> Fault; odometry -> Fault.
#ifndef WHEEL_RADIUS_M
#define WHEEL_RADIUS_M 0.0
#endif

// [UNKNOWN] PH-0010: LiDAR mount x offset in base_link. Unit: m.
// Sentinel NaN: zero is a valid position. Affects: map manager (LiDAR frame).
// Guard: initialize() -> Fault while not finite.
#ifndef LIDAR_MOUNT_X_M
#define LIDAR_MOUNT_X_M (std::numeric_limits<double>::quiet_NaN())
#endif

// [UNKNOWN] PH-0011: LiDAR mount y offset in base_link. Unit: m.
// Sentinel NaN: zero is a valid position. Affects: map manager (LiDAR frame).
// Guard: initialize() -> Fault while not finite.
#ifndef LIDAR_MOUNT_Y_M
#define LIDAR_MOUNT_Y_M (std::numeric_limits<double>::quiet_NaN())
#endif

// [UNKNOWN] PH-0012: LiDAR mount yaw in base_link. Unit: rad.
// Sentinel NaN: zero is a valid angle. Affects: map manager (LiDAR frame).
// Guard: initialize() -> Fault while not finite.
#ifndef LIDAR_MOUNT_THETA_RAD
#define LIDAR_MOUNT_THETA_RAD (std::numeric_limits<double>::quiet_NaN())
#endif

// [UNKNOWN] PH-0016: maximum LiDAR range. Unit: m.
// Sentinel 0.0: zero range cannot observe anything. Affects: map manager
// (ray length and occupied-endpoint marking). Guard: initialize() -> Fault.
#ifndef LIDAR_MAX_RANGE_M
#define LIDAR_MAX_RANGE_M 0.0
#endif

// [UNKNOWN] PH-0017: period after which the global planner is re-run even if
// the goal and map are unchanged. Unit: s. Sentinel 0.0: zero would replan
// every cycle, which is not a chosen policy. Affects: global planner manager
// (needs_replan). Guard: initialize() -> Fault while not > 0.0.
#ifndef GLOBAL_PLANNER_REPLAN_PERIOD_S
#define GLOBAL_PLANNER_REPLAN_PERIOD_S 0.0
#endif

// [UNKNOWN] PH-0018: radius of the circle circumscribing the robot footprint,
// used for obstacle cost. Unit: m. Sentinel 0.0: a zero footprint radius is
// not physical. Affects: TEB local planner (obstacle cost). Guard:
// initialize() -> Fault while not > 0.0.
#ifndef ROBOT_RADIUS_M
#define ROBOT_RADIUS_M 0.0
#endif

// [UNKNOWN] PH-0019: maximum longitudinal acceleration magnitude. Unit: m/s^2.
// Sentinel 0.0: zero would forbid any change of speed. Affects: TEB local
// planner (acceleration limit). Guard: initialize() -> Fault while not > 0.0.
#ifndef MAX_ACCEL_MPS2
#define MAX_ACCEL_MPS2 0.0
#endif

// [UNKNOWN] PH-0021: clearance margin added to ROBOT_RADIUS_M when
// the global planner smooths its path. Unit: m. Sentinel -1.0: 0.0 is valid
// (footprint only); negative is not. Affects: global planner (path smoothing).
// Guard: AStarPlanner::plan() returns Status::Fault while < 0.0.
#ifndef PATH_SMOOTH_MARGIN_M
#define PATH_SMOOTH_MARGIN_M -1.0
#endif

// [UNKNOWN] PH-0022: maximum length of one smoothed global-path segment, so a
// single greedy jump cannot reach far past the robot. Unit: m. Sentinel 0.0: a
// zero cap allows no jump. Must be >= one cell diagonal, or the smoother cannot
// honour it. Affects: global planner (path smoothing). Guard: AStarPlanner::plan()
// returns Status::Fault while not finite, <= 0.0, or < one cell diagonal.
#ifndef PATH_SMOOTH_MAX_SEGMENT_M
#define PATH_SMOOTH_MAX_SEGMENT_M 0.0
#endif

// [UNKNOWN] PH-0020: maximum steering-angle rate magnitude (servo speed). Unit:
// rad/s. Sentinel 0.0: zero would freeze the steering. Affects: TEB local
// planner (steering-rate limit). Guard: initialize() -> Fault while not > 0.0.
#ifndef MAX_STEER_RATE_RADPS
#define MAX_STEER_RATE_RADPS 0.0
#endif
