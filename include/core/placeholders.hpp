// Approved physical configuration for the core.
//
// Every value below is the configuration approved for the target robot
// [FROM: user decision, Phase B]. Each define remains overridable with -D or by
// defining it before this header. initialize() in core.hpp rejects values
// outside the physical domain, so an invalid override still fails loudly.
//
// The registry for these values is UNKNOWNS.md; every row is RESOLVED.

#pragma once

// PH-0001: distance between front and rear axles. Unit: m.
// Affects: TEB local planner.
#ifndef WHEELBASE_M
#define WHEELBASE_M 0.245
#endif

// PH-0002: maximum steering angle magnitude. Unit: rad; pi/4.
// Affects: TEB local planner.
#ifndef MAX_STEER_RAD
#define MAX_STEER_RAD (3.14159265358979323846 / 4.0)
#endif

// PH-0004: maximum forward speed. Unit: m/s.
// Affects: TEB local planner (speed box and target-speed check) and no-goal
// guide length.
#ifndef MAX_SPEED_MPS
#define MAX_SPEED_MPS 0.5
#endif

// PH-0005: age beyond which a sensor reading is stale. Unit: s.
// Affects: status (StaleSensor). Sweeps are stamped at completion time.
#ifndef SENSOR_STALE_TIMEOUT_S
#define SENSOR_STALE_TIMEOUT_S 1.0
#endif

// PH-0007: obstacle inflation radius. Unit: m. 0.0 means no inflation.
// Affects: map manager (inflation layer).
#ifndef INFLATION_RADIUS_M
#define INFLATION_RADIUS_M 0.05
#endif

// PH-0008: edge length of one map grid cell. Unit: m per cell.
// Affects: map manager (GridMap).
#ifndef MAP_RESOLUTION_M
#define MAP_RESOLUTION_M 0.05
#endif

// PH-0009: effective radius of the rear drive wheel. Unit: m.
// encoder_rpm is the effective rear-wheel rotation rate. The encoder is mounted
// on the motor shaft before the differential, so the adapter must divide raw
// RPM by the combined gearbox and differential ratio [FROM: user correction].
// Affects: odometry (forward speed).
#ifndef WHEEL_RADIUS_M
#define WHEEL_RADIUS_M 0.056
#endif

// PH-0010: LiDAR mount x offset in base_link. Unit: m.
// The LiDAR is 0.275 m forward of the rear axle, on the vehicle centreline.
// Height is not represented in the 2D core [FROM: user correction].
// Affects: map manager (LiDAR frame).
#ifndef LIDAR_MOUNT_X_M
#define LIDAR_MOUNT_X_M 0.275
#endif

// PH-0011: LiDAR mount y offset in base_link. Unit: m.
// The LiDAR has no lateral offset; y = 0.
// Affects: map manager (LiDAR frame).
#ifndef LIDAR_MOUNT_Y_M
#define LIDAR_MOUNT_Y_M 0.0
#endif

// PH-0012: LiDAR mount yaw in base_link. Unit: rad.
// LiDAR +x points forward.
// Affects: map manager (LiDAR frame).
#ifndef LIDAR_MOUNT_THETA_RAD
#define LIDAR_MOUNT_THETA_RAD 0.0
#endif

// PH-0016: maximum LiDAR range. Unit: m; nominal sensor range.
// Affects: map manager (ray length and occupied-endpoint marking).
#ifndef LIDAR_MAX_RANGE_M
#define LIDAR_MAX_RANGE_M 8.0
#endif

// PH-0017: period after which the global planner is re-run even if the goal
// and map are unchanged. Unit: s.
// Affects: global planner manager (needs_replan).
#ifndef GLOBAL_PLANNER_REPLAN_PERIOD_S
#define GLOBAL_PLANNER_REPLAN_PERIOD_S 0.1
#endif

// PH-0018: radius of the circle circumscribing the robot footprint. Unit: m.
// Affects: TEB local planner (obstacle cost and collision check).
#ifndef ROBOT_RADIUS_M
#define ROBOT_RADIUS_M 0.30
#endif

// PH-0019: maximum longitudinal acceleration magnitude. Unit: m/s^2.
// Affects: TEB local planner (acceleration limit).
#ifndef MAX_ACCEL_MPS2
#define MAX_ACCEL_MPS2 1.0
#endif

// PH-0020: maximum steering-angle rate magnitude (servo speed). Unit: rad/s;
// pi/3. Affects: TEB local planner (steering-rate limit).
#ifndef MAX_STEER_RATE_RADPS
#define MAX_STEER_RATE_RADPS (3.14159265358979323846 / 3.0)
#endif

// PH-0021: clearance margin added to ROBOT_RADIUS_M when the global planner
// smooths its path. Unit: m. 0.0 means footprint only.
// Affects: global planner (path smoothing).
#ifndef PATH_SMOOTH_MARGIN_M
#define PATH_SMOOTH_MARGIN_M 0.0
#endif

// PH-0022: maximum length of one smoothed global-path segment. Unit: m.
// Affects: global planner (path smoothing).
#ifndef PATH_SMOOTH_MAX_SEGMENT_M
#define PATH_SMOOTH_MAX_SEGMENT_M 0.5
#endif
