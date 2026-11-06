// Public boundary of the drop-in core. This is the only header a host
// system includes. See AGENTS.md sections 2, 3, 5 and 6.
//
// Units: meters, radians, seconds, rad/s [FROM: AGENTS.md section 5].
// Documented exception: encoder_rpm is RPM [FROM: AGENTS.md section 2;
// exception approved by user during Phase A].
// Angles are normalized to (-pi, pi] at the boundary (normalization is
// implemented in the facade, not in this header).
// Frames: map, odom, base_link, lidar, imu [FROM: AGENTS.md section 5].

#pragma once

#include <cmath>
#include <cstddef>
#include <memory>

#include "core/placeholders.hpp"

namespace core {

// Scalar type for every spatial and kinematic quantity in the core.
// Time stays double: a float timestamp loses precision at large epochs
// [FROM: user approval, Q1].
using real = float;

// Pose in a named frame. x, y in m; theta in rad, (-pi, pi].
struct Pose2D {
    real x;      // m
    real y;      // m
    real theta;  // rad
};

// Position in a named frame. x, y in m.
struct Point2D {
    real x;  // m
    real y;  // m
};

// One LiDAR return. r is the beam length in m; alpha is the beam angle in rad.
struct LidarPoint {
    real r;      // m
    real alpha;  // rad
};

// Non-owning view over LiDAR returns (pointer plus count).
// The caller owns the storage. No allocation occurs inside the core.
struct LidarView {
    const LidarPoint* data;  // may be null only when count is 0
    std::size_t count;       // number of valid entries at data
};

// Validated global planner in effect [FROM: AGENTS.md section 2].
enum class GlobalPlannerKind {
    AStar,
    DStarLite,
    RRT,
};

// Frames [FROM: AGENTS.md section 5].
enum class Frame {
    Map,
    Odom,
    BaseLink,
    Lidar,
    Imu,
};

// Status [FROM: AGENTS.md section 2]. No silent fallbacks.
enum class Status {
    Valid,
    NoPath,
    StaleSensor,
    PlannerTimeout,
    Fault,
};

// One immutable control-cycle snapshot [FROM: AGENTS.md section 2].
struct InputSnapshot {
    Pose2D pose;                    // m, m, rad; map frame (already localized)
    Point2D goal;                   // m, m; map frame
    LidarView lidar;                // beam returns (r, alpha)
    // True when `lidar` is a new sweep since the previous cycle. When false,
    // the stale check ignores lidar_timestamp and step() reuses the existing
    // map without re-marking beams [FROM: user approval, Q3].
    bool is_new_sweep;
    real encoder_rpm;             // RPM (documented exception to section 5)
    real steering_angle;          // rad; positive = left turn
    real gyro_yaw_rate;           // rad/s; positive = CCW
    GlobalPlannerKind global_planner;
    double timestamp;               // s; caller-defined epoch
    double dt;                      // s; supplied by caller
    // Per-sensor sample times in s, same epoch as timestamp. Each is checked
    // against timestamp for staleness (PH-0005). A negative or non-finite age
    // (timestamp - sample time) yields Status::Fault; an age above the
    // threshold yields Status::StaleSensor.
    double pose_timestamp;          // s
    double lidar_timestamp;         // s
    double encoder_timestamp;       // s
    double gyro_timestamp;          // s
    double steering_timestamp;      // s
};

// Result of one step() call [FROM: AGENTS.md section 2].
// Numeric fields have no defaults: they are set by the producer, never zero.
struct Output {
    Pose2D next_pose;   // m, m, rad; map frame
    real v_cmd;         // m/s; speed for the first horizon step
    real delta_cmd;     // rad; steering for the first horizon step; + = left
    double dt;          // s
    double timestamp;   // s; caller-defined epoch
    Frame frame_id;     // Frame::Map
    Status status = Status::Fault;  // no-silent-fallback default
    // v_cmd and delta_cmd are NaN whenever status is not Valid, matching
    // next_pose [FROM: user approval, Q5].
};

// Guard [UNKNOWN: PH-0001 to PH-0022, see placeholders.hpp]. Refuses to
// operate while any placeholder still holds its sentinel. Returns
// Status::Valid only when all are replaced. Sentinels are checked per quantity:
//   - 0.0 sentinel: must be > 0.0 (zero is never valid).
//   - PH-0007 sentinel -1.0: must be >= 0.0 (zero means no inflation).
//   - PH-0021 sentinel -1.0: must be >= 0.0 (zero means footprint only).
//   - NaN sentinel (mount poses): must be finite.
// Fault is returned, never a silent fallback [ASSUMED: physical reasoning].
inline Status initialize() {
    const bool placeholders_replaced =
        WHEELBASE_M > 0.0 &&
        MAX_STEER_RAD > 0.0 &&
        MAX_SPEED_MPS > 0.0 &&
        SENSOR_STALE_TIMEOUT_S > 0.0 &&
        INFLATION_RADIUS_M >= 0.0 &&
        MAP_RESOLUTION_M > 0.0 &&
        WHEEL_RADIUS_M > 0.0 &&
        std::isfinite(LIDAR_MOUNT_X_M) &&
        std::isfinite(LIDAR_MOUNT_Y_M) &&
        std::isfinite(LIDAR_MOUNT_THETA_RAD) &&
        LIDAR_MAX_RANGE_M > 0.0 &&
        GLOBAL_PLANNER_REPLAN_PERIOD_S > 0.0 &&
        ROBOT_RADIUS_M > 0.0 &&
        MAX_ACCEL_MPS2 > 0.0 &&
        MAX_STEER_RATE_RADPS > 0.0 &&
        PATH_SMOOTH_MARGIN_M >= 0.0 &&
        PATH_SMOOTH_MAX_SEGMENT_M > 0.0;
    return placeholders_replaced ? Status::Valid : Status::Fault;
}

// Stage types used by the facade. Forward declarations only: the full
// definitions are in the stage headers, which include this header. This keeps
// core.hpp free of circular includes.
namespace map {
struct GridMap;
struct MutableGridMap;
}  // namespace map
namespace global_planner {
struct GlobalPlannerConfig;
struct PlannerWorkspace;
struct WaypointBuffer;
}  // namespace global_planner
namespace teb {
struct TebConfig;
struct TebWorkspace;
}  // namespace teb

// Facade. The single entry point of the core. One instance per host system.
// The instance owns all per-cycle state (map manager, plan cache, planners).
// Storage for every grid and planner buffer is caller-owned: the views passed
// to the constructor must stay valid for the lifetime of the Core. No global
// or hidden state exists outside this object [FROM: AGENTS.md section 5].
class Core final {
public:
    // Stores the configuration and buffers. Does not validate placeholders;
    // call initialize() before step().
    Core(const teb::TebConfig& teb_config,
         const global_planner::GlobalPlannerConfig& planner_config,
         map::GridMap fixed_borders,
         map::MutableGridMap slam_overlay,
         map::MutableGridMap inflation,
         map::MutableGridMap composed,
         global_planner::PlannerWorkspace workspace,
         teb::TebWorkspace teb_workspace,
         global_planner::WaypointBuffer waypoints);
    ~Core();

    Core(const Core&) = delete;
    Core& operator=(const Core&) = delete;

    // Runs the placeholder guard (see the free initialize() above). Returns
    // Status::Valid only when the guard passes; step() refuses to run before
    // a Valid initialize().
    Status initialize();

    // Runs one control cycle in the order odometry, map manager, global
    // planner (on demand), TEB local planner, output converter. Any stage that
    // is not Valid is reported in Output::status with NaN next_pose.
    // Status::Fault is returned when the core is not initialized, or when
    // teb_config.step_dt does not equal snapshot.dt.
    Output step(const InputSnapshot& snapshot);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace core
