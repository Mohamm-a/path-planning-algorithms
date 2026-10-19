// Core facade. Owns the per-cycle state of every stage and runs the pipeline
// in the order fixed by AGENTS.md section 2:
//   odometry -> map manager -> global planner (on demand) -> TEB -> output
// [FROM: AGENTS.md section 2]
//
// Caller-owned storage: grids and planner buffers are passed in views, and the
// caller keeps them alive for the lifetime of the Core [FROM: AGENTS.md
// section 5]. The Core holds no wall-clock reads; the snapshot timestamp is the
// only time source [FROM: AGENTS.md section 5].
//
// Failure handling: any stage that is not Valid is reported in Output::status
// with NaN next_pose. No stage result is replaced by a fallback [FROM:
// AGENTS.md section 3, rule 8].

#include "core/core.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/odometry.hpp"
#include "core/stages/output_converter.hpp"
#include "core/stages/teb_local_planner.hpp"

namespace core {
namespace {

constexpr double kPi = 3.14159265358979323846;  // [FROM: mathematical constant]

// Wraps an angle to (-pi, pi] [FROM: AGENTS.md section 5, angles].
double wrap_angle(double angle) {
    double t = std::fmod(angle + kPi, 2.0 * kPi);
    if (t <= 0.0) {
        t += 2.0 * kPi;
    }
    return t - kPi;
}

// Output for a cycle that failed before a next pose existed. Numeric fields
// are NaN for the pose; dt and timestamp come from the snapshot.
Output fault_output(const InputSnapshot& snapshot, Status status) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Output out{};
    out.next_pose = Pose2D{nan, nan, nan};
    out.dt = snapshot.dt;
    out.timestamp = snapshot.timestamp;
    out.frame_id = Frame::Map;
    out.status = status;
    return out;
}

// Checks every sensor sample age against PH-0005 before any stage runs.
// Age = snapshot.timestamp - sample time. A negative or non-finite age means
// the sample is from the future or the data is corrupt: Fault. An age above
// the threshold means stale data: StaleSensor. Any Fault wins over StaleSensor.
Status sensor_freshness(const InputSnapshot& s) {
    const std::array<double, 5> sample_times{
        s.pose_timestamp, s.lidar_timestamp, s.encoder_timestamp,
        s.gyro_timestamp, s.steering_timestamp};
    bool stale = false;
    for (const double t : sample_times) {
        const double age = s.timestamp - t;
        if (!std::isfinite(age) || age < 0.0) {
            return Status::Fault;
        }
        if (age > CORE_PH_SENSOR_STALE_TIMEOUT_S) {
            stale = true;
        }
    }
    return stale ? Status::StaleSensor : Status::Valid;
}

// One planner per GlobalPlannerKind, indexed by the enum value.
constexpr std::size_t kPlannerKindCount = 3;
static_assert(static_cast<std::size_t>(GlobalPlannerKind::RRT) + 1 ==
                  kPlannerKindCount,
              "planner table must cover every GlobalPlannerKind");

}  // namespace

// All per-cycle state of the core. Stages are constructed here because the
// map manager needs its caller-owned grids at construction.
struct Core::Impl {
    Impl(const teb::TebConfig& teb_cfg,
         const global_planner::GlobalPlannerConfig& planner_cfg,
         map::GridMap fixed_borders, map::MutableGridMap slam_overlay,
         map::MutableGridMap inflation, map::MutableGridMap composed,
         global_planner::PlannerWorkspace ws, global_planner::WaypointBuffer wp)
        : teb_config(teb_cfg),
          planner_config(planner_cfg),
          map_manager(fixed_borders, slam_overlay, inflation, composed),
          workspace(ws),
          waypoints(wp) {
        for (std::size_t i = 0; i < planners.size(); ++i) {
            planners[i] = global_planner::make_global_planner(
                static_cast<GlobalPlannerKind>(i));
        }
    }

    teb::TebConfig teb_config;
    global_planner::GlobalPlannerConfig planner_config;
    map::GridMapManager map_manager;
    global_planner::PlannerWorkspace workspace;
    global_planner::WaypointBuffer waypoints;
    std::array<std::unique_ptr<global_planner::IGlobalPlanner>, kPlannerKindCount>
        planners;

    global_planner::PlanCache cache{false, Point2D{0.0, 0.0}, 0, 0.0};
    global_planner::GlobalGuide guide{Status::Fault, global_planner::PointView{nullptr, 0}};
    GlobalPlannerKind last_kind = GlobalPlannerKind::AStar;
    bool initialized = false;

    teb::TebLocalPlanner teb_stage;
    odometry::EncoderGyroOdometry odometry_stage;
    output::OutputConverter converter_stage;
};

Core::Core(const teb::TebConfig& teb_config,
           const global_planner::GlobalPlannerConfig& planner_config,
           map::GridMap fixed_borders, map::MutableGridMap slam_overlay,
           map::MutableGridMap inflation, map::MutableGridMap composed,
           global_planner::PlannerWorkspace workspace,
           global_planner::WaypointBuffer waypoints)
    : impl_(std::make_unique<Impl>(teb_config, planner_config, fixed_borders,
                                   slam_overlay, inflation, composed, workspace,
                                   waypoints)) {}

Core::~Core() = default;

Status Core::initialize() {
    const Status guard = core::initialize();
    impl_->initialized = (guard == Status::Valid);
    return guard;
}

Output Core::step(const InputSnapshot& snapshot) {
    Impl& m = *impl_;

    // Refuse to run before a Valid initialize() (no silent fallback).
    if (!m.initialized) {
        return fault_output(snapshot, Status::Fault);
    }

    // Sensor freshness is checked before any stage runs (PH-0005).
    const Status freshness = sensor_freshness(snapshot);
    if (freshness != Status::Valid) {
        return fault_output(snapshot, freshness);
    }

    // D2: the TEB horizon step must equal the snapshot dt exactly.
    if (m.teb_config.step_dt != snapshot.dt) {
        return fault_output(snapshot, Status::Fault);
    }

    // 1. Odometry.
    const odometry::OdometryEstimate od = m.odometry_stage.predict(snapshot);
    if (od.status != Status::Valid) {
        return fault_output(snapshot, od.status);
    }

    // 2. Map manager: update the SLAM overlay and compose the costmap.
    const Status map_status = m.map_manager.update(snapshot.pose, snapshot.lidar);
    if (map_status != Status::Valid) {
        return fault_output(snapshot, map_status);
    }

    // 3. Global planner, run on demand (goal, map, or period changed).
    const std::size_t idx = static_cast<std::size_t>(snapshot.global_planner);
    if (idx >= m.planners.size()) {
        return fault_output(snapshot, Status::Fault);
    }
    // A planner-kind change invalidates the cached plan. needs_replan() does
    // not see the kind, so the cache is marked invalid here.
    if (m.cache.valid && snapshot.global_planner != m.last_kind) {
        m.cache.valid = false;
    }
    m.last_kind = snapshot.global_planner;

    const std::uint64_t generation = m.map_manager.map_generation();
    if (global_planner::needs_replan(m.cache, snapshot.goal, generation,
                                     snapshot.timestamp)) {
        const Point2D start{snapshot.pose.x, snapshot.pose.y};
        m.guide = m.planners[idx]->plan(start, snapshot.goal,
                                        m.map_manager.composed(),
                                        m.planner_config, m.workspace,
                                        m.waypoints);
        global_planner::record_plan(m.cache, snapshot.goal, generation,
                                    snapshot.timestamp);
    }

    // 4. TEB local planner. It propagates any non-Valid guide status.
    const teb::LocalPlan plan = m.teb_stage.solve(
        snapshot, od, m.guide, m.map_manager.composed(), m.teb_config);

    // 5. Output converter, then angle normalization at the boundary.
    Output out = m.converter_stage.convert(plan, snapshot);
    if (out.status == Status::Valid) {
        out.next_pose.theta = wrap_angle(out.next_pose.theta);
    }
    return out;
}

}  // namespace core
