// TEB local planner implementation. Header: include/core/stages/teb_local_planner.hpp.
//
// Model: kinematic bicycle, x' = v cos(theta), y' = v sin(theta),
// theta' = v tan(delta) / L [ASSUMED: standard Ackermann kinematics; no
// dynamics or slip model is given in AGENTS.md].
//
// Decision variables: speed and steering for each horizon step. Every
// candidate is projected onto the limits (speed box, acceleration, steering
// box, steering rate) before cost evaluation, so the output is always feasible.
//
// Cost: guide distance, obstacle clearance violation, speed error, speed
// change and steering change. Optimization is fixed-iteration projected
// finite-difference descent. A step is accepted only when it lowers the cost.
//
// Every per-call buffer comes from TebWorkspace, so solve() does not allocate
// [FROM: user approval, Q2].

#include "core/stages/teb_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace core::teb {
namespace {

constexpr real kPi = 3.14159265358979323846F;  // [FROM: mathematical constant]
constexpr real kNaN = std::numeric_limits<real>::quiet_NaN();
constexpr real kInf = std::numeric_limits<real>::infinity();

// Time is checked as double so callers cannot narrow a duration by accident.
// One template instead of two overloads: under CORE_REAL_DOUBLE the `real`
// and `double` overloads would collapse to the same signature [FROM: user
// approval, Phase A finding].
template <typename T>
bool is_finite(T v) {
    return std::isfinite(static_cast<double>(v));
}

bool is_non_negative(real v) { return std::isfinite(v) && v >= real(0); }

// Wraps an angle to (-pi, pi].
real wrap_angle(real a) {
    real w = std::fmod(a + kPi, real(2) * kPi);
    if (w <= real(0)) {
        w += real(2) * kPi;
    }
    return w - kPi;
}

// True when every placeholder used by this stage is replaced. A zero or NaN
// placeholder is not a valid physical value. The steering limit must be below
// pi/2, where tan() is finite.
bool placeholders_replaced() {
    return WHEELBASE_M > 0.0 && MAX_STEER_RAD > 0.0 &&
           MAX_STEER_RAD < static_cast<double>(kPi) / 2.0 && MAX_SPEED_MPS > 0.0 &&
           ROBOT_RADIUS_M > 0.0 && MAX_ACCEL_MPS2 > 0.0 &&
           MAX_STEER_RATE_RADPS > 0.0 && map::kCellResolution_m > real(0);
}

bool config_valid(const TebConfig& c) {
    return c.horizon_steps >= 1 && is_finite(c.step_dt) && c.step_dt > 0.0 &&
           is_finite(c.target_speed) &&
           std::fabs(c.target_speed) <= static_cast<real>(MAX_SPEED_MPS) &&
           is_finite(c.step_size) && c.step_size > real(0) &&
           is_finite(c.fd_epsilon) && c.fd_epsilon > real(0) &&
           is_non_negative(c.w_guide) && is_non_negative(c.w_obstacle) &&
           is_non_negative(c.w_speed) && is_non_negative(c.w_accel) &&
           is_non_negative(c.w_steer_change);
}

bool grid_is_usable(const map::GridMap& g) {
    if (g.width == 0 || g.height == 0 || g.cells == nullptr) {
        return false;
    }
    if (g.width > std::numeric_limits<std::size_t>::max() / g.height) {
        return false;
    }
    return is_finite(g.origin.x) && is_finite(g.origin.y);
}

Point2D cell_center(const map::GridMap& g, std::size_t ix, std::size_t iy) {
    return Point2D{
        g.origin.x + (static_cast<real>(ix) + real(0.5)) * map::kCellResolution_m,
        g.origin.y + (static_cast<real>(iy) + real(0.5)) * map::kCellResolution_m};
}

// Signed clearance from p to the nearest blocked cell (Occupied or Unknown)
// or to the grid boundary. The value is conservative: it subtracts a cell
// half-diagonal from the center distance. It is negative outside the grid.
// Only values below `radius` matter, so cells beyond radius + half-diagonal
// are not searched.
real clearance(const map::GridMap& g, Point2D p, real radius) {
    const real res = map::kCellResolution_m;
    const real x_min = g.origin.x;
    const real y_min = g.origin.y;
    const real x_max = x_min + static_cast<real>(g.width) * res;
    const real y_max = y_min + static_cast<real>(g.height) * res;
    const real half_diag = real(0.5) * res * std::sqrt(real(2));

    const real dx_out = std::max({x_min - p.x, real(0), p.x - x_max});
    const real dy_out = std::max({y_min - p.y, real(0), p.y - y_max});
    if (dx_out > real(0) || dy_out > real(0)) {
        return -std::hypot(dx_out, dy_out);
    }

    real best = std::min({p.x - x_min, x_max - p.x, p.y - y_min, y_max - p.y});

    const real search = radius + half_diag;
    const real max_ix = static_cast<real>(g.width - 1);
    const real max_iy = static_cast<real>(g.height - 1);
    const auto ix0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x - search - x_min) / res), real(0), max_ix));
    const auto ix1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x + search - x_min) / res), real(0), max_ix));
    const auto iy0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y - search - y_min) / res), real(0), max_iy));
    const auto iy1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y + search - y_min) / res), real(0), max_iy));

    for (std::size_t iy = iy0; iy <= iy1; ++iy) {
        for (std::size_t ix = ix0; ix <= ix1; ++ix) {
            if (g.cells[iy * g.width + ix] == map::CellState::Free) {
                continue;
            }
            const Point2D c = cell_center(g, ix, iy);
            const real d = std::hypot(c.x - p.x, c.y - p.y) - half_diag;
            best = std::min(best, d);
        }
    }
    return best;
}

real distance_to_segment(Point2D p, Point2D a, Point2D b) {
    const real vx = b.x - a.x;
    const real vy = b.y - a.y;
    const real len2 = vx * vx + vy * vy;
    const real t =
        len2 > real(0)
            ? std::clamp(((p.x - a.x) * vx + (p.y - a.y) * vy) / len2, real(0), real(1))
            : real(0);
    return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

// Distance from p to the guide polyline. The guide has at least one waypoint.
real distance_to_guide(const global_planner::PointView& guide, Point2D p) {
    if (guide.count == 1) {
        return std::hypot(p.x - guide.data[0].x, p.y - guide.data[0].y);
    }
    real best = kInf;
    for (std::size_t i = 0; i + 1 < guide.count; ++i) {
        best = std::min(best, distance_to_segment(p, guide.data[i], guide.data[i + 1]));
    }
    return best;
}

// Everything the cost and projection need, fixed for one solve() call.
struct Problem {
    Pose2D start;
    real v0;     // m/s; initial speed (validated against PH-0004)
    real delta0; // rad; initial steering (validated against PH-0002)
    global_planner::PointView guide;
    map::GridMap costmap;
    TebConfig cfg;
};

// Forward-simulates the bicycle model. x holds N speeds, then N steering angles.
void rollout(const Problem& pr, const real* x, Pose2D* poses) {
    const std::size_t n = pr.cfg.horizon_steps;
    const real dt = static_cast<real>(pr.cfg.step_dt);
    poses[0] = pr.start;
    for (std::size_t k = 0; k < n; ++k) {
        const Pose2D s = poses[k];
        const real v = x[k];
        const real delta = x[n + k];
        poses[k + 1] = Pose2D{
            s.x + v * std::cos(s.theta) * dt,
            s.y + v * std::sin(s.theta) * dt,
            wrap_angle(s.theta + v * std::tan(delta) / static_cast<real>(WHEELBASE_M) * dt)};
    }
}

real total_cost(const Problem& pr, const real* x, Pose2D* poses) {
    const std::size_t n = pr.cfg.horizon_steps;
    const real radius = static_cast<real>(ROBOT_RADIUS_M);
    rollout(pr, x, poses);

    real j = real(0);
    for (std::size_t k = 1; k <= n; ++k) {
        const Point2D p{poses[k].x, poses[k].y};
        const real dg = distance_to_guide(pr.guide, p);
        j += pr.cfg.w_guide * dg * dg;
        const real violation = std::max(real(0), radius - clearance(pr.costmap, p, radius));
        j += pr.cfg.w_obstacle * violation * violation;
    }
    for (std::size_t k = 0; k < n; ++k) {
        const real v = x[k];
        const real v_prev = k == 0 ? pr.v0 : x[k - 1];
        const real delta = x[n + k];
        const real d_prev = k == 0 ? pr.delta0 : x[n + k - 1];
        const real e_speed = v - pr.cfg.target_speed;
        const real e_accel = v - v_prev;
        const real e_steer = delta - d_prev;
        j += pr.cfg.w_speed * e_speed * e_speed + pr.cfg.w_accel * e_accel * e_accel +
             pr.cfg.w_steer_change * e_steer * e_steer;
    }
    return j;
}

// Projects x onto the limits, in order. Each speed is clamped to the speed box
// and to the acceleration window around the previous speed. Each steering
// angle is clamped to the steering box and to the rate window around the
// previous angle. Both windows contain the previous value, so the clamp bounds
// are ordered and the result is always feasible.
void project(const Problem& pr, real* x) {
    const std::size_t n = pr.cfg.horizon_steps;
    const real dt = static_cast<real>(pr.cfg.step_dt);
    const real v_max = static_cast<real>(MAX_SPEED_MPS);
    const real dv_max = static_cast<real>(MAX_ACCEL_MPS2) * dt;
    const real d_max = static_cast<real>(MAX_STEER_RAD);
    const real dd_max = static_cast<real>(MAX_STEER_RATE_RADPS) * dt;

    real v_prev = pr.v0;
    for (std::size_t k = 0; k < n; ++k) {
        const real lo = std::max(-v_max, v_prev - dv_max);
        const real hi = std::min(v_max, v_prev + dv_max);
        x[k] = std::clamp(x[k], lo, hi);
        v_prev = x[k];
    }

    real d_prev = pr.delta0;
    for (std::size_t k = 0; k < n; ++k) {
        const real lo = std::max(-d_max, d_prev - dd_max);
        const real hi = std::min(d_max, d_prev + dd_max);
        x[n + k] = std::clamp(x[n + k], lo, hi);
        d_prev = x[n + k];
    }
}

LocalPlan invalid_plan(Status status) {
    return LocalPlan{kNaN, kNaN, PoseView{nullptr, 0}, status};
}

// The workspace arrays must be distinct: an aliased buffer would let one
// update overwrite another, silently corrupting the descent.
bool workspace_is_usable(const TebWorkspace& ws, std::size_t dim, std::size_t poses) {
    if (ws.decision_capacity < dim || ws.trajectory_capacity < poses) {
        return false;
    }
    const real* const arrays[4] = {ws.x, ws.grad, ws.probe, ws.cand};
    for (const real* a : arrays) {
        if (a == nullptr) {
            return false;
        }
    }
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = i + 1; j < 4; ++j) {
            if (arrays[i] == arrays[j]) {
                return false;
            }
        }
    }
    return ws.trajectory != nullptr;
}

}  // namespace

LocalPlan TebLocalPlanner::solve(const InputSnapshot& snapshot,
                                 const odometry::OdometryEstimate& odometry,
                                 const global_planner::GlobalGuide& guide,
                                 map::GridMap costmap,
                                 const TebConfig& config,
                                 TebWorkspace& workspace) const {
    // Placeholders and configuration. Invalid input is a Fault, never a fallback.
    if (!placeholders_replaced() || !config_valid(config) || !grid_is_usable(costmap)) {
        return invalid_plan(Status::Fault);
    }

    // Upstream status is propagated, so a stale or no-path input is not hidden.
    if (odometry.status != Status::Valid) {
        return invalid_plan(odometry.status);
    }
    if (guide.status != Status::Valid) {
        return invalid_plan(guide.status);
    }
    if (guide.waypoints.data == nullptr || guide.waypoints.count == 0) {
        return invalid_plan(Status::Fault);
    }
    for (std::size_t i = 0; i < guide.waypoints.count; ++i) {
        if (!is_finite(guide.waypoints.data[i].x) || !is_finite(guide.waypoints.data[i].y)) {
            return invalid_plan(Status::Fault);
        }
    }

    // Initial state. The initial speed and steering must already lie inside
    // the limits, so the projection below is always feasible.
    const Pose2D& pose = snapshot.pose;
    if (!is_finite(pose.x) || !is_finite(pose.y) || !is_finite(pose.theta) ||
        !is_finite(odometry.forward_speed) || !is_finite(snapshot.steering_angle)) {
        return invalid_plan(Status::Fault);
    }
    if (std::fabs(odometry.forward_speed) > static_cast<real>(MAX_SPEED_MPS) ||
        std::fabs(snapshot.steering_angle) > static_cast<real>(MAX_STEER_RAD)) {
        return invalid_plan(Status::Fault);
    }

    const Problem pr{pose,
                     odometry.forward_speed,
                     snapshot.steering_angle,
                     guide.waypoints,
                     costmap,
                     config};
    const std::size_t n = config.horizon_steps;
    const std::size_t dim = 2 * n;
    if (!workspace_is_usable(workspace, dim, n + 1)) {
        return invalid_plan(Status::Fault);
    }

    real* const x = workspace.x;
    real* const grad = workspace.grad;
    real* const probe = workspace.probe;
    real* const cand = workspace.cand;

    // Start from the current speed and steering held constant over the horizon.
    std::fill(x, x + n, odometry.forward_speed);
    std::fill(x + n, x + dim, snapshot.steering_angle);
    project(pr, x);

    // Fixed-iteration projected descent. The step is halved when a candidate
    // does not lower the cost.
    real j = total_cost(pr, x, workspace.trajectory);
    real eta = config.step_size;
    for (std::size_t it = 0; it < config.iterations; ++it) {
        for (std::size_t i = 0; i < dim; ++i) {
            std::copy(x, x + dim, probe);
            probe[i] += config.fd_epsilon;
            grad[i] = (total_cost(pr, probe, workspace.trajectory) - j) / config.fd_epsilon;
        }
        for (std::size_t i = 0; i < dim; ++i) {
            cand[i] = x[i] - eta * grad[i];
        }
        project(pr, cand);
        const real jc = total_cost(pr, cand, workspace.trajectory);
        if (jc < j) {
            std::copy(cand, cand + dim, x);
            j = jc;
        } else {
            eta *= real(0.5);
        }
    }

    if (!is_finite(j)) {
        return invalid_plan(Status::Fault);
    }

    // Final rollout. Every trajectory point must keep the robot radius clear
    // of blocked cells, or no collision-free local plan was found.
    rollout(pr, x, workspace.trajectory);
    const real radius = static_cast<real>(ROBOT_RADIUS_M);
    for (std::size_t k = 1; k <= n; ++k) {
        if (clearance(costmap, Point2D{workspace.trajectory[k].x, workspace.trajectory[k].y},
                      radius) < radius) {
            return invalid_plan(Status::NoPath);
        }
    }

    return LocalPlan{x[0], x[n], PoseView{workspace.trajectory, n + 1}, Status::Valid};
}

}  // namespace core::teb
