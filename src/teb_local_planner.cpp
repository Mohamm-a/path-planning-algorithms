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

#include "core/stages/teb_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace core::teb {
namespace {

constexpr double kPi = 3.14159265358979323846;  // [FROM: mathematical constant]
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool is_finite(double v) { return std::isfinite(v); }

bool is_non_negative(double v) { return std::isfinite(v) && v >= 0.0; }

// Wraps an angle to (-pi, pi].
double wrap_angle(double a) {
    double w = std::fmod(a + kPi, 2.0 * kPi);
    if (w <= 0.0) {
        w += 2.0 * kPi;
    }
    return w - kPi;
}

// True when every placeholder used by this stage is replaced. A zero or NaN
// placeholder is not a valid physical value. The steering limit must be below
// pi/2, where tan() is finite.
bool placeholders_replaced() {
    return WHEELBASE_M > 0.0 && MAX_STEER_RAD > 0.0 &&
           MAX_STEER_RAD < kPi / 2.0 && MAX_SPEED_MPS > 0.0 &&
           ROBOT_RADIUS_M > 0.0 && MAX_ACCEL_MPS2 > 0.0 &&
           MAX_STEER_RATE_RADPS > 0.0 && map::kCellResolution_m > 0.0;
}

bool config_valid(const TebConfig& c) {
    return c.horizon_steps >= 1 && is_finite(c.step_dt) && c.step_dt > 0.0 &&
           is_finite(c.target_speed) &&
           std::fabs(c.target_speed) <= MAX_SPEED_MPS &&
           is_finite(c.step_size) && c.step_size > 0.0 &&
           is_finite(c.fd_epsilon) && c.fd_epsilon > 0.0 &&
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
        g.origin.x + (static_cast<double>(ix) + 0.5) * map::kCellResolution_m,
        g.origin.y + (static_cast<double>(iy) + 0.5) * map::kCellResolution_m};
}

// Signed clearance from p to the nearest blocked cell (Occupied or Unknown)
// or to the grid boundary. The value is conservative: it subtracts a cell
// half-diagonal from the center distance. It is negative outside the grid.
// Only values below `radius` matter, so cells beyond radius + half-diagonal
// are not searched.
double clearance(const map::GridMap& g, Point2D p, double radius) {
    const double res = map::kCellResolution_m;
    const double x_min = g.origin.x;
    const double y_min = g.origin.y;
    const double x_max = x_min + static_cast<double>(g.width) * res;
    const double y_max = y_min + static_cast<double>(g.height) * res;
    const double half_diag = 0.5 * res * std::sqrt(2.0);

    const double dx_out = std::max({x_min - p.x, 0.0, p.x - x_max});
    const double dy_out = std::max({y_min - p.y, 0.0, p.y - y_max});
    if (dx_out > 0.0 || dy_out > 0.0) {
        return -std::hypot(dx_out, dy_out);
    }

    double best = std::min({p.x - x_min, x_max - p.x, p.y - y_min, y_max - p.y});

    const double search = radius + half_diag;
    const double max_ix = static_cast<double>(g.width - 1);
    const double max_iy = static_cast<double>(g.height - 1);
    const auto ix0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x - search - x_min) / res), 0.0, max_ix));
    const auto ix1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x + search - x_min) / res), 0.0, max_ix));
    const auto iy0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y - search - y_min) / res), 0.0, max_iy));
    const auto iy1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y + search - y_min) / res), 0.0, max_iy));

    for (std::size_t iy = iy0; iy <= iy1; ++iy) {
        for (std::size_t ix = ix0; ix <= ix1; ++ix) {
            if (g.cells[iy * g.width + ix] == map::CellState::Free) {
                continue;
            }
            const Point2D c = cell_center(g, ix, iy);
            const double d = std::hypot(c.x - p.x, c.y - p.y) - half_diag;
            best = std::min(best, d);
        }
    }
    return best;
}

double distance_to_segment(Point2D p, Point2D a, Point2D b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    const double t =
        len2 > 0.0 ? std::clamp(((p.x - a.x) * vx + (p.y - a.y) * vy) / len2, 0.0, 1.0)
                   : 0.0;
    return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

// Distance from p to the guide polyline. The guide has at least one waypoint.
double distance_to_guide(const global_planner::PointView& guide, Point2D p) {
    if (guide.count == 1) {
        return std::hypot(p.x - guide.data[0].x, p.y - guide.data[0].y);
    }
    double best = kInf;
    for (std::size_t i = 0; i + 1 < guide.count; ++i) {
        best = std::min(best, distance_to_segment(p, guide.data[i], guide.data[i + 1]));
    }
    return best;
}

// Everything the cost and projection need, fixed for one solve() call.
struct Problem {
    Pose2D start;
    double v0;     // m/s; initial speed (validated against PH-0004)
    double delta0; // rad; initial steering (validated against PH-0002)
    global_planner::PointView guide;
    map::GridMap costmap;
    TebConfig cfg;
};

// Forward-simulates the bicycle model. x holds N speeds, then N steering angles.
void rollout(const Problem& pr, const std::vector<double>& x, std::vector<Pose2D>& poses) {
    const std::size_t n = pr.cfg.horizon_steps;
    const double dt = pr.cfg.step_dt;
    poses.resize(n + 1);
    poses[0] = pr.start;
    for (std::size_t k = 0; k < n; ++k) {
        const Pose2D s = poses[k];
        const double v = x[k];
        const double delta = x[n + k];
        poses[k + 1] = Pose2D{
            s.x + v * std::cos(s.theta) * dt,
            s.y + v * std::sin(s.theta) * dt,
            wrap_angle(s.theta + v * std::tan(delta) / WHEELBASE_M * dt)};
    }
}

double total_cost(const Problem& pr, const std::vector<double>& x, std::vector<Pose2D>& poses) {
    const std::size_t n = pr.cfg.horizon_steps;
    const double radius = ROBOT_RADIUS_M;
    rollout(pr, x, poses);

    double j = 0.0;
    for (std::size_t k = 1; k <= n; ++k) {
        const Point2D p{poses[k].x, poses[k].y};
        const double dg = distance_to_guide(pr.guide, p);
        j += pr.cfg.w_guide * dg * dg;
        const double violation = std::max(0.0, radius - clearance(pr.costmap, p, radius));
        j += pr.cfg.w_obstacle * violation * violation;
    }
    for (std::size_t k = 0; k < n; ++k) {
        const double v = x[k];
        const double v_prev = k == 0 ? pr.v0 : x[k - 1];
        const double delta = x[n + k];
        const double d_prev = k == 0 ? pr.delta0 : x[n + k - 1];
        const double e_speed = v - pr.cfg.target_speed;
        const double e_accel = v - v_prev;
        const double e_steer = delta - d_prev;
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
void project(const Problem& pr, std::vector<double>& x) {
    const std::size_t n = pr.cfg.horizon_steps;
    const double dv_max = MAX_ACCEL_MPS2 * pr.cfg.step_dt;
    const double dd_max = MAX_STEER_RATE_RADPS * pr.cfg.step_dt;

    double v_prev = pr.v0;
    for (std::size_t k = 0; k < n; ++k) {
        const double lo = std::max(-MAX_SPEED_MPS, v_prev - dv_max);
        const double hi = std::min(MAX_SPEED_MPS, v_prev + dv_max);
        x[k] = std::clamp(x[k], lo, hi);
        v_prev = x[k];
    }

    double d_prev = pr.delta0;
    for (std::size_t k = 0; k < n; ++k) {
        const double lo = std::max(-MAX_STEER_RAD, d_prev - dd_max);
        const double hi = std::min(MAX_STEER_RAD, d_prev + dd_max);
        x[n + k] = std::clamp(x[n + k], lo, hi);
        d_prev = x[n + k];
    }
}

LocalPlan invalid_plan(Status status) {
    return LocalPlan{kNaN, kNaN, {}, status};
}

}  // namespace

LocalPlan TebLocalPlanner::solve(const InputSnapshot& snapshot,
                                 const odometry::OdometryEstimate& odometry,
                                 const global_planner::GlobalGuide& guide,
                                 map::GridMap costmap,
                                 const TebConfig& config) const {
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
    if (std::fabs(odometry.forward_speed) > MAX_SPEED_MPS ||
        std::fabs(snapshot.steering_angle) > MAX_STEER_RAD) {
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

    // Start from the current speed and steering held constant over the horizon.
    std::vector<double> x(dim);
    std::fill(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(n), odometry.forward_speed);
    std::fill(x.begin() + static_cast<std::ptrdiff_t>(n), x.end(), snapshot.steering_angle);
    project(pr, x);

    // Fixed-iteration projected descent. The step is halved when a candidate
    // does not lower the cost.
    std::vector<Pose2D> poses;
    double j = total_cost(pr, x, poses);
    std::vector<double> grad(dim);
    std::vector<double> probe(dim);
    std::vector<double> cand(dim);
    double eta = config.step_size;
    for (std::size_t it = 0; it < config.iterations; ++it) {
        for (std::size_t i = 0; i < dim; ++i) {
            probe = x;
            probe[i] += config.fd_epsilon;
            grad[i] = (total_cost(pr, probe, poses) - j) / config.fd_epsilon;
        }
        for (std::size_t i = 0; i < dim; ++i) {
            cand[i] = x[i] - eta * grad[i];
        }
        project(pr, cand);
        const double jc = total_cost(pr, cand, poses);
        if (jc < j) {
            x = cand;
            j = jc;
        } else {
            eta *= 0.5;
        }
    }

    if (!is_finite(j)) {
        return invalid_plan(Status::Fault);
    }

    // Final rollout. Every trajectory point must keep the robot radius clear
    // of blocked cells, or no collision-free local plan was found.
    rollout(pr, x, poses);
    const double radius = ROBOT_RADIUS_M;
    for (std::size_t k = 1; k <= n; ++k) {
        if (clearance(costmap, Point2D{poses[k].x, poses[k].y}, radius) < radius) {
            return invalid_plan(Status::NoPath);
        }
    }

    return LocalPlan{x[0], x[n], std::move(poses), Status::Valid};
}

}  // namespace core::teb
