// Regression harness for the safety-critical Phase B fixes. It is a sim-only
// executable (built with ROBOT_CORE_BUILD_SIM) and uses only the public Core
// boundary, so it exercises the same path as the wall simulation.
//
// Covered here:
//   1. C1: finite goal -> NaN goal -> same finite goal must replan instead of
//      reusing the straight-ahead no-goal guide.
//   2. C3: horizon_steps above kMaxHorizonSteps returns Fault.
//   3. L1/L2: overlapping grid and TEB workspace storage returns Fault.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

#include "core/core.hpp"
#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/teb_local_planner.hpp"

namespace {

using namespace core;

constexpr std::size_t kN = 100;
constexpr std::size_t kCells = kN * kN;
constexpr std::size_t kHorizon = 10;
constexpr double kDt = 0.1;

struct Fixture {
    std::vector<map::CellState> fixed{kCells, map::CellState::Free};
    std::vector<map::CellState> slam{kCells, map::CellState::Unknown};
    std::vector<map::CellState> inflation{kCells, map::CellState::Unknown};
    std::vector<map::CellState> composed{kCells, map::CellState::Unknown};
    std::vector<real> g = std::vector<real>(kCells);
    std::vector<std::size_t> parent = std::vector<std::size_t>(kCells);
    std::vector<std::size_t> heap = std::vector<std::size_t>(kCells);
    std::vector<std::size_t> slot = std::vector<std::size_t>(kCells);
    std::unique_ptr<bool[]> closed{new bool[kCells]()};
    std::vector<Point2D> waypoints = std::vector<Point2D>(kCells);
    std::vector<real> x = std::vector<real>(2 * kHorizon);
    std::vector<real> grad = std::vector<real>(2 * kHorizon);
    std::vector<real> probe = std::vector<real>(2 * kHorizon);
    std::vector<real> cand = std::vector<real>(2 * kHorizon);
    std::vector<Pose2D> trajectory = std::vector<Pose2D>(kHorizon + 1);

    teb::TebConfig teb_config(std::size_t horizon = kHorizon) const {
        teb::TebConfig c{};
        c.horizon_steps = horizon;
        c.step_dt = kDt;
        c.target_speed = real(0.2);
        c.iterations = 20;
        c.step_size = real(0.1);
        c.fd_epsilon = real(1e-3);
        c.w_guide = real(0.3);
        c.w_obstacle = real(10);
        c.w_speed = real(0.1);
        c.w_accel = real(0.1);
        c.w_steer_change = real(0.1);
        return c;
    }

    global_planner::GlobalPlannerConfig planner_config() const {
        global_planner::GlobalPlannerConfig c{};
        c.max_expansions = 10000;
        c.max_nodes = 100;
        c.rrt_step_size = real(0.3);
        c.rrt_goal_tolerance = real(0.2);
        c.rrt_goal_bias = real(0.05);
        c.rrt_seed = 1;
        return c;
    }

    Core make(std::size_t horizon = kHorizon, real* aliased_grad = nullptr,
              map::CellState* aliased_slam = nullptr) {
        global_planner::PlannerWorkspace ws{};
        ws.capacity = kCells;
        ws.g_cost = g.data();
        ws.parent = parent.data();
        ws.closed = closed.get();
        ws.heap = heap.data();
        ws.heap_slot = slot.data();

        teb::TebWorkspace tw{};
        tw.decision_capacity = x.size();
        tw.x = x.data();
        tw.grad = aliased_grad == nullptr ? grad.data() : aliased_grad;
        tw.probe = probe.data();
        tw.cand = cand.data();
        tw.trajectory_capacity = trajectory.size();
        tw.trajectory = trajectory.data();

        return Core(
            teb_config(horizon), planner_config(),
            map::GridMap{Point2D{0, 0}, kN, kN, fixed.data()},
            map::MutableGridMap{Point2D{0, 0}, kN, kN,
                                aliased_slam == nullptr ? slam.data()
                                                        : aliased_slam},
            map::MutableGridMap{Point2D{0, 0}, kN, kN, inflation.data()},
            map::MutableGridMap{Point2D{0, 0}, kN, kN, composed.data()}, ws, tw,
            global_planner::WaypointBuffer{waypoints.data(), waypoints.size()});
    }
};

InputSnapshot snapshot(double t, Pose2D pose, Point2D goal, bool sweep) {
    InputSnapshot s{};
    s.pose = pose;
    s.goal = goal;
    s.is_new_sweep = sweep;
    s.encoder_rpm = real(0);
    s.steering_angle = real(0);
    s.gyro_yaw_rate = real(0);
    s.global_planner = GlobalPlannerKind::AStar;
    s.timestamp = t;
    s.dt = kDt;
    s.pose_timestamp = t;
    s.lidar_timestamp = t;
    s.encoder_timestamp = t;
    s.gyro_timestamp = t;
    s.steering_timestamp = t;
    return s;
}

bool fault_output(const Output& o) {
    return o.status == Status::Fault && std::isnan(o.next_pose.x) &&
           std::isnan(o.next_pose.y) && std::isnan(o.next_pose.theta);
}

int test_no_goal_cache() {
    Fixture f;
    Core core = f.make();
    if (core.initialize() != Status::Valid) {
        std::printf("C1 initialize failed\n");
        return 1;
    }

    const Point2D goal{3.0f, 2.0f};
    Output out = core.step(snapshot(1.0, Pose2D{2.0f, 2.0f, 0.0f}, goal, true));
    if (out.status != Status::Valid) {
        std::printf("C1 initial finite status=%d\n", static_cast<int>(out.status));
        return 1;
    }

    const real nan = std::numeric_limits<real>::quiet_NaN();
    out = core.step(snapshot(1.1, Pose2D{2.0f, 2.0f, 1.5707963f},
                             Point2D{nan, 2.0f}, false));
    if (out.status != Status::Valid) {
        std::printf("C1 no-goal status=%d\n", static_cast<int>(out.status));
        return 1;
    }
    // Keep the same finite goal and map generation, but block the start cell
    // directly in the caller-owned composed grid. If the no-goal cycle left
    // the cache valid, step() would reuse the straight-ahead guide and report
    // Valid. After the C1 fix it must replan, see the occupied start, and
    // return a fault/non-Valid status.
    const std::size_t start_cell = 40 * kN + 40;  // (2.0 m, 2.0 m) at 0.05 m
    f.composed[start_cell] = map::CellState::Occupied;
    out = core.step(snapshot(1.2, Pose2D{2.0f, 2.0f, 0.0f}, goal, false));
    const bool replanned = out.status != Status::Valid;
    std::printf("C1 replanned=%d status=%d nan=%d\n", replanned ? 1 : 0,
                static_cast<int>(out.status), fault_output(out) ? 1 : 0);
    return replanned ? 0 : 1;
}

int test_horizon_bound() {
    Fixture f;
    Core core = f.make(teb::kMaxHorizonSteps + 1);
    if (core.initialize() != Status::Valid) {
        std::printf("C3 initialize failed\n");
        return 1;
    }
    const Output out = core.step(snapshot(
        1.0, Pose2D{2.0f, 2.0f, 0.0f}, Point2D{3.0f, 2.0f}, true));
    std::printf("C3 horizon status=%d nan=%d\n", static_cast<int>(out.status),
                fault_output(out) ? 1 : 0);
    return fault_output(out) ? 0 : 1;
}

int test_teb_alias() {
    Fixture f;
    // Offset by one real: the ranges overlap without sharing a base pointer.
    Core core = f.make(kHorizon, f.x.data() + 1);
    if (core.initialize() != Status::Valid) {
        std::printf("L2 initialize failed\n");
        return 1;
    }
    const Output out = core.step(snapshot(
        1.0, Pose2D{2.0f, 2.0f, 0.0f}, Point2D{3.0f, 2.0f}, true));
    std::printf("L2 teb alias status=%d nan=%d\n", static_cast<int>(out.status),
                fault_output(out) ? 1 : 0);
    return fault_output(out) ? 0 : 1;
}

int test_grid_alias() {
    Fixture f;
    // Offset by one cell: the fixed and SLAM ranges overlap, not just alias.
    Core core = f.make(kHorizon, nullptr, f.fixed.data() + 1);
    if (core.initialize() != Status::Valid) {
        std::printf("L1 initialize failed\n");
        return 1;
    }
    const Output out = core.step(snapshot(
        1.0, Pose2D{2.0f, 2.0f, 0.0f}, Point2D{3.0f, 2.0f}, true));
    std::printf("L1 grid alias status=%d nan=%d\n", static_cast<int>(out.status),
                fault_output(out) ? 1 : 0);
    return fault_output(out) ? 0 : 1;
}

}  // namespace

int main() {
    const int failures = test_no_goal_cache() + test_horizon_bound() +
                         test_teb_alias() + test_grid_alias();
    std::printf("RESULT %s failures=%d\n", failures == 0 ? "PASS" : "FAIL",
                failures);
    return failures == 0 ? 0 : 1;
}
