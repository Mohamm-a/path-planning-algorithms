// End-to-end wall simulation for core::Core. Opt-in build only
// (CMake option ROBOT_CORE_BUILD_SIM). Not part of the core.
//
// Scenario [FROM: user answer, Phase B]:
//   start (1.0, 1.0, 0.0), goal (8.0, 8.0), goal tolerance 0.2 m,
//   wall segment (5.0, 2.0) -> (5.0, 8.0), 180 beams over -1.57..1.57 rad,
//   cycle dt 0.1 s, max 500 cycles, wall clearance 0.05 m beyond robot radius.
//
// Plant model [ASSUMED: the robot tracks next_pose exactly each cycle; no
// actuator or sensor noise]. Sensor values are derived from the applied motion
// so that the core's odometry reproduces the same pose:
//   omega = wrap(theta' - theta) / dt
//   v     = ((x' - x) cos(theta_mid) + (y' - y) sin(theta_mid)) / dt
//   rpm   = v * 60 / (2 pi * CORE_PH_WHEEL_RADIUS_M)
//   delta = atan(CORE_PH_WHEELBASE_M * omega / v)   (0 when v ~ 0)
//
// LiDAR: each beam is cast from the robot pose at angle theta + alpha against
// the wall. A beam with no hit within CORE_PH_LIDAR_MAX_RANGE_M returns the
// first value above max range, which the map manager treats as "no return".
//
// Placeholder values are supplied with -D on the compiler command line.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "core/core.hpp"
#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/odometry.hpp"
#include "core/stages/output_converter.hpp"
#include "core/stages/teb_local_planner.hpp"

using namespace core;

namespace {

constexpr std::size_t kN = 100;     // grid cells per side
constexpr double kOrigin = 0.0;     // m; map-frame position of cell (0, 0)
constexpr double kDt = 0.1;         // s [FROM: user answer]
constexpr double kGoalTol = 0.2;    // m [FROM: user answer]
constexpr int kMaxCycles = 500;     // [FROM: user answer]
// Default TEB target speed (m/s). Overridden by the optional speed argument.
constexpr double kDefaultSpeed_mps = 1.0;  // [FROM: user answer, speed 1.0 run]
// Sim-only TEB weight defaults; the same values as before the sweep options existed.
constexpr double kDefaultWGuide = 1.0;     // [ASSUMED: prior make_teb_config value]
constexpr double kDefaultWObstacle = 10.0;  // [ASSUMED: prior make_teb_config value]
constexpr double kClearanceMargin = 0.05;  // m beyond robot radius [FROM: user answer]
constexpr int kBeams = 180;         // [FROM: user answer]
constexpr double kAlphaMin = -1.57; // rad [FROM: user answer]
constexpr double kAlphaMax = 1.57;  // rad [FROM: user answer]
constexpr double kPi = 3.14159265358979323846;

const Point2D kStart{1.0, 1.0};     // m [FROM: user answer]
const Point2D kGoal{8.0, 8.0};      // m [FROM: user answer]
const Point2D kWallA{5.0, 2.0};     // m [FROM: user answer]
const Point2D kWallB{5.0, 8.0};     // m [FROM: user answer]

// Wraps an angle to (-pi, pi].
double wrap_angle(double a) {
    double t = std::fmod(a + kPi, 2.0 * kPi);
    if (t <= 0.0) {
        t += 2.0 * kPi;
    }
    return t - kPi;
}

// Range along the unit ray from (ox, oy) at angle ang to the wall, or +inf.
double ray_range(double ox, double oy, double ang) {
    const double dx = std::cos(ang);
    const double dy = std::sin(ang);
    const double ex = kWallB.x - kWallA.x;
    const double ey = kWallB.y - kWallA.y;
    const double denom = dx * ey - dy * ex;  // cross(d, e)
    if (std::fabs(denom) < 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    const double ax = kWallA.x - ox;
    const double ay = kWallA.y - oy;
    const double t = (ax * ey - ay * ex) / denom;  // along the ray
    const double s = (ax * dy - ay * dx) / denom;  // along the wall
    if (t < 0.0 || s < 0.0 || s > 1.0) {
        return std::numeric_limits<double>::infinity();
    }
    return t;
}

// Shortest distance from (px, py) to the wall segment.
double dist_to_wall(double px, double py) {
    const double ex = kWallB.x - kWallA.x;
    const double ey = kWallB.y - kWallA.y;
    const double len2 = ex * ex + ey * ey;
    const double u = std::clamp(((px - kWallA.x) * ex + (py - kWallA.y) * ey) / len2, 0.0, 1.0);
    const double cx = kWallA.x + u * ex - px;
    const double cy = kWallA.y + u * ey - py;
    return std::hypot(cx, cy);
}

// One LiDAR scan from the pose. Beams with no hit in range are set just past
// the max range, so the map manager treats them as no return.
std::vector<LidarPoint> scan(const Pose2D& p) {
    const double no_return = std::nextafter(CORE_PH_LIDAR_MAX_RANGE_M,
                                            std::numeric_limits<double>::infinity());
    std::vector<LidarPoint> pts;
    pts.reserve(kBeams);
    for (int i = 0; i < kBeams; ++i) {
        const double alpha =
            kAlphaMin + static_cast<double>(i) * (kAlphaMax - kAlphaMin) / (kBeams - 1);
        double r = ray_range(p.x, p.y, p.theta + alpha);
        if (!(r <= CORE_PH_LIDAR_MAX_RANGE_M)) {
            r = no_return;
        }
        pts.push_back(LidarPoint{r, alpha});
    }
    return pts;
}

struct Sensors {
    double rpm;
    double steer;
    double yaw_rate;
};

// Sensor values that make the core's odometry reproduce the applied motion.
Sensors sensors_for_motion(const Pose2D& p, const Pose2D& q) {
    const double omega = wrap_angle(q.theta - p.theta) / kDt;
    const double theta_mid = p.theta + omega * kDt / 2.0;
    const double v = ((q.x - p.x) * std::cos(theta_mid) + (q.y - p.y) * std::sin(theta_mid)) / kDt;
    const double steer_raw = std::fabs(v) > 1e-9 ? std::atan(CORE_PH_WHEELBASE_M * omega / v) : 0.0;
    // Clamp to the steering limit. This models the physical hard stops of the
    // Ackermann steering mechanism: the plant cannot steer past
    // CORE_PH_MAX_STEER_RAD, so the reported steering angle must not either.
    const double steer = std::clamp(steer_raw, -CORE_PH_MAX_STEER_RAD, CORE_PH_MAX_STEER_RAD);
    const double rpm = v * 60.0 / (2.0 * kPi * CORE_PH_WHEEL_RADIUS_M);
    return Sensors{rpm, steer, omega};
}

// TEB and global planner tuning: values carried over from the test
// harness. [ASSUMED: not supplied by the user; horizon_steps and
// weights are tunables, step_dt is set to the user's 0.1 s.]
teb::TebConfig make_teb_config(double target_speed, double w_guide, double w_obstacle) {
    teb::TebConfig tc{};
    tc.horizon_steps = 10;
    tc.step_dt = kDt;
    tc.target_speed = target_speed;
    tc.iterations = 100;
    tc.step_size = 0.1;
    tc.fd_epsilon = 1e-6;
    tc.w_guide = w_guide;
    tc.w_obstacle = w_obstacle;
    tc.w_speed = 0.1;
    tc.w_accel = 0.1;
    tc.w_steer_change = 0.1;
    return tc;
}

global_planner::GlobalPlannerConfig make_planner_config() {
    global_planner::GlobalPlannerConfig pc{};
    pc.max_expansions = 100000;
    return pc;
}

// Owns every caller-side buffer so the Core's views stay valid.
struct World {
    std::vector<map::CellState> fixed;
    std::vector<map::CellState> slam;
    std::vector<map::CellState> inflation;
    std::vector<map::CellState> composed;
    std::vector<double> g_cost;
    std::vector<std::size_t> parent;
    std::unique_ptr<bool[]> closed;
    std::vector<std::size_t> heap;
    std::vector<std::size_t> heap_slot;
    std::vector<Point2D> waypoints;

    World()
        : fixed(kN * kN, map::CellState::Free),  // no fixed obstacles; the wall is sensed
          slam(kN * kN, map::CellState::Unknown),
          inflation(kN * kN, map::CellState::Unknown),
          composed(kN * kN, map::CellState::Unknown),
          g_cost(kN * kN),
          parent(kN * kN),
          closed(new bool[kN * kN]()),
          heap(kN * kN),
          heap_slot(kN * kN),
          waypoints(256) {}

    Core make_core(double target_speed, double w_guide, double w_obstacle) {
        const teb::TebConfig tc = make_teb_config(target_speed, w_guide, w_obstacle);
        const global_planner::GlobalPlannerConfig pc = make_planner_config();

        map::GridMap fixed_map{Point2D{kOrigin, kOrigin}, kN, kN, fixed.data()};
        map::MutableGridMap slam_m{Point2D{kOrigin, kOrigin}, kN, kN, slam.data()};
        map::MutableGridMap infl_m{Point2D{kOrigin, kOrigin}, kN, kN, inflation.data()};
        map::MutableGridMap comp_m{Point2D{kOrigin, kOrigin}, kN, kN, composed.data()};

        global_planner::PlannerWorkspace ws{};
        ws.capacity = kN * kN;
        ws.g_cost = g_cost.data();
        ws.parent = parent.data();
        ws.closed = closed.get();
        ws.heap = heap.data();
        ws.heap_slot = heap_slot.data();

        global_planner::WaypointBuffer wb{waypoints.data(), waypoints.size()};
        return Core(tc, pc, fixed_map, slam_m, infl_m, comp_m, ws, wb);
    }
};

}  // namespace

int main(int argc, char** argv) {
    // Optional speed argument (m/s): wall_sim [speed_mps].
    double speed = kDefaultSpeed_mps;
    if (argc > 1) {
        char* end = nullptr;
        speed = std::strtod(argv[1], &end);
        if (end == argv[1] || *end != '\0' || !(speed > 0.0)) {
            std::printf("RESULT FAIL invalid speed argument '%s'\n", argv[1]);
            return 2;
        }
    }
    // Optional TEB weights: wall_sim [speed] [w_guide] [w_obstacle].
    double w_guide = kDefaultWGuide;
    double w_obstacle = kDefaultWObstacle;
    if (argc > 2) {
        char* end = nullptr;
        w_guide = std::strtod(argv[2], &end);
        if (end == argv[2] || *end != '\0' || !(w_guide >= 0.0)) {
            std::printf("RESULT FAIL invalid w_guide argument '%s'\n", argv[2]);
            return 2;
        }
    }
    if (argc > 3) {
        char* end = nullptr;
        w_obstacle = std::strtod(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !(w_obstacle >= 0.0)) {
            std::printf("RESULT FAIL invalid w_obstacle argument '%s'\n", argv[3]);
            return 2;
        }
    }
    World w;
    Core core = w.make_core(speed, w_guide, w_obstacle);
    if (core.initialize() != Status::Valid) {
        std::printf("RESULT FAIL initialize\n");
        return 2;
    }

    Pose2D pose{kStart.x, kStart.y, 0.0};
    Sensors last{0.0, 0.0, 0.0};  // robot starts at rest
    std::printf("cycle t x y theta status dist_goal clearance\n");

    for (int k = 0; k < kMaxCycles; ++k) {
        const double t = static_cast<double>(k) * kDt;
        const double dist_goal = std::hypot(kGoal.x - pose.x, kGoal.y - pose.y);
        const double clearance = dist_to_wall(pose.x, pose.y) - CORE_PH_ROBOT_RADIUS_M;

        if (clearance < kClearanceMargin) {
            std::printf("RESULT FAIL wall clearance at cycle %d pose %.9f %.9f %.9f\n", k,
                        pose.x, pose.y, pose.theta);
            return 1;
        }
        if (dist_goal <= kGoalTol) {
            std::printf("RESULT SUCCESS cycles=%d final %.9f %.9f %.9f\n", k, pose.x, pose.y,
                        pose.theta);
            return 0;
        }

        const std::vector<LidarPoint> pts = scan(pose);
        InputSnapshot s{};
        s.pose = pose;
        s.goal = kGoal;
        s.lidar = LidarView{pts.data(), pts.size()};
        s.encoder_rpm = last.rpm;
        s.steering_angle = last.steer;
        s.gyro_yaw_rate = last.yaw_rate;
        s.global_planner = GlobalPlannerKind::AStar;
        s.timestamp = t;
        s.dt = kDt;
        s.pose_timestamp = t;
        s.lidar_timestamp = t;
        s.encoder_timestamp = t;
        s.gyro_timestamp = t;
        s.steering_timestamp = t;

        const Output out = core.step(s);
        if (out.status != Status::Valid) {
            std::printf("RESULT FAIL status %d at cycle %d\n", static_cast<int>(out.status), k);
            return 1;
        }

        std::printf("%d %.3f %.9f %.9f %.9f %d %.9f %.9f\n", k, t, pose.x, pose.y, pose.theta,
                    static_cast<int>(out.status), dist_goal, clearance);
        last = sensors_for_motion(pose, out.next_pose);
        pose = out.next_pose;
    }

    std::printf("RESULT FAIL max cycles (%d) reached without goal, final %.9f %.9f %.9f\n",
                kMaxCycles, pose.x, pose.y, pose.theta);
    return 1;
}
