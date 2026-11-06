// TEB local planner stage interface. Declarations only; implementation is in
// src/teb_local_planner.cpp. Runs every cycle with Ackermann constraints.
// Consumes pose, odometry, global guide, costmap and limits [FROM: AGENTS.md
// section 2].
//
// Ackermann limits are compile-time placeholders in placeholders.hpp
// (PH-0001, PH-0002, PH-0004, PH-0018, PH-0019, PH-0020, PH-0008). The minimum
// turn radius (old PH-0003) is derived as wheelbase / tan(max steer), so it is
// not stored separately [FROM: user answer, TEB Phase A approval].
//
// No allocation occurs inside solve(): every per-call buffer is caller-owned
// and supplied through TebWorkspace [FROM: user approval, Q2].

#pragma once

#include <cstddef>

#include "core/core.hpp"
#include "core/placeholders.hpp"
#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/odometry.hpp"

namespace core::teb {

// Non-owning view over an ordered pose sequence. The caller owns the storage.
struct PoseView {
    const Pose2D* data;  // may be null only when count is 0
    std::size_t count;   // number of valid entries at data
};

// Caller-owned scratch memory for one solve() call. x, grad, probe and cand
// each hold 2 * horizon_steps reals; trajectory holds horizon_steps + 1 poses.
// A smaller capacity, a null pointer, or two arrays that share storage returns
// Status::Fault. No allocation occurs inside solve() [FROM: user approval, Q2].
struct TebWorkspace {
    std::size_t decision_capacity;  // reals available in each decision array
    real* x;                        // current decision vector
    real* grad;                     // finite-difference gradient
    real* probe;                    // perturbed copy for one gradient component
    real* cand;                     // projected candidate step
    std::size_t trajectory_capacity;  // poses available in trajectory
    Pose2D* trajectory;               // rollout of the accepted decision vector
};

// Runtime tuning. No defaults: the caller must supply every field. Invalid
// values make solve() return Status::Fault. step_dt stays double because it is
// a duration [FROM: user approval, Q1].
struct TebConfig {
    std::size_t horizon_steps;  // integration steps in the horizon; >= 1
    double step_dt;             // s; duration of one horizon step; > 0
    real target_speed;          // m/s; reference speed; |value| <= PH-0004
    std::size_t iterations;     // fixed number of descent iterations
    real step_size;             // m or rad per unit gradient; > 0; halved on rejection
    real fd_epsilon;            // finite-difference perturbation; > 0
    real w_guide;               // >= 0; squared distance to the global guide
    real w_obstacle;            // >= 0; squared clearance violation (robot radius)
    real w_speed;               // >= 0; squared speed error to target_speed
    real w_accel;               // >= 0; squared speed change between steps
    real w_steer_change;        // >= 0; squared steering change between steps
};

// Local plan for one control cycle. Commands and trajectory are valid only
// when status is Status::Valid. Otherwise v_cmd and delta_cmd are NaN and
// trajectory is empty, so a faulted or no-path plan cannot be used silently.
struct LocalPlan {
    real v_cmd;                      // m/s; speed for the first step
    real delta_cmd;                  // rad; steering for the first step; + = left
    PoseView trajectory;             // horizon_steps + 1 poses, start first; map frame
    Status status = Status::Fault;   // no-silent-fallback default
};

// TEB local planner interface.
class ITebLocalPlanner {
public:
    virtual ~ITebLocalPlanner() = default;

    // Optimizes speed and steering over the horizon, starting at snapshot.pose
    // with odometry.forward_speed and snapshot.steering_angle as initial state.
    // Returns Status::NoPath when the guide or costmap leaves no collision-free
    // trajectory, and propagates any non-Valid input status.
    virtual LocalPlan solve(const InputSnapshot& snapshot,
                            const odometry::OdometryEstimate& odometry,
                            const global_planner::GlobalGuide& guide,
                            map::GridMap costmap,
                            const TebConfig& config,
                            TebWorkspace& workspace) const = 0;
};

// Fixed-iteration projected finite-difference descent over the speed and
// steering sequence. Speed and steering are bounded by the placeholder limits
// every iteration, so each candidate is feasible.
class TebLocalPlanner final : public ITebLocalPlanner {
public:
    LocalPlan solve(const InputSnapshot& snapshot,
                    const odometry::OdometryEstimate& odometry,
                    const global_planner::GlobalGuide& guide,
                    map::GridMap costmap,
                    const TebConfig& config,
                    TebWorkspace& workspace) const override;
};

}  // namespace core::teb
