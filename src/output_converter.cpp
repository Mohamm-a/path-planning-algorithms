// Output converter. Builds the cycle Output from the TEB local plan.
//   next_pose = trajectory[1]   (the pose after one horizon step; map frame)
//   dt        = snapshot.dt      [FROM: AGENTS.md section 2]
//   timestamp = snapshot.timestamp
//   frame_id  = Frame::Map       [FROM: AGENTS.md section 2]
//   status    = plan status, or Fault when the plan cannot give a next pose.
//
// [ASSUMED: trajectory is start-first, per teb_local_planner.hpp]
// When the plan is not Valid, next_pose is NaN, so a faulted or no-path
// result cannot be used silently [FROM: teb_local_planner.hpp convention].
// Angle normalization to (-pi, pi] is done by the facade, not here.

#include "core/stages/output_converter.hpp"

#include <limits>

namespace core::output {

Output OutputConverter::convert(const teb::LocalPlan& plan,
                                const InputSnapshot& snapshot) const {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Output out{};
    out.dt = snapshot.dt;
    out.timestamp = snapshot.timestamp;
    out.frame_id = Frame::Map;

    if (plan.status != Status::Valid) {
        out.next_pose = Pose2D{nan, nan, nan};
        out.status = plan.status;
        return out;
    }

    // A Valid plan must hold the start pose and at least one horizon step.
    if (plan.trajectory.size() < 2) {
        out.next_pose = Pose2D{nan, nan, nan};
        out.status = Status::Fault;
        return out;
    }

    out.next_pose = plan.trajectory[1];
    out.status = Status::Valid;
    return out;
}

}  // namespace core::output
