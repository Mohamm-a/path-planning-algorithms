// Output converter. Builds the cycle Output from the TEB local plan.
//   next_pose = trajectory[1]   (the pose after one horizon step; map frame)
//   v_cmd     = plan.v_cmd      (speed for the first horizon step)
//   delta_cmd = plan.delta_cmd  (steering for the first horizon step)
//   dt        = snapshot.dt      [FROM: AGENTS.md section 2]
//   timestamp = snapshot.timestamp
//   frame_id  = Frame::Map       [FROM: AGENTS.md section 2]
//   status    = plan status, or Fault when the plan cannot give a next pose.
//
// The commands are copied from the plan rather than re-derived from
// trajectory[0], which is the current pose and holds no command
// [FROM: user approval, Q5].
// When the plan is not Valid, next_pose, v_cmd and delta_cmd are NaN, so a
// faulted or no-path result cannot be used silently [FROM: teb_local_planner.hpp
// convention]. Angle normalization to (-pi, pi] is done by the facade.

#include "core/stages/output_converter.hpp"

#include <limits>

namespace core::output {

Output OutputConverter::convert(const teb::LocalPlan& plan,
                                const InputSnapshot& snapshot) const {
    const real nan = std::numeric_limits<real>::quiet_NaN();
    Output out{};
    out.dt = snapshot.dt;
    out.timestamp = snapshot.timestamp;
    out.frame_id = Frame::Map;

    if (plan.status != Status::Valid) {
        out.next_pose = Pose2D{nan, nan, nan};
        out.v_cmd = nan;
        out.delta_cmd = nan;
        out.status = plan.status;
        return out;
    }

    // A Valid plan must hold the start pose and at least one horizon step.
    if (plan.trajectory.data == nullptr || plan.trajectory.count < 2) {
        out.next_pose = Pose2D{nan, nan, nan};
        out.v_cmd = nan;
        out.delta_cmd = nan;
        out.status = Status::Fault;
        return out;
    }

    out.next_pose = plan.trajectory.data[1];
    out.v_cmd = plan.v_cmd;
    out.delta_cmd = plan.delta_cmd;
    out.status = Status::Valid;
    return out;
}

}  // namespace core::output
