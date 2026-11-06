// Odometry stage interface. Fuses encoder RPM and gyro yaw rate into
// predicted motion [FROM: AGENTS.md section 2]. Implementation: src/odometry.cpp.
// Steering angle is not used in this stage (no Ackermann yaw yet).

#pragma once

#include "core/core.hpp"
#include "core/placeholders.hpp"

namespace core::odometry {

// Predicted motion for one control cycle. All numeric fields are NaN when
// status is Status::Fault, so a faulted estimate cannot be used silently.
struct OdometryEstimate {
    Pose2D predicted_pose;  // m, m, rad; map frame
    real forward_speed;     // m/s; signed (negative = reverse)
    real yaw_rate;          // rad/s; positive is CCW
    Status status = Status::Fault;  // no-silent-fallback default
};

// Odometry stage interface.
class IOdometry {
public:
    virtual ~IOdometry() = default;

    // Predict motion from the snapshot's encoder, gyro and pose inputs.
    virtual OdometryEstimate predict(const InputSnapshot& snapshot) const = 0;
};

// Encoder + gyro odometry. Stateless: all inputs come from the snapshot.
class EncoderGyroOdometry final : public IOdometry {
public:
    OdometryEstimate predict(const InputSnapshot& snapshot) const override;
};

}  // namespace core::odometry
