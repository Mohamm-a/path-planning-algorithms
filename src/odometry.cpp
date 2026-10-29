// Encoder + gyro odometry. Per control cycle, with dt from the snapshot:
//   v         = encoder_rpm * (2*pi/60) * WHEEL_RADIUS_M   [m/s, signed]
//   omega     = gyro_yaw_rate                                      [rad/s]
//   theta_mid = theta + omega * dt / 2
//   x'        = x + v * cos(theta_mid) * dt
//   y'        = y + v * sin(theta_mid) * dt
//   theta'    = wrap(theta + omega * dt)                           (-pi, pi]
//
// [FROM: AGENTS.md section 5, units and sign conventions]
// [FROM: unit definition, rpm to rad/s = 2*pi/60]
// [ASSUMED: encoder_rpm is the rotation rate of the drive wheel, signed,
//  negative = reverse]
// Yaw comes from the gyro only. Steering angle and wheelbase are not used
// [FROM: Phase A plan approval].
// Wheel radius is PH-0009 [UNKNOWN]. Guard: any non-positive or non-finite
// input, or an unreplaced wheel radius, returns Status::Fault with NaN fields.

#include "core/stages/odometry.hpp"

#include <cmath>
#include <limits>

namespace core::odometry {
namespace {

constexpr double kPi = 3.14159265358979323846;  // [FROM: mathematical constant]
constexpr double kRpmToRadPerS = 2.0 * kPi / 60.0;  // [FROM: unit definition]

// Wraps an angle to (-pi, pi].
double wrap_angle(double angle) {
    double t = std::fmod(angle + kPi, 2.0 * kPi);
    if (t <= 0.0) {
        t += 2.0 * kPi;
    }
    return t - kPi;
}

bool snapshot_is_finite(const InputSnapshot& s) {
    return std::isfinite(s.pose.x) && std::isfinite(s.pose.y) &&
           std::isfinite(s.pose.theta) && std::isfinite(s.encoder_rpm) &&
           std::isfinite(s.gyro_yaw_rate) && std::isfinite(s.dt);
}

}  // namespace

OdometryEstimate EncoderGyroOdometry::predict(const InputSnapshot& snapshot) const {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const OdometryEstimate fault{{nan, nan, nan}, nan, nan, Status::Fault};

    // No silent fallback: an unreplaced wheel radius or invalid input faults.
    if (!(WHEEL_RADIUS_M > 0.0)) {
        return fault;
    }
    if (!snapshot_is_finite(snapshot) || !(snapshot.dt > 0.0)) {
        return fault;
    }

    const double dt = snapshot.dt;
    const double omega = snapshot.gyro_yaw_rate;
    const double v = snapshot.encoder_rpm * kRpmToRadPerS * WHEEL_RADIUS_M;

    const double theta = snapshot.pose.theta;
    const double theta_mid = theta + omega * dt / 2.0;

    OdometryEstimate out;
    out.predicted_pose.x = snapshot.pose.x + v * std::cos(theta_mid) * dt;
    out.predicted_pose.y = snapshot.pose.y + v * std::sin(theta_mid) * dt;
    out.predicted_pose.theta = wrap_angle(theta + omega * dt);
    out.forward_speed = v;
    out.yaw_rate = omega;
    out.status = Status::Valid;
    return out;
}

}  // namespace core::odometry
