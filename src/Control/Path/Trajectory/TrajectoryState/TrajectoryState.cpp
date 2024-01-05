//
// Created by cc on 7/12/23.
//

#include "TrajectoryState.h"

TrajectoryState::TrajectoryState(frc::Trajectory::State state) :
        pose(state.pose),
        timestamp(state.t),
        accel(1_mps_sq, units::meters_per_second_squared_t(std::copysign(std::sqrt(std::abs(state.acceleration.value()) - 1), state.acceleration.value())))
        // in this specific case we only care about the magnitude,
        // which is what we are storing in y asuming that x is 1

        {}

TrajectoryState::TrajectoryState(
        units::second_t timestamp,
        frc::Pose2d pose,
        frc::Velocity2d vel,
        units::radians_per_second_t omega,
        frc::Acceleration2d accel) :
        timestamp(timestamp),
        pose(pose),
        vel(vel),
        omega(omega),
        accel(accel)
        {}

TrajectoryState::TrajectoryState(
        units::second_t timestamp,
        units::meter_t x,
        units::meter_t y,
        units::radian_t theta,
        units::meters_per_second_t vx,
        units::meters_per_second_t vy,
        units::radians_per_second_t omega,
        units::meters_per_second_squared_t ax,
        units::meters_per_second_squared_t ay) :
        timestamp(timestamp),
        pose(x, y, {theta}),
        vel(vx, vy),
        omega(omega),
        accel(ax, ay)
        {}

TrajectoryState::TrajectoryState() {;}

TrajectoryState TrajectoryState::interpolate(const TrajectoryState &forward, double t) const {
    double scale = (t - timestamp.value()) / (forward.timestamp - timestamp).value();
    TrajectoryState out;
    out.pose = EctoMath::interpolate(pose, forward.pose, scale);
    out.vel = {
            units::meters_per_second_t(EctoMath::interpolate(vel.X().value(), forward.vel.X().value(), scale)),
            units::meters_per_second_t(EctoMath::interpolate(vel.Y().value(), forward.vel.Y().value(), scale))
    };
    out.omega = units::radians_per_second_t(EctoMath::interpolate(omega.value(), forward.omega.value(), scale));
    out.accel = {
            units::meters_per_second_squared_t (EctoMath::interpolate(accel.X().value(), forward.accel.X().value(), scale)),
            units::meters_per_second_squared_t(EctoMath::interpolate(accel.Y().value(), forward.accel.Y().value(), scale))
    };
    return out;
}

frc::Trajectory::State TrajectoryState::toWPI() const{
    frc::Trajectory::State out;
    out.t = this->timestamp;
    out.pose = this->pose;
    out.velocity = this->vel.Norm();
    return out;
}

std::string TrajectoryState::toString() const {
    return fmt::format(
            "timestamp: {}, x: {}, y: {}, heading: {}, vx: {}, vy: {}, omega: {}",
            timestamp.value(),
            pose.X().value(),
            pose.Y().value(),
            pose.Rotation().Degrees().value(),
            vel.X().value(),
            vel.Y().value(),
            omega.value()
            );
}