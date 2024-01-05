//
// Created by cc on 7/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_PATHSTATE_H
#define BOTBUSTERS_REBIRTH_PATHSTATE_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Velocity2d.h>
#include <units/angular_velocity.h>
#include <frc/trajectory/Trajectory.h>
#include <units/acceleration.h>
#include <frc/geometry/Acceleration2d.h>

#include "Math/EctoMath.h"

class TrajectoryState {
public:
    TrajectoryState();
    explicit TrajectoryState(frc::Trajectory::State state);
    TrajectoryState(units::second_t timestamp, frc::Pose2d pose, frc::Velocity2d vel, units::radians_per_second_t omega, frc::Acceleration2d accel);
    TrajectoryState(units::second_t timestamp, units::meter_t x, units::meter_t y, units::radian_t theta, units::meters_per_second_t vx, units::meters_per_second_t vy, units::radians_per_second_t omega, units::meters_per_second_squared_t ax, units::meters_per_second_squared_t ay);


    [[nodiscard]] frc::Trajectory::State toWPI() const;

    std::string toString() const;


    [[nodiscard]] TrajectoryState interpolate(const TrajectoryState &forward, double t) const;

    units::second_t timestamp{};
    frc::Pose2d pose{};
    frc::Velocity2d vel{};
    frc::Acceleration2d accel{};
    units::radians_per_second_t omega{};
};


#endif //BOTBUSTERS_REBIRTH_PATHSTATE_H
