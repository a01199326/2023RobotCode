//
// Created by cc on 7/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_WPITRAJECTORY_H
#define BOTBUSTERS_REBIRTH_WPITRAJECTORY_H

#include <frc/trajectory/Trajectory.h>
#include "Control/Path/Trajectory/Trajectory.h"

class WPITrajectory : public botbusters::Trajectory{
public:
    WPITrajectory(const std::vector<frc::Trajectory::State> &states);

    WPITrajectory(const frc::Trajectory& trajectory);

    TrajectoryState sample(units::second_t timestamp) const override;

    units::second_t getTotalTime() const override;

    std::vector<TrajectoryState> getStates() const override;

    TrajectoryState getStartState() const override;

    TrajectoryState getEndState() const override;

    std::vector<frc::Trajectory::State> getWPIStates() const;

    frc::Trajectory getTraj() const {
        return traj;
    }

private:
    frc::Trajectory traj;
};


#endif //BOTBUSTERS_REBIRTH_WPITRAJECTORY_H
