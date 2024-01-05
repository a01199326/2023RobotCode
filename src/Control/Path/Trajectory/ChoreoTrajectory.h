//
// Created by cc on 7/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_CHOREOTRAJECTORY_H
#define BOTBUSTERS_REBIRTH_CHOREOTRAJECTORY_H

#include "Control/Path/Trajectory/Trajectory.h"

class ChoreoTrajectory : public botbusters::Trajectory{
public:
    ChoreoTrajectory(const std::vector<TrajectoryState> &states);

    TrajectoryState sample(units::second_t timestamp) const override;

    units::second_t getTotalTime() const override;

    std::vector<TrajectoryState> getStates() const override;

    TrajectoryState getStartState() const override;

    TrajectoryState getEndState() const override;


};


#endif //BOTBUSTERS_REBIRTH_CHOREOTRAJECTORY_H
