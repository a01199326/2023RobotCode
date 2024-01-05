//
// Created by cc on 7/12/23.
//

#include "WPITrajectory.h"

WPITrajectory::WPITrajectory(const frc::Trajectory& trajectory) {
    this->traj = trajectory;
    for (auto &state: trajectory.States()) {
        this->states.emplace_back(state);
    }
}
WPITrajectory::WPITrajectory(const std::vector<frc::Trajectory::State> &states) {
    this->traj = frc::Trajectory(states);
    for (auto &state: traj.States()) {
        this->states.emplace_back(state);
    }
}

TrajectoryState WPITrajectory::sample(units::second_t timestamp) const {
    return TrajectoryState(traj.Sample(timestamp));
}

units::second_t WPITrajectory::getTotalTime() const {
    return traj.TotalTime();
}

std::vector<TrajectoryState> WPITrajectory::getStates() const{
    return states;
}

TrajectoryState WPITrajectory::getStartState() const {
    return states.at(0);
}

TrajectoryState WPITrajectory::getEndState() const {
    return states.at(states.size() - 1);
}

std::vector<frc::Trajectory::State> WPITrajectory::getWPIStates() const {
    return traj.States();
}


