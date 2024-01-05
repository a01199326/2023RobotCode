//
// Created by cc on 7/12/23.
//

#include "ChoreoTrajectory.h"

ChoreoTrajectory::ChoreoTrajectory(const std::vector<TrajectoryState> &states) {
    this->states = states;

    TrajectoryState prevState;
    prevState.timestamp = 0_s;
    prevState.accel = {0_mps_sq, 0_mps_sq};
    for (auto& state : this->states){
        auto dt = prevState.timestamp - state.timestamp;
        state.accel = {
                (prevState.vel.X() - state.vel.X()) / dt,
                (prevState.vel.Y() - state.vel.Y()) / dt,
        };
        prevState = state;
    }
}

//https://github.com/SleipnirGroup/ChoreoSwerveBot/blob/main/src/main/java/lib/choreolib/ChoreoTrajectory.java
TrajectoryState ChoreoTrajectory::sample(units::second_t timestamp) const {
    if (timestamp < getStartState().timestamp) return getStartState();
    else if (timestamp > getEndState().timestamp) return getEndState();

    int low = 0;
    int high = states.size() - 1;

    while (low != high) {
        int mid = (low + high) / 2;
        if (states.at(mid).timestamp < timestamp) low = mid + 1;
        else high = mid;
    }

    auto behindState = states.at(low - 1);
    auto currentState = states.at(low);

    if (std::abs(currentState.timestamp.value() - behindState.timestamp.value()) < 1E-5) return currentState;
//    return currentState;
    return behindState.interpolate(currentState, timestamp.value());
}

units::second_t ChoreoTrajectory::getTotalTime() const {
    return getEndState().timestamp - getStartState().timestamp;
}

TrajectoryState ChoreoTrajectory::getStartState() const {
    return states.at(0);
}

TrajectoryState ChoreoTrajectory::getEndState() const {
    return states.at(states.size() - 1);
}

std::vector<TrajectoryState> ChoreoTrajectory::getStates() const {
    return states;
}





