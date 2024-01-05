//
// Created by cc on 7/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_TRAJECTORY_H
#define BOTBUSTERS_REBIRTH_TRAJECTORY_H

#include <units/time.h>
#include <bits/stdc++.h>
#include "Control/Path/Trajectory/TrajectoryState/TrajectoryState.h"

namespace botbusters {

    class Trajectory {
    public:
        virtual TrajectoryState sample(units::second_t timestamp) const = 0;

        virtual units::second_t getTotalTime() const = 0;

        virtual std::vector<TrajectoryState> getStates() const = 0;

        virtual TrajectoryState getStartState() const = 0;

        virtual TrajectoryState getEndState() const = 0;

        static std::vector<TrajectoryState> readJSON(const std::string &pathName);


    protected:
        std::vector<TrajectoryState> states;
        units::second_t totalTime = 0_s;


    };

}


#endif //BOTBUSTERS_REBIRTH_TRAJECTORY_H
