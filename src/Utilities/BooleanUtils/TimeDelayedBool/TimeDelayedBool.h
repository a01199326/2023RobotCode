//
// Created by cc on 1/11/23.
//

#ifndef BOTBUSTERS_REBIRTH_TIMEDELAYEDBOOL_H
#define BOTBUSTERS_REBIRTH_TIMEDELAYEDBOOL_H

#include <frc/Timer.h>

//https://github.com/Team254/FRC-2023-Public/tree/main/src/main/java/com/team254/lib/util
class TimeDelayedBool {
public:
    TimeDelayedBool();

    bool update(bool val, units::second_t timeout);

    bool update(bool val, double timeout){
        return update(val, units::second_t(timeout));
    }

private:
    frc::Timer timer{};
    bool old = false;
};


#endif //BOTBUSTERS_REBIRTH_TIMEDELAYEDBOOL_H
