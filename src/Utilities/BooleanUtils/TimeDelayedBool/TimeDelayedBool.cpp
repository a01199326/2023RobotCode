//
// Created by cc on 1/11/23.
//

#include "TimeDelayedBool.h"


TimeDelayedBool::TimeDelayedBool() {;}

bool TimeDelayedBool::update(bool val, units::second_t timeout) {
    if (!old && val) {
        timer.Reset();
        timer.Start();
    }
    old = val;
    return val && timer.Get() >= timeout;
}