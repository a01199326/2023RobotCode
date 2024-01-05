//
// Created by cc on 19/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_ARMSTATE_H
#define BOTBUSTERS_REBIRTH_ARMSTATE_H

#include <units/angle.h>
#include <units/length.h>

struct ArmState{
    ArmState(units::radian_t angle, units::meter_t distance, units::meter_t maxExtension){
        this->distance = distance;
        this->angle = angle;
    }
    units::radian_t angle;
    units::meter_t distance;
    units::meter_t maxExtension;
};


#endif //BOTBUSTERS_REBIRTH_ARMSTATE_H
