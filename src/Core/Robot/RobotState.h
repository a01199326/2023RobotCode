//
// Created by cc on 9/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_ROBOTSTATE_H
#define BOTBUSTERS_REBIRTH_ROBOTSTATE_H

#include "Core/EctoModule/WPISubsystem.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <units/length.h>
#include <units/angle.h>

#include <bits/stdc++.h>


enum class ScoreHeight {
    LOW,
    MID,
    HIGH
};

enum class ScorePiece {
    CONE,
    CUBE,
    EMPTY,
    OVERLOAD
};

struct State{
    State(){;};
    State(units::radian_t jointAngle,
    units::radian_t wristAngle,
    units::meter_t telescopicHeight,
//    bool leaveGently = false,
    bool manipulatorPiston = false,
    double manipulatorMotor = 0){
        this->jointAngle = jointAngle;
        this->wristAngle = wristAngle;
        this->telescopicHeight = telescopicHeight;
        this->manipulatorPiston = manipulatorPiston;
        this->manipulatorMotor = manipulatorMotor;
//        this->leaveGently = leaveGently;
    }

    units::radian_t jointAngle = 0_rad;
    units::radian_t wristAngle = 0_rad;
    units::meter_t telescopicHeight = 0_m;

    bool manipulatorPiston{false};
    double manipulatorMotor = 0;
//    bool leaveGently{false};
};

#endif //BOTBUSTERS_REBIRTH_ROBOTSTATE_H
