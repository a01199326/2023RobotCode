//
// Created by cc on 19/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SIMPLEARMKINEMATICS_H
#define BOTBUSTERS_REBIRTH_SIMPLEARMKINEMATICS_H

#include <units/length.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>

#include <bits/stdc++.h>

#include "ArmState.h"

struct SimpleArmKinematicsConfig{
    units::meter_t armLengths;
    units::meter_t maxExtension;
    frc::Translation2d armJointToRobot;
    units::meter_t manipulatorLength;
    units::meter_t chassisLength; //with bumper
};

class SimpleArmKinematics {
public:
    SimpleArmKinematics(const SimpleArmKinematicsConfig& config);

    // the coords are from the center of the robot center as odometry
    //x is away from the robot and y is up from the robot
    ArmState toArmState(const frc::Translation2d &coords);

    frc::Translation2d toCoords(const ArmState &state);

private:
    SimpleArmKinematicsConfig config;

};


#endif //BOTBUSTERS_REBIRTH_SIMPLEARMKINEMATICS_H
