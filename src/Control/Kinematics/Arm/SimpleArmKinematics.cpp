//
// Created by cc on 19/01/23.
//

#include "SimpleArmKinematics.h"


SimpleArmKinematics::SimpleArmKinematics(const SimpleArmKinematicsConfig &config) {
    this->config = config;
    this->config.maxExtension = 48_in + (config.chassisLength / 2);
}

ArmState SimpleArmKinematics::toArmState(const frc::Translation2d &coords) {
    units::meter_t distanceToRobotCenter = coords.Distance({0_m ,0_m}) + config.manipulatorLength;
    auto jointRelativeCoords = frc::Translation2d(coords - config.armJointToRobot);
    auto armLength = units::meter_t(std::hypot(jointRelativeCoords.X().value(), jointRelativeCoords.Y().value()));

    if (distanceToRobotCenter > config.maxExtension){
        armLength = config.maxExtension;
    }

    auto jointAngle = units::radian_t(std::acos(jointRelativeCoords.X().value() / armLength.value()));

    return {jointAngle, armLength, coords.X()};
}

frc::Translation2d SimpleArmKinematics::toCoords(const ArmState &state) {
    units::meter_t jointRelativeX = (state.maxExtension + config.chassisLength) - config.armJointToRobot.X();
    auto y = units::meter_t(state.distance.value() * std::sin(state.angle.value()));
    return{state.maxExtension, y};
}