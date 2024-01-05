//
// Created by cc on 18/02/23.
//

#include "GoToAngle.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

GoToAngle::GoToAngle(const std::shared_ptr<EctoSwerve> &swerve,
                         units::radian_t angle, units::second_t waitTime) {
    this->swerve = swerve;
    atSetpointWaitTime = waitTime;
    this->angle = angle;

    AddRequirements(swerve.get());
}

void GoToAngle::Initialize() {
    auto pi = units::radian_t(M_PI);
    visionAnglePID.EnableContinuousInput(-pi, pi);
    visionAnglePID.Reset(units::radian_t(swerve->getYaw()));
    visionAnglePID.SetTolerance(0.05_rad);
    lastState = 0_rad;
    lastTime = frc::Timer::GetFPGATimestamp();

    atSetpointTimer.Reset();
    atSetpointTimer.Start();
}

//Switch to using pose estimated vision alignment
void GoToAngle::Execute() {
    auto dt = frc::Timer::GetFPGATimestamp() - lastTime;
    auto state = units::radian_t(swerve->getYaw());
    if(state != lastState)
        stateVel = (state - lastState) / dt;

    visionOut = visionAnglePID.Calculate(state, angle);
    chassisSpeeds.omega = units::radians_per_second_t(visionOut);
    swerve->setVoltage(chassisSpeeds);

    if(!isReady()){
        atSetpointTimer.Reset();
    }

    lastTime = frc::Timer::GetFPGATimestamp();
    lastState = state;
}

void GoToAngle::End(bool interrupted) {
    chassisSpeeds.omega = units::radians_per_second_t(0);
    swerve->setPercent(chassisSpeeds);
    atSetpointTimer.Stop();

}

bool GoToAngle::isReady() const {
    double state = 0;
//    std::cout << fmt::format("Setpoint: {}, State: {}, Error: {} Tol: {} StateVel: {}", setPoint, state, std::abs(setPoint - state), tol, stateVel.value()) << std::endl;

//    auto atSetpoint = std::abs(setPoint - state) < posTol;
//    auto stateVelStab = std::abs(stateVel.value()) < velTol;

//    return atSetpoint && stateVelStab;
    return visionAnglePID.AtGoal();
}

bool GoToAngle::IsFinished() {
//    return isReady() and atSetpointTimer.Get() > atSetpointWaitTime;
    return isReady();
}