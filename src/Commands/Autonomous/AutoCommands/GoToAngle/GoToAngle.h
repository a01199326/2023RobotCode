//
// Created by cc on 18/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_GOTOANGLE_H
#define BOTBUSTERS_REBIRTH_GOTOANGLE_H


#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <limits>

class GoToAngle : public frc2::CommandHelper<frc2::CommandBase, GoToAngle> {
public:
    GoToAngle(const std::shared_ptr<EctoSwerve> &swerve, units::radian_t angle, units::second_t waitTime = 100_ms);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;


private:
    bool isReady() const;

    std::shared_ptr<EctoSwerve> swerve;
    frc::ChassisSpeeds chassisSpeeds;

    double setPoint = 0;
    double visionOut{};
    bool isFinished{};
    double swerveAngle;

    units::radian_t lastState;
    units::second_t lastTime;
    units::radians_per_second_t stateVel;

    units::radian_t angle;

    frc::ProfiledPIDController<units::radian> visionAnglePID{7.2, 0.01, 0.0003, {4.35_rad_per_s,3.05_rad_per_s_sq}};

    frc::Timer atSetpointTimer;
    units::second_t atSetpointWaitTime;
};

#endif //BOTBUSTERS_REBIRTH_GOTOANGLE_H
