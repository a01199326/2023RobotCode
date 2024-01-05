//
// Created by cc on 27/04/23.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVEDRIVEVELCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_SWERVEDRIVEVELCHARACTERIZATIONROBOT_H

#include "Core/EctoRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Core/EctoCharacterizationRobot.h"
#include <sysid/logging/SysIdGeneralMechanismLogger.h>


class SwerveDriveVelCharacterizationRobot : public EctoCharacterizationRobot{
public:
    SwerveDriveVelCharacterizationRobot();

    void robotInit() override;
    void robotUpdate() override;
    void teleopUpdate() override;
    void autoInit() override;
    void autoUpdate() override;
    void disabledInit() override;

private:
    double gearRatio = 6.92;
    double wheelCircumference = 0.0508 * 2 * M_PI;

    double getMotorPose() const;
    double getMotorVel() const;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return{
                {EctoMotorType::SparkMax, "flDrive", 7},
                {EctoMotorType::SparkMax, "frDrive", 5},
                {EctoMotorType::SparkMax, "blDrive", 1},
                {EctoMotorType::SparkMax, "brDrive", 3},

        };
    };

    JoystickAxisExpo manualAxis{0.2, 0.2};

    std::shared_ptr<EctoMotor> driveMotor;

    sysid::SysIdGeneralMechanismLogger logger;
};


#endif //BOTBUSTERS_REBIRTH_SWERVEDRIVEVELCHARACTERIZATIONROBOT_H
