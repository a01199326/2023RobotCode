//
// Created by cc on 29/04/23.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVESTEERTEST_H
#define BOTBUSTERS_REBIRTH_SWERVESTEERTEST_H

#include "Core/EctoRobot.h"

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/EctoSwerve/EctoSwerveInputHandler.h"

#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>

class SwerveSteerTest : public EctoRobot{
public:
    SwerveSteerTest();

    void disabledInit() override;
    void robotInit() override;
    void robotUpdate() override;
    void teleopUpdate() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "flSteer",  8},
                {EctoMotorType::SparkMax, "frSteer", 6},
                {EctoMotorType::SparkMax, "blSteer",   2},
                {EctoMotorType::SparkMax, "brSteer",  4},
                {EctoMotorType::SparkMax, "flWheel",     7},
                {EctoMotorType::SparkMax, "frWheel",    5},
                {EctoMotorType::SparkMax, "blWheel",      1},
                {EctoMotorType::SparkMax, "brWheel",     3},
        };
    };

    std::shared_ptr<SwerveModule> sModule;

    std::shared_ptr<EctoMotor> motor;

    double analogOffset{-1.155}, gearReduction{5.19223076921}, wheelCircumference{0.0508 * 2 * M_PI};

    double kp{0.0}, ki{0.0}, kd{0.0}, kf{0.0};

};


#endif //BOTBUSTERS_REBIRTH_SWERVESTEERTEST_H
