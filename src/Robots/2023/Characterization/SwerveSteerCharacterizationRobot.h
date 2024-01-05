//
// Created by abiel on 11/3/21.
//

#pragma once

#include "Core/EctoCharacterizationRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

#include "sysid/logging/SysIdGeneralMechanismLogger.h"

#include <frc/smartdashboard/SmartDashboard.h>

class SwerveSteerCharacterizationRobot : public EctoCharacterizationRobot {
public:
	SwerveSteerCharacterizationRobot();
	
	void robotInit() override;
	void robotUpdate() override;
	void teleopUpdate() override;
	void autoInit() override;
	void autoUpdate() override;
	void disabledInit() override;

private:
    double getPose() const;
    double getVel() const;

protected:
	std::list<MotorInfo> getMotorConfig() override {
		return {
		        {EctoMotorType::SparkMax, "fl",  8},
		        {EctoMotorType::SparkMax, "fr", 6},
		        {EctoMotorType::SparkMax, "bl",   2},
		        {EctoMotorType::SparkMax, "br",  4},
		};
	};

    JoystickAxisExpo manualControl{0.2, 0.2};
	
	std::shared_ptr<EctoMotor> steerMotor;
	
	sysid::SysIdGeneralMechanismLogger logger;
};