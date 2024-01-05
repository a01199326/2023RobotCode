//
// Created by cc on 6/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_JOINTCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_JOINTCHARACTERIZATIONROBOT_H


#include "Core/EctoCharacterizationRobot.h"
#include "sysid/logging/SysIdGeneralMechanismLogger.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/filter/LinearFilter.h>

class JointCharacterizationRobot : public EctoCharacterizationRobot {
public:
    JointCharacterizationRobot();

    void robotInit() override;

    void robotUpdate() override;

    void teleopUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void disabledInit() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "arm",                    13},
                {EctoMotorType::SparkMax, "armFollower",            14},
        };
    }

private:

    void updateTelemetry();

    double gearing = 214.38;
    std::shared_ptr<EctoMotor> arm, armFollower;

    double analogOffset = 0.909;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("arm");

    sysid::SysIdGeneralMechanismLogger logger;
    JoystickAxisExpo wristAxis{0.2, 0.2};

//    frc::LinearFilter<double> velFilter = frc::LinearFilter<double>::MovingAverage(10);

};


#endif //BOTBUSTERS_REBIRTH_JOINTCHARACTERIZATIONROBOT_H
