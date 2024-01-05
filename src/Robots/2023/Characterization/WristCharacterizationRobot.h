//
// Created by terromn on 2/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_WRISTCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_WRISTCHARACTERIZATIONROBOT_H


#include "Core/EctoCharacterizationRobot.h"
#include "sysid/logging/SysIdGeneralMechanismLogger.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/filter/LinearFilter.h>

class WristCharacterizationRobot : public EctoCharacterizationRobot {
public:
    WristCharacterizationRobot();

    void robotInit() override;

    void robotUpdate() override;

    void teleopUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void disabledInit() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "wrist", 16},
        };
    }

private:

    double getAngle() const;

    double getVel() const;

    void updateTelemetry();

    double gearing = 300;
    std::shared_ptr<EctoMotor> wrist;

    double analogOffset{0.055};

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("Wrist");

    sysid::SysIdGeneralMechanismLogger logger;
    JoystickAxisExpo wristAxis{0.2, 0.2};

    frc::LinearFilter<double> velFilter = frc::LinearFilter<double>::MovingAverage(10);

};


#endif //BOTBUSTERS_REBIRTH_WRISTCHARACTERIZATIONROBOT_H
