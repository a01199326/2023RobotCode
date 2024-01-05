//
// Created by terromn on 30/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_TELESCOPICCHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_TELESCOPICCHARACTERIZATIONROBOT_H


#include "Core/EctoCharacterizationRobot.h"
#include "sysid/logging/SysIdGeneralMechanismLogger.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"

class TelescopicCharacterizationRobot : public EctoCharacterizationRobot {
public:
    TelescopicCharacterizationRobot();

    void robotInit() override;

    void robotUpdate() override;

    void teleopUpdate() override;

    void autoInit() override;

    void autoUpdate() override;

    void disabledInit() override;

protected:
    std::list<MotorInfo> getMotorConfig() override {
        return {
                {EctoMotorType::SparkMax, "telescopic", 10},
                {EctoMotorType::SparkMax, "telescopicFollower", 11},
                {EctoMotorType::SparkMax, "telescopicSecondFollower", 12},

        };
    }

private:

    double gearRatioTelescopic = 8.444444444;
    std::shared_ptr<EctoMotor> telescopic, telescopic1, telescopic2;

    sysid::SysIdGeneralMechanismLogger logger;
    JoystickAxisExpo telescopicAxis{0.2, 0.2};

};


#endif //BOTBUSTERS_REBIRTH_TELESCOPICCHARACTERIZATIONROBOT_H
