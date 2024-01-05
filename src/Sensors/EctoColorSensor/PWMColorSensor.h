//
// Created by cc on 11/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_PWMCOLORSENSOR_H
#define BOTBUSTERS_REBIRTH_PWMCOLORSENSOR_H

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/Robot/RobotState.h"
#include "Core/EctoInput/InputManager.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"

#include <frc/DigitalInput.h>
#include <frc/SynchronousInterrupt.h>


struct PWMColorSensorConfig{
    int DIOPin = 9;
    double pulsePeriod = 0;
    double bound = 0;
    double coneThreshold = 0;
    double cubeThreshold = 0;
};

class PWMColorSensor : WPISubsystem{
public:
    explicit PWMColorSensor(const PWMColorSensorConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    double getRatio();

    static bool inRange(double in, double to, double tol);

    ScorePiece getDetectedPiece();

    [[nodiscard]] bool hasPiece();

    ScorePiece getCurrentPiece();

private:
    InputManager &input = InputManager::getInstance();

    JoystickAxisExpo overrideCone{0.2, 0.2}, overrideCube{0.2, 0.2};

    [[nodiscard]] double adjustRatio(double value) const;
    std::shared_ptr<frc::SynchronousInterrupt> interrupt;
    PWMColorSensorConfig config;

    double prevRatio{0.5};

    ScorePiece prevScorePiece, currentPiece{ScorePiece::CONE};
};


#endif //BOTBUSTERS_REBIRTH_PWMCOLORSENSOR_H
