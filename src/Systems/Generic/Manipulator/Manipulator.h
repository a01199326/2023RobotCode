//
// Created by cc on 22/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_MANIPULATOR_H
#define BOTBUSTERS_REBIRTH_MANIPULATOR_H

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/PCM/PCMManager.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Core/Robot/RobotState.h"

#include <bits/stdc++.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/LinearFilter.h>


struct ManipulatorConfig {
//    std::vector<std::shared_ptr<frc::DoubleSolenoid>> pistons;

    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    units::ampere_t currentLimit = 10_A;

    double idlePercent = 0.1;
    units::ampere_t coneCurrentThreshold = 7_A;
    units::ampere_t cubeCurrentThreshold = 35_A;

    bool breakingOnIdle = false;

    double manipulatorWidth = 0.34;

    double coneRadius = 0.05;
};



class Manipulator : public WPISubsystem{
public:

    Manipulator(const ManipulatorConfig &config);

    void robotUpdate() override;

    void set(double set, MotorControlMode controlMode = MotorControlMode::Percent);

    [[nodiscard]] bool hasPiece() const;

    void idle();

    double getDistance();

    double getPiecePose();

    void setPieceThreshold(ScorePiece piece);

//    void setMotor(double set, MotorControlMode controlMode = MotorControlMode::Percent);

//    void setPistons(bool set);
//
//    void set(double setPct, double closed);

private:


    ManipulatorConfig config;
    PCMManager &pcm = PCMManager::getInstance();

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("Manipulator");

    double current{0};
    double setPoint{0};

    std::vector<std::shared_ptr<EctoMotor>> motors;

    frc::LinearFilter<double> currentFilter = frc::LinearFilter<double>::MovingAverage(5);
    frc::LinearFilter<double> distanceFilter = frc::LinearFilter<double>::SinglePoleIIR(0.15, 20_ms);

    units::ampere_t currentPieceThreshold = 0_A;
//    std::vector<std::shared_ptr<frc::DoubleSolenoid>> pistons;

};


#endif //BOTBUSTERS_REBIRTH_MANIPULATOR_H
