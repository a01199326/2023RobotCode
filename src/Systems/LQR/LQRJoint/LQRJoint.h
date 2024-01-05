//
// Created by cc on 27/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_LQRJOINT_H
#define BOTBUSTERS_REBIRTH_LQRJOINT_H

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/MotorHandler/MotorManager.h"

#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>


#include <units/voltage.h>
#include <units/current.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

struct LQRJointConfig{
    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    units::volt_t kV;
    units::volt_t kA;

    double rampRate = 0.01;
    double gearReduction = 1;

    units::ampere_t currentLimit = 30_A;

    bool enableForwarSoftLimit = true;
    bool enableReverseSoftLimit = true;

    units::radian_t forwarSoftLimit = 0_rad;
    units::radian_t reverseSoftLimit = 0_rad;

    units::radian_t analogOffset = 0_rad;

    units::radians_per_second_t maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAcceleration;

    units::radian_t distTol = 1_rad;
    units::radians_per_second_t velTol = 1_rad_per_s;
};


class LQRJoint : public WPISubsystem {
public:
    LQRJoint(const LQRJointConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    bool getForwardLimitSwitch();
    bool getReverseLimitSwitch();

    units::radian_t getAngle();
    units::radians_per_second_t getVel();

    void set(units::radian_t radians);

    void set(double radians){
        set(units::radian_t(radians));
    }

    bool getHome() {return hasHomed;}

    void resetController();

    void useLQRControl(bool use);

    void useSoftLimits(bool useSoftLimits);

    void setAnalogOffset(units::radian_t angle);

    void setCurrentLimit(units::ampere_t amps);

    bool atGoal();

    void setHomed(bool home){
        hasHomed = home;
    }

    void setVoltage(double set){manualVoltage = set;}

private:
    void updateTelemetry();

    std::vector<std::shared_ptr<EctoMotor>> motors;
    LQRJointConfig config;

    units::radian_t analogOffset;

    bool hasHomed{false};
    bool useLQR{true};

    double manualVoltage;
    double out;
    units::radian_t goal;

    frc::LinearSystem<2,1,1> plant;
    frc::KalmanFilter<2,1,1> observer;
    frc::LinearSystemLoop<2,1,1> loop;
    frc::LinearQuadraticRegulator<2,1> controller;
    frc::LinearPlantInversionFeedforward<2,1> ff;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("LQRJoint");

    Eigen::Vector<double, 1> u;
    Eigen::Matrix<double, 2, 1> error = {99999999.0, 999999.0};

};


#endif //BOTBUSTERS_REBIRTH_LQRJOINT_H
