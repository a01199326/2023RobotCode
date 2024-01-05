//
// Created by cc on 3/11/22.
//

#ifndef BOTBUSTERS_REBIRTH_PIDJOINT_H
#define BOTBUSTERS_REBIRTH_PIDJOINT_H

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Math/EctoMath.h"


#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/LinearFilter.h>
#include <frc/estimator/KalmanFilter.h>

#include <units/current.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

struct PIDJointConfig{
    PIDConfig pidConfig;
    std::vector<std::shared_ptr<EctoMotor>> motors;

    std::vector<bool> isInverted;
    bool invertSensor = false;

    double rampRate = 0.01;
    double gearReduction = 1;// in/out

    units::ampere_t currentLimit = 30_A;

    bool enableForwardSoftLimit = true;
    bool enableReverseSoftLimit = true;

    units::radian_t forwardSoftLimit = 0_rad;
    units::radian_t reverseSoftLimit = 0_rad;

    double analogOffset = 0;

//    MotorFeedbackMode motorFeedbackMode = MotorFeedbackMode::Potentiometer;

    units::radians_per_second_t maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;

    units::radian_t pidDistTol = 1_rad;
    units::radians_per_second_t pidVelTol = 1_rad_per_s;

    frc::SimpleMotorFeedforward<units::radians> ff{0.0_V, 0.0_V / 1_rad_per_s, 0.0_V / 1_rad_per_s_sq};

};


class PIDJoint : public WPISubsystem{
public:
    PIDJoint(const PIDJointConfig &config);

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

    void usePIDControl(bool usePID);

    void useSoftLimits(bool useSoftLimits);

    void setAnalogOffset(units::radian_t angle);

    bool atGoal();

    bool inTol(units::radian_t tol);

    bool inTolOf(units::radian_t tol, units::radian_t of);

    units::radian_t getGoal() {return goal;};

    void setHomed(bool home){
        hasHomed = home;
    }

    void setVoltage(double set){manualVoltage = set;}

    units::radians_per_second_t getMaxVel();

    units::radians_per_second_squared_t getMaxAccel();

    void setPID(PIDConfig &config);

    void setConstraints(units::radians_per_second_t maxSpeed, units::radians_per_second_squared_t maxAccel);

    units::volt_t calculateFF(units::radian_t angle, units::radians_per_second_t vel);

    void setFF(frc::SimpleMotorFeedforward<units::radians> ff);


private:

    void updateTelemetry(double out);


    std::vector<std::shared_ptr<EctoMotor>> motors;
    PIDConfig pidConfig;
    PIDJointConfig config;

    double analogOffset;



    std::unique_ptr<frc::ProfiledPIDController<units::radians>> pidController;
    std::unique_ptr<frc::TrapezoidProfile<units::radians>::Constraints> constraints;

    frc::SimpleMotorFeedforward<units::radians> * ff;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("PIDJoint");

    bool hasHomed{false};
    bool usePID{true};
    double manualVoltage{0};
    units::radian_t goal{};


    frc::LinearFilter<double> analogFilter = frc::LinearFilter<double>::SinglePoleIIR(0.045, 20_ms);

};


#endif //BOTBUSTERS_REBIRTH_PIDJOINT_H
