//
// Created by cc on 10/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_PIDWRIST_H
#define BOTBUSTERS_REBIRTH_PIDWRIST_H

//#define USE_ARM_FEEDFORWARD

#include "Core/EctoModule/WPISubsystem.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Math/EctoMath.h"


#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/LinearFilter.h>

#include <units/current.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>


struct PIDWristConfig{
    PIDConfig pidConfig;

    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    bool invertSensor;

    double rampRate = 0.01;
    double gearReduction = 1;
    double sensorReduction = 1.3;

    units::ampere_t currentLimit = 30_A;

    bool enableForwardSoftLimit = true;
    bool enableReverseSoftLimit = true;

    units::radian_t forwardSoftLimit = 0_rad;
    units::radian_t reverseSoftLimit = 0_rad;

    double sensorOffset = 0;

    units::radians_per_second_t  maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;

    units::radians_per_second_t intakeMaxVel = 0_rad_per_s;
    units::radians_per_second_squared_t intakeMaxAccel = 0_rad_per_s_sq;

    units::radian_t pidDistTol = 1_rad;
    units::radians_per_second_t pidVelTol = 1_rad_per_s;
    frc::SimpleMotorFeedforward<units::radians> ff{0.0_V, 0.0_V / 1_rad_per_s, 0.0_V / 1_rad_per_s_sq};

};

class PIDWrist : public WPISubsystem{
public:
    explicit PIDWrist(const PIDWristConfig &config);

    void robotInit() override;
    void robotUpdate() override;

    bool getForwardLimitSwitch();
    bool getReverseLimitSwitch();

    units::radian_t getEncoderAngle();
    units::radians_per_second_t getEncoderVel();

    units::radian_t getAngle() const;
    units::radians_per_second_t getVel() const;

    void set(units::radian_t radians);

    void set(double radians){
        set(units::radian_t(radians));
    }

    bool getHome() {return hasHomed;}

    void resetController();

    void usePIDControl(bool usePID);

    void useSoftLimits(bool useSoftLimits);

    void setAnalogOffset(units::radian_t angle);

    void setCurrentLimit(units::ampere_t amps);

    bool atGoal();

    bool inTol(units::radian_t tol);

    bool inTolOf(units::radian_t tol, units::radian_t of);

    void setHomed(bool home){
        hasHomed = home;
    }

    void setVoltage(double set){manualVoltage = set;}

    void setPID(PIDConfig &config);

    units::volt_t calculateFF(units::radian_t angle, units::radians_per_second_t vel);

    void setFF(frc::SimpleMotorFeedforward<units::radians> ff);

    void updateTelemetry(double out);

    void setConstraints(const units::radians_per_second_t &maxVel, const units::radians_per_second_squared_t &maxAccel);

    void setDefaultConstraints() {
        setConstraints(config.maxVel, config.maxAccel);
    }

    PIDWristConfig getConfig(){
        return config;
    }

private:
    PIDConfig pidConfig;
    PIDWristConfig config;

    std::vector<std::shared_ptr<EctoMotor>> motors;

    bool hasHomed{false};

    std::unique_ptr<frc::ProfiledPIDController<units::radians>> pidController;
    std::unique_ptr<frc::TrapezoidProfile<units::radians>::Constraints> constraints;

    frc::SimpleMotorFeedforward<units::radians> * ff;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    bool usePID;
    double manualVoltage{0};

    units::radians_per_second_t  maxVel = 0_rad_per_s;
    units::radians_per_second_squared_t maxAccel = 0_rad_per_s_sq;

    double sensorOffset{0};
    units::radian_t goal;

    frc::LinearFilter<double> analogFilter = frc::LinearFilter<double>::SinglePoleIIR(0.055, 20_ms);



};


#endif //BOTBUSTERS_REBIRTH_PIDWRIST_H
