//
// Created by cc on 23/01/23.
//

#include "PIDTelescopic.h"
#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

PIDTelescopic::PIDTelescopic(const PIDTelescopicConfig &config)
        : WPISubsystem("PIDTelescopic") {
    this->config = config;
    this->motors = config.motors;

    table = ntInstance.GetTable("PIDTelescopic");

    frc::ProfiledPIDController<units::meters>::Constraints constraints(config.maxVelocity,
                                                                       config.maxAcceleration);

    pidController = std::make_unique<frc::ProfiledPIDController<units::meters>>(
            config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints);

    pidController->SetTolerance(config.pidDistTol,
                                config.pidVelTol);

    if (motors.empty()) {
        log->error("No motors given to PIDTelescopic!!!");
        throw std::runtime_error("Motor vector is empty!");
    }

    if (config.isInverted.size() != motors.size())
        throw std::runtime_error("Motor inversion not configured!");


    for (size_t i = 0; i < motors.size(); i++) {
        const auto motor = motors[i];
        if (i == 0) {
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(heightToRad(config.reverseSoftLimit).value());
            motor->setForwardSoftLimit(heightToRad(config.forwardSoftLimit).value());
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);
            motor->invertMotor(config.isInverted[0]);
//            motor-


        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(heightToRad(config.reverseSoftLimit).value());
            motor->setForwardSoftLimit(heightToRad(config.forwardSoftLimit).value());
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);

        }
    }

    pidController->Reset(getHeight());

    masterMotorCurrent = table->GetEntry("masterMotorCurrent");
    slaveMotorCurrent = table->GetEntry("salveMotorCurrent");

    setFF(config.freeFF);

}

void PIDTelescopic::robotInit() {
    resetToZero();
}

void PIDTelescopic::setPIDConfig(const PIDConfig &pidConfig) {
    pidController->SetPID(pidConfig.p, pidConfig.i, pidConfig.d);
}

void PIDTelescopic::set(units::meter_t height) {

    goal = height;
    pidController->SetGoal(goal);
}

units::meter_t PIDTelescopic::radToHeight(units::radian_t rads) const{
    double motorRotations = rads.value() / (2.0 * M_PI);
    motorRotations /= config.gearReduction;
    return units::meter_t(config.stages * (motorRotations * (config.pulleyDiameter.value() * M_PI)));
}

units::radian_t PIDTelescopic::heightToRad(units::meter_t height) const{
    double rots = ((height.value() / config.stages) / (config.pulleyDiameter.value() * M_PI));
    rots = config.gearReduction * rots;
    return units::radian_t(rots * (2.0 * M_PI));
}

void PIDTelescopic::setVoltage(double setVoltage) { manualVoltage = setVoltage; }

bool PIDTelescopic::getLimitSwitchState() {
    return motors[0]->getReverseLimitSwitch();
}

void PIDTelescopic::setUsePID(bool usePID) {
    if (this->usePID != usePID && usePID) {
        pidController->Reset(getHeight());
    }
    this->usePID = usePID;
}

void PIDTelescopic::resetToZero() { 
    motors[0]->setSensorPosition(0);
    resetController(0.0_m);
 }

bool PIDTelescopic::atGoal() { return pidController->AtGoal(); }

bool PIDTelescopic::inTol(units::meter_t tol) {
    return std::abs(getHeight().value() - goal.value()) < tol.value();
}

units::meters_per_second_t PIDTelescopic::getMaxVel() const {
    return config.maxVelocity;
}

units::meters_per_second_squared_t PIDTelescopic::getMaxAccel() const {
    return config.maxAcceleration;
}

void PIDTelescopic::setConstraints(const units::meters_per_second_t maxSpeed,
                                   const units::meters_per_second_squared_t maxAccel) {
    frc::ProfiledPIDController<units::meters>::Constraints constraint(
            maxSpeed,
            maxAccel
            );
    pidController->SetConstraints(constraint);
    config.maxVelocity = maxSpeed;
    config.maxAcceleration = maxAccel;
}

void PIDTelescopic::setFF(frc::ElevatorFeedforward ff) {
    this->ff = &ff;
}

void PIDTelescopic::useSoftLimits(bool useSoftLimits) {
    motors[0]->enableForwardSoftLimit(useSoftLimits);
    motors[0]->enableReverseSoftLimit(useSoftLimits);
    table->GetEntry("usingSoftLimits").SetBoolean(useSoftLimits);

}

units::meter_t PIDTelescopic::getHeight() const {
    auto motorRads = units::radian_t(motors[0]->getPosition());
    return units::meter_t(radToHeight(motorRads));
}


units::meters_per_second_t PIDTelescopic::getVel() const {
    auto motorOut = units::radian_t(motors[0]->getVelocity());
    return units::meters_per_second_t(radToHeight(motorOut).value());
}

void PIDTelescopic::resetController(units::meter_t height){
    pidController->Reset(height);
}

PIDTelescopicConfig PIDTelescopic::getConfig() const {
    return config;
}

void PIDTelescopic::robotUpdate() {
//  if (gearBox->engaged()){return;}
    double out{0};
    if (usePID) {
        if (!inTol(0.02_m)){
            out = pidController->Calculate(getHeight());
//            out += ff->Calculate(pidController->GetSetpoint().velocity).value();
            motors[0]->set(std::clamp(out, -15.0, 15.0), MotorControlMode::Voltage);
        }else{
            motors[0]->set(out, MotorControlMode::Voltage);
        }
    } else {
        out = manualVoltage;
        motors[0]->set(manualVoltage, MotorControlMode::Voltage);
    }

    updateTelemetry(out);


}

void PIDTelescopic::updateTelemetry(double out) {
    table->GetEntry("PIDOut").SetDouble(out);
//    table->GetEntry("PIDGoal").SetDouble(pidController->GetGoal().position.value());
    table->GetEntry("height").SetDouble(getHeight().value());
    table->GetEntry("usePID").SetBoolean(usePID);
//    masterMotorCurrent.SetDouble(motors[0]->getCurrent());
//    slaveMotorCurrent.SetDouble(motors[1]->getCurrent());
    table->GetEntry("manualVoltage").SetDouble(manualVoltage);
//    table->GetEntry("forwardLimitSwitch").SetBoolean(motors[0]->getForwardLimitSwitch());
//    table->GetEntry("reverseLimitSwitch").SetBoolean(motors[0]->getReverseLimitSwitch());
    table->GetEntry("current").SetDouble(motors[0]->getCurrent());
    table->GetEntry("maxAcceleration").SetDouble(getMaxAccel().value());
    table->GetEntry("maxVel").SetDouble(getMaxVel().value());
    table->GetEntry("setPoint").SetDouble(goal.value());
    table->GetEntry("vel").SetDouble(getVel().value());
//    std::cout << getMaxAccel().value() << std::endl;
//    std::cout << getMaxVel().value() << std::endl;
//    table->GetEntry("debug/out").SetDouble(out);
//    table->GetEntry("debug/pid_error").SetDouble(pidController->GetPositionError().value());
//    table->GetEntry("debug/setpoint/pos").SetDouble(pidController->GetSetpoint().position.value());
//    table->GetEntry("debug/setpoint/vel").SetDouble(pidController->GetSetpoint().velocity.value());
//    table->GetEntry("debug/velocity_error").SetDouble(pidController->GetVelocityError().value());
}