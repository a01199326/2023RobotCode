//
// Created by cc on 10/01/23.
//

#include "PIDWrist.h"
#include <iostream>

PIDWrist::PIDWrist(const PIDWristConfig &config) : WPISubsystem("PIDWrist"){
    this->config = config;
    this->motors = config.motors;

    table = ntInstance.GetTable("PIDWrist");

    frc::ProfiledPIDController<units::radians>::Constraints constraints(
            config.maxVel,
            config.maxAccel
            );
    pidController = std::make_unique<frc::ProfiledPIDController<units::radians>>(
            config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints
            );

    pidController->SetTolerance(config.pidDistTol, config.pidVelTol);

    if (motors.empty()){
        log->error("No motors given to PIDWrist");
        throw std::runtime_error("Motor vector is empty!");
    }

    if (config.isInverted.size() != motors.size()){
        log->error("Motors inversion not configured!");
        throw std::runtime_error("Motor inversion not configured in PIDWrist");
    }

    for (size_t i = 0; i < motors.size(); i++) {
        const auto motor = motors[i];
        if (i == 0) {
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeAbsUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(config.forwardSoftLimit.value() * config.gearReduction);
            motor->setForwardSoftLimit(config.reverseSoftLimit.value() * config.gearReduction);
            motor->enableForwardSoftLimit(false);
            motor->enableReverseSoftLimit(false);
            motor->enableLimitSwitches(false);
            motor->invertMotor(true);
            motor->setFeedbackMode(MotorFeedbackMode::AbsoluteEncoder);
            motor->setAbsolutePositionConversionFactor(2 * M_PI);
            motor->setAbsoluteVelocityConversionFactor(2 * M_PI);
            motor->invertSensor(config.invertSensor);
            motor->setAbsoluteSensorOffset(config.sensorOffset);
            sensorOffset = config.sensorOffset;
            motor->burnFlash();


        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(config.forwardSoftLimit.value() * config.gearReduction);
            motor->setForwardSoftLimit(config.reverseSoftLimit.value() * config.gearReduction);
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);
            motor->burnFlash();

        }

    }

    pidController->Reset(getAngle());
    setFF(config.ff);

}

void PIDWrist::robotInit(){
    motors[0]->setSensorPosition(getAngle().value() * config.gearReduction);
}

bool PIDWrist::getForwardLimitSwitch() {
    return motors[0]->getForwardLimitSwitch();
}

bool PIDWrist::getReverseLimitSwitch() {
    return motors[0]->getReverseLimitSwitch();
}

units::radian_t PIDWrist::getAngle() const {
    double pose = motors[0]->getPosition() / config.sensorReduction;
//    pose = analogFilter.Calculate(pose);
    return units::radian_t(pose);
}

units::radians_per_second_t PIDWrist::getVel() const {
    return units::radians_per_second_t(motors[0]->getVelocity() / config.sensorReduction);
}


units::radian_t PIDWrist::getEncoderAngle() {
    double pos = motors[0]->getMotorEncoderPos() / config.gearReduction;
    pos = EctoMath::wrapAngle(pos);
    return units::radian_t(pos);
}

units::radians_per_second_t PIDWrist::getEncoderVel() {
    double pos = motors[0]->getMotorEncoderVel() / config.gearReduction;
    pos = EctoMath::wrapAngle(pos);
    return units::radians_per_second_t(pos);
}

void PIDWrist::set(units::radian_t radians) {
    double wrapOut = EctoMath::wrapAngle(radians.value());
    double clamped = std::clamp(wrapOut, config.reverseSoftLimit.value(), config.forwardSoftLimit.value());
    goal = units::radian_t(clamped);
    pidController->SetGoal(goal);
}

void PIDWrist::resetController() {
    pidController->Reset(getAngle());
}

void PIDWrist::usePIDControl(bool usePID) {
    this->usePID = usePID;
}

void PIDWrist::useSoftLimits(bool useSoftLimits) {
    for (auto &motor : motors){
        motor->enableForwardSoftLimit(useSoftLimits);
        motor->enableReverseSoftLimit(useSoftLimits);
    }
}

void PIDWrist::setAnalogOffset(units::radian_t angle) {
    motors[0]->setAnalogSensorOffset(angle.value());
    sensorOffset = angle.value();
}

void PIDWrist::setCurrentLimit(units::ampere_t amps) {
    for (auto &motor : motors){
        motor->setMotorCurrentLimit(amps.value());
        motor->enableCurrentLimit(true);
    }
}

bool PIDWrist::atGoal() {
    return pidController->AtGoal();
}

bool PIDWrist::inTol(units::radian_t tol) {
    return std::abs(getAngle().value() - goal.value()) < tol.value();
}

bool PIDWrist::inTolOf(units::radian_t tol, units::radian_t of) {
    return std::abs(getAngle().value() - of.value()) < tol.value();
}

void PIDWrist::setPID(PIDConfig &config) {
    pidController->SetPID(config.p, config.i, config.d);
}

void PIDWrist::setFF(frc::SimpleMotorFeedforward<units::radians> ff) {
    this->ff = &ff;
}

units::volt_t PIDWrist::calculateFF(units::radian_t angle, units::radians_per_second_t vel) {
    return ff->Calculate(vel);
}

void PIDWrist::setConstraints(const units::radians_per_second_t &maxVel,
                              const units::radians_per_second_squared_t &maxAccel) {
    this->maxVel = maxVel;
    this->maxAccel = maxAccel;
    pidController->SetConstraints({maxVel, maxAccel});
}

void PIDWrist::updateTelemetry(double out) {
    table->GetEntry("PIDOut").SetDouble(out);
    table->GetEntry("angle").SetDouble(getAngle().value());
//    table->GetEntry("Encoder/Pos").SetDouble(getEncoderAngle().value());
//    table->GetEntry("Encoder/Vel").SetDouble(getEncoderVel().value());
    table->GetEntry("goal").SetDouble(goal.value());
    table->GetEntry("Vel").SetDouble(getVel().value());
    table->GetEntry("usePID").SetBoolean(usePID);
    table->GetEntry("manualVoltage").SetDouble(manualVoltage);
//    table->GetEntry("forwardLimitSwitch").SetBoolean(motors[0]->getForwardLimitSwitch());
//    table->GetEntry("reverseLimitSwitch").SetBoolean(motors[0]->getReverseLimitSwitch());
    table->GetEntry("current").SetDouble(motors[0]->getCurrent());
    table->GetEntry("constraints/maxVel").SetDouble(maxVel.value());
    table->GetEntry("constraints/maxAccel").SetDouble(maxAccel.value());
    auto analogPos = getAngle().value() * config.sensorReduction;
    analogPos += sensorOffset;
    analogPos = EctoMath::wrapAngle(analogPos);
    table->GetEntry("RawAnalog").SetDouble(analogPos);
}

void PIDWrist::robotUpdate() {
    double out{0};
    if (!inTol(0.008_rad)){
        out = pidController->Calculate(getAngle());
//        out += ff->Calculate(pidController->GetSetpoint().velocity).value();
        motors[0]->set(std::clamp(out, -15.0, 15.0), MotorControlMode::Voltage);
    }else{
        motors[0]->set(out, MotorControlMode::Voltage);
    }
    updateTelemetry(out);
}

