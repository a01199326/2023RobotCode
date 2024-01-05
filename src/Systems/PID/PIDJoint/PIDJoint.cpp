//
// Created by cc on 3/11/22.
//

#include "PIDJoint.h"


PIDJoint::PIDJoint(const PIDJointConfig &config) : WPISubsystem("PIDJoint"){
    this->config = config;
    this->motors = config.motors;

    frc::ProfiledPIDController<units::radian>::Constraints constraints(
            config.maxVel,
            config.maxAccel
            );
    pidController = std::make_unique<frc::ProfiledPIDController<units::radians>>(
            config.pidConfig.p, config.pidConfig.i, config.pidConfig.d, constraints
            );

    pidController->SetTolerance(config.pidDistTol, config.pidVelTol);

    if(motors.empty()){
        log->error("No motor given to PIDJoint!!!");
        throw std::runtime_error("Motor vector is empty!");
    }

    if(config.isInverted.size() != motors.size()){
        log->error("Motor inversion not configured!");
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
            motor->invertMotor(config.isInverted[i]);
            motor->setFeedbackMode(MotorFeedbackMode::AbsoluteEncoder);
//            motor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //potentiometer v/rot
//            motor->setAnalogVelocityConversionFactor(2 * M_PI / 3.33333333);
            motor->setAbsolutePositionConversionFactor(2 * M_PI);
            motor->setAbsoluteVelocityConversionFactor(2 * M_PI);
            motor->invertSensor(config.invertSensor);
            motor->setAbsoluteSensorOffset(config.analogOffset);
            analogOffset = config.analogOffset;


        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(config.forwardSoftLimit.value() / config.gearReduction);
            motor->setForwardSoftLimit(config.forwardSoftLimit.value() / config.gearReduction);
            motor->enableForwardSoftLimit(false);
            motor->enableReverseSoftLimit(false);
            motor->enableLimitSwitches(false);

        }

    }

    pidController->Reset(getAngle());
    setFF(config.ff);

}

void PIDJoint::robotInit() { ; }

bool PIDJoint::getForwardLimitSwitch() {
    return motors[0]->getForwardLimitSwitch();
}

bool PIDJoint::getReverseLimitSwitch() {
    return motors[0]->getReverseLimitSwitch();
}

units::radian_t PIDJoint::getAngle() {
    double pose = motors[0]->getPosition();
//    pose = analogFilter.Calculate(pose);
    return units::radian_t(pose);
}

units::radians_per_second_t PIDJoint::getVel() {
    return units::radians_per_second_t(motors[0]->getVelocity());
}

void PIDJoint::set(units::radian_t radians){
    double wrapOut = EctoMath::wrapAngle(radians.value());
    double clamped = std::clamp(wrapOut, config.reverseSoftLimit.value(), config.forwardSoftLimit.value());
    goal = units::radian_t(clamped);
    pidController->SetGoal(goal);
}

void PIDJoint::resetController() {
    pidController->Reset(getAngle());
}

void PIDJoint::usePIDControl(bool usePID){
    this->usePID = usePID;
}

void PIDJoint::useSoftLimits(bool useSoftLimits) {
    for (auto &motor : motors){
        motor->enableForwardSoftLimit(useSoftLimits);
        motor->enableReverseSoftLimit(useSoftLimits);
    }}

void PIDJoint::setAnalogOffset(units::radian_t angle) {
    motors[0]->setAnalogSensorOffset(angle.value());
    analogOffset = angle.value();
}

bool PIDJoint::atGoal() {
    return pidController->AtGoal();
}

bool PIDJoint::inTol(units::radian_t tol) {
    return std::abs(getAngle().value() - goal.value()) < tol.value();
}

bool PIDJoint::inTolOf(units::radian_t tol, units::radian_t of) {
    return std::abs(getAngle().value() - of.value()) < tol.value();
}

units::radians_per_second_t PIDJoint::getMaxVel() {
    return config.maxVel;
};


units::radians_per_second_squared_t PIDJoint::getMaxAccel() {
    return config.maxAccel;
}

void PIDJoint::setConstraints(units::radians_per_second_t maxSpeed, units::radians_per_second_squared_t maxAccel) {
    frc::ProfiledPIDController<units::radian>::Constraints constraint(
            maxSpeed,
            maxAccel
    );
    pidController->SetConstraints(constraint);
    config.maxVel = maxSpeed;
    config.maxAccel = maxAccel;
}

void PIDJoint::setPID(PIDConfig &config) {
    pidController->SetPID(config.p, config.i, config.d);
}

void PIDJoint::setFF(frc::SimpleMotorFeedforward<units::radians> ff) {
    this->ff = &ff;
}

units::volt_t PIDJoint::calculateFF(units::radian_t angle, units::radians_per_second_t vel) {
    return ff->Calculate(vel);
}

void PIDJoint::updateTelemetry(double out) {
    table->GetEntry("PIDOut").SetDouble(out);
    table->GetEntry("angle").SetDouble(getAngle().value());
    table->GetEntry("Vel").SetDouble(getVel().value());
    table->GetEntry("usePID").SetBoolean(usePID);
    table->GetEntry("manualVoltage").SetDouble(manualVoltage);
    table->GetEntry("forwardLimitSwitch").SetBoolean(motors[0]->getForwardLimitSwitch());
    table->GetEntry("reverseLimitSwitch").SetBoolean(motors[0]->getReverseLimitSwitch());
    table->GetEntry("current").SetDouble(motors[0]->getCurrent());
    auto analogPos = getAngle().value();
    analogPos += analogOffset;
    analogPos = EctoMath::wrapAngle(analogPos);
    table->GetEntry("RawAnalog").SetDouble(analogPos);
    table->GetEntry("SetPoint").SetDouble(goal.value());
}

void PIDJoint::robotUpdate() {
    double out{0};
    if (!inTol(0.006_rad)){
        out = pidController->Calculate(getAngle());
        out += ff->Calculate(pidController->GetSetpoint().velocity).value();
        motors[0]->set(out, MotorControlMode::Voltage);
    }else{
        motors[0]->set(out, MotorControlMode::Voltage);
    }
    updateTelemetry(out);
}