//
// Created by cc on 3/02/23.
//

#include "LQRWrist.h"
#include "Math/EctoMath.h"

LQRWrist::LQRWrist(const LQRWristConfig &config) : WPISubsystem("LQRWrist"),
                                                   plant(frc::LinearSystemId::IdentifyPositionSystem<units::radians>(config.kV / 1_rad_per_s, config.kA / 1_rad_per_s_sq)),
                                                   observer(plant,
                                                                 {1.2155, 0.652},
                                                                 {0.001},
                                                                 20_ms),
                                                   controller(plant,
                                                                   {0.355, 1.73575},
                                                                   {5.675},
                                                                   20_ms),
                                                   loop(plant, controller, observer, 10_V, 20_ms),
                                                   ff(plant, 20_ms){
    this->config = config;
    table = ntInstance.GetTable("LQRWrist");

    if (motors.empty()) {
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
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(config.forwardSoftLimit.value() / config.gearReduction);
            motor->setForwardSoftLimit(config.reverseSoftLimit.value() / config.gearReduction);
            motor->enableForwardSoftLimit(false);
            motor->enableReverseSoftLimit(false);
            motor->enableLimitSwitches(false);
            motor->invertMotor(true);
            motor->setFeedbackMode(MotorFeedbackMode::Potentiometer);
            motor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //potentiometer v/rot
            motor->setAnalogVelocityConversionFactor(2 * M_PI / 3.33333333);
            motor->invertSensor(true);
            motor->setAnalogSensorOffset(config.analogOffset.value());
            analogOffset = config.analogOffset;


        } else {
            motor->enableBrakingOnIdle(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->setReverseSoftLimit(config.forwardSoftLimit.value() / config.gearReduction);
            motor->setForwardSoftLimit(config.reverseSoftLimit.value() / config.gearReduction);
            motor->enableForwardSoftLimit(true);
            motor->enableReverseSoftLimit(true);
            motor->enableLimitSwitches(false);
            motor->invertMotor(true);

        }

    }
//    controller.LatencyCompensate(plant, 20_ms, 25_ms);
//    loop = frc::LinearSystemLoop<2,1,1>(plant, controller, obswerver, config.maxControlVoltage, 20_ms);
    resetController();
}

void LQRWrist::robotInit(){
    ;
}

bool LQRWrist::getForwardLimitSwitch() {
    return motors[0]->getForwardLimitSwitch();
}

bool LQRWrist::getReverseLimitSwitch() {
    return motors[0]->getReverseLimitSwitch();
}

units::radian_t LQRWrist::getAngle() {
    return units::radian_t(motors[0]->getPosition());
}

units::radians_per_second_t LQRWrist::getVel() {
    return units::radians_per_second_t(motors[0]->getVelocity());
}

void LQRWrist::set(units::radian_t radians) {
    double wrapOut = EctoMath::wrapAngle(radians.value());
    double clamped = std::clamp(wrapOut, config.reverseSoftLimit.value(), config.forwardSoftLimit.value());
    goal = units::radian_t(clamped);
}

void LQRWrist::resetController() {
    controller.Reset();
    observer.Reset();
    ff.Reset();
    loop.Reset(Eigen::Vector<double, 2>{getAngle(), 0});

}

void LQRWrist::useLQRControl(bool use) {
    this->useLQR = use;
}

void LQRWrist::useSoftLimits(bool useSoftLimits) {
    motors[0]->enableForwardSoftLimit(useSoftLimits);
    motors[0]->enableReverseSoftLimit(useSoftLimits);
}

void LQRWrist::setAnalogOffset(units::radian_t angle) {
    motors[0]->setAnalogSensorOffset(config.analogOffset.value());
    analogOffset = config.analogOffset;
}

void LQRWrist::setCurrentLimit(units::ampere_t amps) {
    for (auto &motor : motors){
        motor->setMotorCurrentLimit(amps.value());
        motor->enableCurrentLimit(true);
    }
}

bool LQRWrist::atGoal() {
    return std::abs(error(0,0)) < 0.1 and std::abs(error(1,0)) < 0.1;
}

void LQRWrist::updateTelemetry() {
    table->GetEntry("PIDOut").SetDouble(out.value());
    table->GetEntry("angle").SetDouble(getAngle().value());
    table->GetEntry("Vel").SetDouble(getVel().value());
    table->GetEntry("manualVoltage").SetDouble(manualVoltage);
    table->GetEntry("forwardLimitSwitch").SetBoolean(motors[0]->getForwardLimitSwitch());
    table->GetEntry("reverseLimitSwitch").SetBoolean(motors[0]->getReverseLimitSwitch());
    table->GetEntry("current").SetDouble(motors[0]->getCurrent());
}

units::volt_t LQRWrist::calculate(units::radian_t posSet, units::radians_per_second_t velSet){
    loop.SetNextR(Eigen::Vector<double, 2>{
        posSet.value(),
        velSet.value()
    });

    double angle = getAngle().value();
    error = loop.Controller().R() - Eigen::Vector<double, 2>(angle, 0.0);
    u = loop.Controller().K() * error;
    u += ff.Calculate(loop.NextR());
    u = loop.ClampInput(u);
    controller.Calculate(loop.Xhat(), loop.NextR());
    observer.Correct(u, Eigen::Vector<double, 1>{angle});
    observer.Predict(u, 20_ms);

    lastAngle = angle * 1_rad;
    return u(0,0) * 1_V;
}

void LQRWrist::robotUpdate() {
    out = calculate(goal, 0_rad_per_s);
    motors[0]->set(out.value(), MotorControlMode::Voltage);
    updateTelemetry();
}