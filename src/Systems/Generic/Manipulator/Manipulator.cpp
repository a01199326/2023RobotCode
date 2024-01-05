//
// Created by cc on 22/01/23.
//

#include "Manipulator.h"


Manipulator::Manipulator(const ManipulatorConfig &config) : WPISubsystem("manipulator"){
    this->config = config;
    motors = config.motors;
//    pistons = config.pistons;
    this->currentPieceThreshold = config.cubeCurrentThreshold;

    if (motors.empty()) {
        log->error("No motors given to Manipulator!!!");
        throw std::runtime_error("Motor vector is empty!");
    }


    if (config.isInverted.size() != motors.size())
        throw std::runtime_error("Motor inversion not configured!");

    for (size_t i = 0; i < motors.size(); i++) {
        const auto motor = motors[i];
        if (i == 0) {
            motor->enableBrakingOnIdle(config.breakingOnIdle);
            motor->enableCurrentLimit(true);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->enableForwardSoftLimit(false);
            motor->enableReverseSoftLimit(false);
            motor->enableLimitSwitches(false);
            motor->invertMotor(config.isInverted[i]);
            motor->setFeedbackMode(MotorFeedbackMode::AbsoluteEncoder);
            motor->setAbsolutePositionConversionFactor(1);
            motor->setAbsoluteVelocityConversionFactor(1);
            motor->setAbsoluteSensorOffset(0);
            motor->invertSensor(false);
            motor->burnFlash();


        } else {
            motor->followMotor(*motors[0], config.isInverted[i]);
            motor->enableBrakingOnIdle(config.breakingOnIdle);
            motor->setMotorCurrentLimit(config.currentLimit.value());
            motor->enableCurrentLimit(true);
            motor->prioritizeUpdateRate();
            motor->setControlMode(MotorControlMode::Voltage);
            motor->setOpenLoopRampRate(0.01);
            motor->enableForwardSoftLimit(false);
            motor->enableReverseSoftLimit(false);
            motor->enableLimitSwitches(false);
            motor->burnFlash();

        }
    }
}

void Manipulator::set(double set, MotorControlMode controlMode) {
    setPoint = set;
    motors[0]->set(set, controlMode);
}

void Manipulator::idle(){
    motors[0]->set(config.idlePercent, MotorControlMode::Percent);
}

void Manipulator::setPieceThreshold(ScorePiece piece) {
    currentPieceThreshold = piece == ScorePiece::CONE ? config.coneCurrentThreshold : config.cubeCurrentThreshold;
}

bool Manipulator::hasPiece() const {
    return current > currentPieceThreshold.value();
}

double Manipulator::getDistance() {
    double out = distanceFilter.Calculate(config.coneRadius + (motors[0]->getPosition() * 0.5));
    if (out > 0.33){
        return (config.manipulatorWidth / 2);
    }
    if(out > 0.5){
        log->error("uWu my sensor is bowken :(");
        return 0.25;
    }
    return out;
}

double Manipulator::getPiecePose() {
    return (config.manipulatorWidth / 2) - getDistance();
}

void Manipulator::robotUpdate() {
    current = currentFilter.Calculate(motors[0]->getCurrent());
    table->GetEntry("current").SetDouble(current);
    table->GetEntry("hasPiece").SetBoolean(hasPiece());
    table->GetEntry("set").SetDouble(setPoint);
    table->GetEntry("sensorDistance").SetDouble(getDistance());
    table->GetEntry("sensorDelta").SetDouble(motors[0]->getVelocity());
    table->GetEntry("PiecePose").SetDouble(getPiecePose());
    table->GetEntry("CurrentThreshold").SetDouble(currentPieceThreshold.value());
}

//void Manipulator::setMotor(double set, MotorControlMode controlMode) {
//    motors[0]->set(set, controlMode);
//    table->GetEntry("MotorSet").SetDouble(set);
//    table->GetEntry("MotorControlMode").SetString(toString(controlMode));
//}
//
//void Manipulator::setPistons(bool set) {
//    for (auto &piston : pistons){
//        PCMManager::set(piston, set);
//    }
//    table->GetEntry("PistonState").SetBoolean(set);
//}
//
//void Manipulator::set(double setPct, bool closed){
//    setMotor(setPct, MotorControlMode::Percent);
//    setPistons(closed);
//}