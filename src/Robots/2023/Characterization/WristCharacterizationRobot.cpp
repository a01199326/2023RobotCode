//
// Created by terromn on 2/02/23.
//

#include "WristCharacterizationRobot.h"
#include "Math/EctoMath.h"
#include <iostream>

WristCharacterizationRobot::WristCharacterizationRobot() : EctoCharacterizationRobot("wristCharacterization") {
    ;
}


void WristCharacterizationRobot::robotInit() {
    wrist = motorManager.getMotor("wrist");

    wrist->setFeedbackMode(MotorFeedbackMode::AbsoluteEncoder);
    double conversionFactor = (2 * M_PI);
    wrist->setAbsolutePositionConversionFactor(conversionFactor);
    wrist->setAbsoluteVelocityConversionFactor(conversionFactor);
    wrist->setAbsoluteSensorOffset(analogOffset);
    wrist->enableForwardSoftLimit(false);
    wrist->enableReverseSoftLimit(false);
    wrist->invertMotor(true);
    wrist->invertSensor(true);
    wrist->setOpenLoopRampRate(0.01);
    double currentLimit = 40;
    wrist->enableBrakingOnIdle(true);
    wrist->setMotorCurrentLimit(currentLimit);
    wrist->enableCurrentLimit(true);
    wrist->prioritizeAbsUpdateRate();

    inputManager.registerAxis(wristAxis, "rightY");
//    wrist->setSensorPosition(wrist->getPosition());
}

void WristCharacterizationRobot::teleopUpdate() {
    wrist->set(-wristAxis.get(), MotorControlMode::Percent);
}

void WristCharacterizationRobot::robotUpdate() {
    updateTelemetry();
}

void WristCharacterizationRobot::autoInit() {
    logger.InitLogging();

}

void WristCharacterizationRobot::autoUpdate() {
    double pos = ((getAngle()) / (2 * M_PI));
    double vel = ((getVel()) / (2 * M_PI));

    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    wrist->set(voltage, MotorControlMode::Voltage);

    logger.Log(voltage, pos, vel);
}

void WristCharacterizationRobot::disabledInit(){
    std::cout << "sending the data" << std::endl;
    logger.SendData();
    std::cout << "data sent" << std::endl;
    wrist->set(0);
}

double WristCharacterizationRobot::getAngle() const{
    return EctoMath::wrapAngle(wrist->getPosition() / 1.3);
//    return EctoMath::wrapAngle(wrist->getPosition());
}

double WristCharacterizationRobot::getVel() const{
    return EctoMath::wrapAngle(wrist->getVelocity() / 1.3);
//    return EctoMath::wrapAngle(wrist->getVelocity());
}

void WristCharacterizationRobot::updateTelemetry() {
    table->GetEntry("Pos").SetDouble(getAngle());
    table->GetEntry("rawVel").SetDouble(getVel());
    table->GetEntry("vel").SetDouble(velFilter.Calculate(getVel()));

    auto analogPos = getAngle() * 1.3;
    analogPos += analogOffset;
    analogPos = EctoMath::wrapAngle(analogPos);
    table->GetEntry("RawAnalog").SetDouble(analogPos);
    table->GetEntry("analogOffset").SetDouble(analogOffset);
}
