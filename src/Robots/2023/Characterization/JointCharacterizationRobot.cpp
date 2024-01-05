//
// Created by cc on 6/02/23.
//

#include "JointCharacterizationRobot.h"

#include "Math/EctoMath.h"

JointCharacterizationRobot::JointCharacterizationRobot() : EctoCharacterizationRobot("JointCharacterizationRobot") {
    ;
}


void JointCharacterizationRobot::robotInit() {
    arm = motorManager.getMotor("arm");
    armFollower = motorManager.getMotor("armFollower");

    arm->setFeedbackMode(MotorFeedbackMode::Potentiometer);
    double conversionFactor = 2 * M_PI / 3.333333;
    arm->setAnalogVelocityConversionFactor(conversionFactor);
    arm->setAnalogPositionConversionFactor(conversionFactor);
    arm->setAnalogSensorOffset(analogOffset);
    arm->enableForwardSoftLimit(false);
    arm->enableReverseSoftLimit(false);
    arm->enableLimitSwitches(false);
    arm->invertMotor(false);
    arm->invertSensor(true);
    double currentLimit = 40;
    arm->enableBrakingOnIdle(true);
    arm->setMotorCurrentLimit(currentLimit);
    arm->enableCurrentLimit(true);
    arm->prioritizeUpdateRate();

    armFollower->followMotor(*arm, true);
    armFollower->enableForwardSoftLimit(false);
    armFollower->enableReverseSoftLimit(false);
    armFollower->enableBrakingOnIdle(true);
    armFollower->setMotorCurrentLimit(currentLimit);
    armFollower->enableCurrentLimit(true);
    armFollower->prioritizeUpdateRate();
    armFollower->enableLimitSwitches(false);


    inputManager.registerAxis(wristAxis, "rightY");
//    wrist->setSensorPosition(wrist->getPosition());
}

void JointCharacterizationRobot::teleopUpdate() {
    arm->set(-wristAxis.get(), MotorControlMode::Percent);
}

void JointCharacterizationRobot::robotUpdate() {
    updateTelemetry();
}

void JointCharacterizationRobot::autoInit() {
    logger.InitLogging();

}

void JointCharacterizationRobot::autoUpdate() {
    double pos = (arm->getPosition() / (2 * M_PI));
    double vel = (arm->getVelocity() / (2 * M_PI));

    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    arm->set(voltage, MotorControlMode::Voltage);

    logger.Log(voltage, pos, vel);
}

void JointCharacterizationRobot::disabledInit(){
    logger.SendData();
    arm->set(0);
}

void JointCharacterizationRobot::updateTelemetry() {
    table->GetEntry("Pos").SetDouble(arm->getPosition());
    table->GetEntry("rawVel").SetDouble(arm->getVelocity());
//    table->GetEntry("vel").SetDouble(velFilter.Calculate(arm->getVelocity()));

    auto analogPos = arm->getPosition();
    analogPos += analogOffset;
    analogPos = EctoMath::wrapAngle(analogPos);
    table->GetEntry("RawAnalog").SetDouble(analogPos);
}
