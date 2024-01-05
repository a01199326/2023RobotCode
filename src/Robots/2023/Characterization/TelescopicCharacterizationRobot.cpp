//
// Created by terromn on 30/01/23.
//

#include "TelescopicCharacterizationRobot.h"

TelescopicCharacterizationRobot::TelescopicCharacterizationRobot() : EctoCharacterizationRobot(
        "TelescopicCharacterization") {

}

void TelescopicCharacterizationRobot::robotInit() {
    telescopic = motorManager.getMotor("telescopic");
    telescopic1 = motorManager.getMotor("telescopicFollower");
    telescopic2 = motorManager.getMotor("telescopicSecondFollower");

    telescopic->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    double currentLimit = 40;
    telescopic->enableLimitSwitches(false);
    telescopic->enableForwardSoftLimit(false);
    telescopic->enableForwardSoftLimit(false);
    telescopic->enableBrakingOnIdle(true);
    telescopic->setMotorCurrentLimit(currentLimit);
    telescopic->enableCurrentLimit(true);
    telescopic->invertMotor(false);
    telescopic->prioritizeUpdateRate();
    telescopic->burnFlash();


    telescopic1->followMotor(*telescopic, true);
    telescopic1->prioritizeUpdateRate();
    telescopic1->enableBrakingOnIdle(true);
    telescopic1->enableLimitSwitches(false);
    telescopic1->enableForwardSoftLimit(false);
    telescopic1->enableForwardSoftLimit(false);
    telescopic1->setMotorCurrentLimit(currentLimit);
    telescopic1->enableCurrentLimit(true);
    telescopic1->burnFlash();


    telescopic2->followMotor(*telescopic, true);
    telescopic2->prioritizeUpdateRate();
    telescopic2->enableLimitSwitches(false);
    telescopic2->enableForwardSoftLimit(false);
    telescopic2->enableForwardSoftLimit(false);
    telescopic2->enableBrakingOnIdle(true);
    telescopic2->setMotorCurrentLimit(currentLimit);
    telescopic2->enableCurrentLimit(true);
    telescopic2->burnFlash();

    inputManager.registerAxis(telescopicAxis, "rightY");

}

void TelescopicCharacterizationRobot::teleopUpdate() {
    telescopic->set(-telescopicAxis.get(), MotorControlMode::Percent);
}

void TelescopicCharacterizationRobot::robotUpdate() {
    frc::SmartDashboard::PutNumber("Telescopic Position", telescopic->getPosition());
}

void TelescopicCharacterizationRobot::autoInit() {
    logger.InitLogging();
}

void TelescopicCharacterizationRobot::autoUpdate() {
    double pos = (telescopic->getPosition() / gearRatioTelescopic / (2 * M_PI) * 2);
    double vel = (telescopic->getVelocity() / gearRatioTelescopic / (2 * M_PI) * 2);

    frc::SmartDashboard::PutNumber("pos", pos);
    frc::SmartDashboard::PutNumber("vel", vel);

    double voltage = logger.GetMotorVoltage().value();
    telescopic->set(voltage, MotorControlMode::Voltage);

    logger.Log(voltage, pos, vel);
}

void TelescopicCharacterizationRobot::disabledInit(){
    logger.SendData();
    telescopic->set(0);
}

