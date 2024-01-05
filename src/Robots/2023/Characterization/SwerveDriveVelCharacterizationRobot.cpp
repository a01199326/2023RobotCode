//
// Created by cc on 27/04/23.
//

#include "SwerveDriveVelCharacterizationRobot.h"


SwerveDriveVelCharacterizationRobot::SwerveDriveVelCharacterizationRobot() : EctoCharacterizationRobot("SwerveDriveCharacterization"){
    ;
}

void SwerveDriveVelCharacterizationRobot::robotInit() {
    driveMotor = motorManager.getMotor("blDrive");
    driveMotor->prioritizeUpdateRate();

    inputManager.registerAxis(manualAxis, "rightY");


    driveMotor->enableBrakingOnIdle(true);

    driveMotor->setMotorCurrentLimit(40);
    driveMotor->enableCurrentLimit(true);

    driveMotor->invertMotor(false);

    driveMotor->setClosedLoopRampRate(0.05);
    driveMotor->setOpenLoopRampRate(0.05);

    driveMotor->enableLimitSwitches(false);
    driveMotor->enableReverseSoftLimit(false);
    driveMotor->enableForwardSoftLimit(false);

    driveMotor->burnFlash();

}

void SwerveDriveVelCharacterizationRobot::robotUpdate() {
    frc::SmartDashboard::PutNumber("motorPose", getMotorPose());
    frc::SmartDashboard::PutNumber("motorVel", getMotorVel());
}

void SwerveDriveVelCharacterizationRobot::teleopUpdate() {
    driveMotor->set(-manualAxis.get(), MotorControlMode::Percent);
}


double SwerveDriveVelCharacterizationRobot::getMotorPose() const {
    return (driveMotor->getPosition() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
}

double SwerveDriveVelCharacterizationRobot::getMotorVel() const {
    return (driveMotor->getVelocity() / (2.0 * M_PI)) / gearRatio * wheelCircumference;
}

void SwerveDriveVelCharacterizationRobot::autoInit() {
    logger.InitLogging();
}

void SwerveDriveVelCharacterizationRobot::autoUpdate() {
    double voltage = logger.GetMotorVoltage().value();
    driveMotor->set(voltage, MotorControlMode::Voltage);

    double pos = getMotorPose();
    double vel = getMotorVel();
    logger.Log(voltage, pos, vel);

}

void SwerveDriveVelCharacterizationRobot::disabledInit() {
    driveMotor->set(0);
    logger.SendData();
}