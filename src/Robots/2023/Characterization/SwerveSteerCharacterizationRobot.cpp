#include "SwerveSteerCharacterizationRobot.h"

SwerveSteerCharacterizationRobot::SwerveSteerCharacterizationRobot() : EctoCharacterizationRobot("SwerveSteerCharacterization"){
    
}

void SwerveSteerCharacterizationRobot::robotInit(){
    steerMotor = motorManager.getMotor("bl");
    steerMotor->enableBrakingOnIdle(true);

    steerMotor->setMotorCurrentLimit(30);
    steerMotor->enableCurrentLimit(true);

    steerMotor->setAnalogPositionConversionFactor(2 * M_PI / 3.33333333); //Potentiometer v/rot
    steerMotor->setAnalogVelocityConversionFactor(2 * M_PI / 3.33333333);
    steerMotor->setAnalogSensorOffset(0);

    steerMotor->setFeedbackMode(MotorFeedbackMode::Potentiometer);
    steerMotor->setOpenLoopRampRate(0.00001);
    steerMotor->setClosedLoopRampRate(0.00001);

    steerMotor->invertMotor(false);
    steerMotor->invertSensor(true);

    steerMotor->enableLimitSwitches(false);


    steerMotor->prioritizeUpdateRate();
    steerMotor->burnFlash();

    inputManager.registerAxis(manualControl, "rightY");

}

void SwerveSteerCharacterizationRobot::teleopUpdate(){
    steerMotor->set(-manualControl.get(), MotorControlMode::Percent);
}

double SwerveSteerCharacterizationRobot::getPose() const {
    return steerMotor->getPosition();
}

double SwerveSteerCharacterizationRobot::getVel() const {
    return steerMotor->getVelocity();
}

void SwerveSteerCharacterizationRobot::robotUpdate(){
    frc::SmartDashboard::PutNumber("Pose_rad", getPose());
    frc::SmartDashboard::PutNumber("Pose_rot", getPose() / (2 * M_PI));
    frc::SmartDashboard::PutNumber("Vel_rad", getVel());
    frc::SmartDashboard::PutNumber("Vel_rot", getVel() / (2 * M_PI));
}

void SwerveSteerCharacterizationRobot::autoInit(){
    logger.InitLogging();
}

void SwerveSteerCharacterizationRobot::autoUpdate(){
    double pos = getPose() / (2 * M_PI);
    double vel = getVel() / (2 * M_PI);

    double voltage = logger.GetMotorVoltage().value();
    steerMotor->set(voltage, MotorControlMode::Voltage);

    logger.Log(voltage, pos, vel);
}

void SwerveSteerCharacterizationRobot::disabledInit(){
    steerMotor->set(0);
    logger.SendData();
}