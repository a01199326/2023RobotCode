//
// Created by cc on 29/04/23.
//

#include "SwerveSteerTest.h"


SwerveSteerTest::SwerveSteerTest() : EctoRobot("SwerveSteerTest"){

    frc::SmartDashboard::PutNumber("P Gain", 0);
    frc::SmartDashboard::PutNumber("I Gain", 0);
    frc::SmartDashboard::PutNumber("D Gain", 0);
    frc::SmartDashboard::PutNumber("I Zone", 0);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);

}

void SwerveSteerTest::robotInit(){
    motor = motorManager.getMotor("frWheel");

    motor->enableBrakingOnIdle(true);
    motor->setMotorCurrentLimit(40);
    motor->setOpenLoopRampRate(0.0001);
    motor->setClosedLoopRampRate(0.0001);
    motor->invertMotor(false);
    motor->setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    motor->enableForwardSoftLimit(false);
    motor->enableReverseSoftLimit(false);
    motor->enableLimitSwitches(false);

    motor->setPIDConfig({kp, ki, kd});

    motor->burnFlash();


    frc::SmartDashboard::PutNumber("PIDIn", 0);


    frc::SmartDashboard::PutNumber("p", 0);
    frc::SmartDashboard::PutNumber("i", 0);
    frc::SmartDashboard::PutNumber("d", 0);
    frc::SmartDashboard::PutNumber("f", 0);


}

void SwerveSteerTest::teleopUpdate(){



    double in = frc::SmartDashboard::GetNumber("PIDIn", 0);
    in *= gearReduction;
    in *= 2 * M_PI;
    motor->set(in, MotorControlMode::Velocity);


}

void SwerveSteerTest::robotUpdate(){
    double p = frc::SmartDashboard::GetNumber("p", 0);
    double i = frc::SmartDashboard::GetNumber("i", 0);
    double d = frc::SmartDashboard::GetNumber("d", 0);
    double f = frc::SmartDashboard::GetNumber("f", 0);

    if (p != kp || i != ki || d != kd || f != kf){
        PIDConfig config{p, i, d};
        config.f = f;
        motor->setPIDConfig(config);
        kp = p;
        ki = i;
        kd = d;
        kf = f;
    }

    frc::SmartDashboard::PutNumber("pose", motor->getPosition());
    frc::SmartDashboard::PutNumber("vel", (motor->getVelocity() / (2.0 * M_PI) / gearReduction * wheelCircumference));


}

void SwerveSteerTest::disabledInit(){

}