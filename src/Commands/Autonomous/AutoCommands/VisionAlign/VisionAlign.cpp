//
// Created by cc on 04/01/22.
//

#include "VisionAlign.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

VisionAlign::VisionAlign(const std::shared_ptr<EctoSwerve> &swerve,
                         const std::shared_ptr<VisionManager> &visionManager,
                         const std::shared_ptr<Manipulator> &manipulator,
                         const VisionAlignConfig &config,
                         ScorePiece scorePiece) :
     yawController(config.yawPIDConfig.p, config.yawPIDConfig.i, config.yawPIDConfig.d, {config.maxAngularVel, config.maxAngularAccel}),
     yController(config.yPIDConfig.p, config.yPIDConfig.i, config.yPIDConfig.d, {config.maxVel, config.maxAccel}),
     xController(config.yPIDConfig.p, config.yPIDConfig.i, config.yPIDConfig.d, {1.0_mps, 1.0_mps_sq}){



	this->visionManager = visionManager;
    this->manipulator = manipulator;
	this->swerve = swerve;
    this->config = config;

    AddRequirements(swerve.get());

    camera = std::make_shared<LimeLight>("center", ntInstance);

}

void VisionAlign::Initialize() {
    auto pi = units::radian_t(M_PI);
    auto pose = swerve->getPose();
    yawController.Reset(pose.Rotation().Radians());
    yawController.EnableContinuousInput(-pi, pi);
    yawController.SetTolerance(0.012_rad);
    auto measuredX = visionManager->getXOffsetToTarget(0);
    auto measuredY = visionManager->getYOffsetToTarget(0);

    if (!measuredY.has_value() && !measuredX.has_value() && measuredY == std::nullopt){
        swerve->setVoltage(out);
        return;
    }

    yController.Reset(measuredY.value());
    yController.SetTolerance(0.01_m);

    xController.Reset(measuredX.value());
    xController.SetTolerance(0.01_m);

    bool isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    yawGoal = isRed ? 0_deg : 180_deg;


}

//Switch to using pose estimated vision alignment
void VisionAlign::Execute() {
    auto pose = swerve->getPose();
    auto yaw = units::radian_t(swerve->getYaw());
    auto measuredX = visionManager->getXOffsetToTarget(0);
    auto measuredY = visionManager->getYOffsetToTarget(0);

    out = {};

    if (!measuredY.has_value() && !measuredX.has_value() && measuredY == std::nullopt){
        swerve->setVoltage(out);
        return;
    }
    auto stateY = measuredY.value();
    auto stateX = measuredX.value();

    double goalX = 0.49;
    if (camera->getPipeLine() == 1) goalX = 0.43;
//    out.vx = units::meters_per_second_t(xController.Calculate(stateX, units::meter_t(goalX)));

    double y = stateY.value();
    double error = magicToReal(y) - manipulator->getPiecePose();
    table->GetEntry("yError").SetDouble(error);
    out.vy = units::meters_per_second_t(yController.Calculate(units::meter_t(error), 0_m));

    if (xController.AtGoal()) out.vx = 0_mps;
    if (std::abs(error) < 0.05) out.vy = 0_mps;

    out = ChassisSpeeds::FromFieldRelativeSpeeds(out, frc::Rotation2d(yaw));
    out.omega = units::radians_per_second_t(yawController.Calculate(pose.Rotation().Radians(), yawGoal));

    swerve->setVoltage(out);

    updateTelemetry();

}

void VisionAlign::End(bool interrupted) {
    swerve->setVoltage({});
}

bool VisionAlign::isReady() const {
    auto pose = swerve->getPose();
    bool dist = yController.AtGoal() && swerve->getVelocity().Y() < 0.05_mps;
    double rot = std::abs(pose.Rotation().Radians().value() - yawGoal.value());
    return dist && rot < 0.01;
}

bool VisionAlign::IsFinished() {
    return isReady();
}

void VisionAlign::updateTelemetry() {
    table->GetEntry("out/x").SetDouble(out.vx.value());
    table->GetEntry("out/y").SetDouble(out.vy.value());
    table->GetEntry("out/omega").SetDouble(out.omega.value());
    table->GetEntry("goal/x").SetDouble(xGoal.value());
    table->GetEntry("goal/y").SetDouble(yGoal.value());
    table->GetEntry("goal/theta").SetDouble(yawGoal.value());
}

double VisionAlign::magicToReal(double magic) const {
    return (2.0749 * (std::pow(magic, 2)) + (1.6712 * magic) - 0.0453);
}