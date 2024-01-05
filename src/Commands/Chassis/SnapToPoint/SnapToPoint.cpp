//
// Created by cc on 20/02/23.
//

#include "SnapToPoint.h"
#include <units/angle.h>
#include <iostream>

SnapToPoint::SnapToPoint(const std::shared_ptr<EctoSwerve> &swerve, const SnapToPointConfig &config, ScorePiece scorePiece) :
        xController(config.pos_p, config.pos_i, config.pos_d, {config.maxVel, config.maxAccel}),
        yController(config.pos_p, config.pos_i, config.pos_d, {config.maxVel, config.maxAccel}),
        thetaController(config.theta_p, config.theta_i, config.theta_d, {config.maxAngularVel, config.maxAngularAccel})
{
    this->config = config;
    this->swerve = swerve;
    this->scorePiece = scorePiece;

    AddRequirements(swerve.get());
}

std::optional<frc::Pose2d> SnapToPoint::getClosestPose(const std::vector<frc::Translation2d> &points,
                                                       const frc::Pose2d currentPose) {

    std::vector<double> distances;
    for (auto &point : points){
        distances.emplace_back(
                currentPose.Translation().Distance(point).value()
        );
    }

    auto it = std::min_element(distances.begin(), distances.end());
    auto selectedPose = points.at(std::distance(distances.begin(), it));

    double yaw = std::abs(currentPose.Rotation().Radians().value());
//    std::cout << yaw << std::endl;
    frc::Rotation2d yawOut;
    if (yaw < M_PI / 2){
        yawOut = 0_deg;
    } else {
        yawOut = units::degree_t(180);
    }
//    std::cout << yawOut.Radians().value();

    auto poseOut = frc::Pose2d(selectedPose, yawOut);
    return poseOut;
}

void SnapToPoint::Initialize() {
    auto pose = swerve->getPose();
    xController.Reset(pose.X());
    yController.Reset(pose.Y());
    thetaController.Reset(pose.Rotation().Radians());

    thetaController.EnableContinuousInput(-3.141592653589_rad, 3.141592653589_rad);

    points = scorePiece == ScorePiece::CONE ? config.conePoints : config.cubePoints;

    auto closestPose = getClosestPose(points, swerve->getPose());
    if (closestPose.has_value()){
        targetPose = closestPose.value();
    }else {
        targetPose = pose;
    }
}

void SnapToPoint::Execute() {
    auto pose = swerve->getPose();
    out.vx = units::meters_per_second_t(xController.Calculate(pose.X(), targetPose.X()));
    out.vy = units::meters_per_second_t(yController.Calculate(pose.Y(), targetPose.Y()));

    distToTarget = pose.Translation().Distance(targetPose.Translation());
//    std::cout << targetPose.Rotation().Degrees().value() << std::endl;
    if(distToTarget < 1.2_m){
        out.omega = units::radians_per_second_t (thetaController.Calculate(pose.Rotation().Radians(), targetPose.Rotation().Radians()));
    }

    out = frc::ChassisSpeeds::FromFieldRelativeSpeeds(out.vx, out.vy, out.omega, frc::Rotation2d(units::radian_t(swerve->getYaw())));

    swerve->setVoltage(out);

    table->GetEntry("out").SetString(fmt::format("({}, {}, {})", out.vx, out.vy, out.omega));

}

void SnapToPoint::End(bool interrupted) {
    swerve->setVoltage({});
}

bool SnapToPoint::IsFinished() {
    auto pose = swerve->getPose();
    auto dist = pose.Translation().Distance(targetPose.Translation());
    auto rot = std::abs(pose.Rotation().Radians().value() - targetPose.Rotation().Radians().value());
    return dist < 0.01_m && rot < 0.01;
}

void SnapToPoint::updateTelemetry() {
    table->GetEntry("Out/vx").SetDouble(out.vx.value());
    table->GetEntry("Out/vy").SetDouble(out.vy.value());
    table->GetEntry("Out/omega").SetDouble(out.omega.value());
    table->GetEntry("distance").SetDouble(distToTarget.value());
    table->GetEntry("targetPose/x").SetDouble(targetPose.X().value());
    table->GetEntry("targetPose/y").SetDouble(targetPose.Y().value());
    table->GetEntry("targetPose/theta").SetDouble(targetPose.Rotation().Radians().value());

}