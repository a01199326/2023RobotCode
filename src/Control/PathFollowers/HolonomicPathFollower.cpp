//
// Created by abiel on 11/3/21.
//

#include "HolonomicPathFollower.h"
#include <frc/Timer.h>
#include <units/angular_acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>

HolonomicPathFollower::HolonomicPathFollower(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<botbusters::Trajectory> &traj,
                                             const HolonomicPathFollowerConfig &config) {
    this->config = config;
    ff = config.ff;

    this->traj = traj;
    this->swerve = swerve;

    xController = std::make_unique<frc2::PIDController>(
            config.posConfig.p, config.posConfig.i, config.posConfig.d
    );

    yController = std::make_unique<frc2::PIDController>(
            config.posConfig.p, config.posConfig.i, config.posConfig.d
    );

    auto thetaControllerConstraints = frc::TrapezoidProfile<units::radians>::Constraints(
            units::radians_per_second_t(config.maxAngularVelocity),
            units::radians_per_second_squared_t(config.maxAngularAcceleration)
    );

    thetaController = std::make_unique<frc::ProfiledPIDController<units::radians>>(
            config.thetaConfig.p, config.thetaConfig.i, config.thetaConfig.d, thetaControllerConstraints
    );

    thetaController->EnableContinuousInput(units::radian_t(-M_PI), units::radian_t(M_PI));
}

void HolonomicPathFollower::Initialize() {
    table = nt::NetworkTableInstance::GetDefault().GetTable("HolonomicPathFollower");

    startTime = frc::Timer::GetFPGATimestamp().value();
    xController->Reset();
    yController->Reset();
    thetaController->Reset(swerve->getPose().Rotation().Radians());
}

void HolonomicPathFollower::Execute() {
    double time = getTime();
    TrajectoryState state = traj->sample(units::second_t(time));
    double angle = state.pose.Rotation().Radians().value();
    double vel = state.vel.Norm().value();
    double accel = state.accel.Norm().value();

    table->GetEntry("Velocity").SetDouble(vel);
    table->GetEntry("Accel").SetDouble(accel);
    table->GetEntry("Pose/x").SetDouble(state.pose.X().value());
    table->GetEntry("Pose/y").SetDouble(state.pose.Y().value());
    table->GetEntry("Pose/theta").SetDouble(state.pose.Rotation().Radians().value());

    if (time >= traj->getTotalTime().value()) {
        vel = 0;
        accel = 0;
    }

    //Calculate ff vector (units are in volts)
    auto ffVec = ff.calculateFF(
            {state.vel.X().value(), state.vel.Y().value()},
            {state.accel.X().value(), state.accel.Y().value()}
    );

    frc::Pose2d swervePose = swerve->getPose();
//    auto rotSet = config.rotationFunction(swervePose.X().value(), swervePose.Y().value()).Radians();
    units::radian_t rotSet = state.pose.Rotation().Radians();
    double xOut = xController->Calculate(swervePose.X().value(), state.pose.X().value());
    double yOut = yController->Calculate(swervePose.Y().value(), state.pose.Y().value());
    double rotOut = thetaController->Calculate(swervePose.Rotation().Radians(), rotSet);
    double distanceToTarget = std::hypot(swervePose.X().value() - state.pose.X().value(), swervePose.Y().value() - state.pose.Y().value());
    table->GetEntry("DistanceToSetpoint").SetDouble(distanceToTarget);
    frc::ChassisSpeeds targetVel;

    //Add FF to target voltage
    targetVel.vx = units::meters_per_second_t(xOut + ffVec.x);
    targetVel.vy = units::meters_per_second_t(yOut + ffVec.y);
    targetVel.omega = units::radians_per_second_t(rotOut);
    targetVel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(targetVel.vx, targetVel.vy, targetVel.omega,
                                                            swervePose.Rotation());

    //targetVel.vx += units::meters_per_second_t(ffVec.x);
    //targetVel.vy += units::meters_per_second_t(ffVec.y);
    swerve->setVoltage(targetVel);
    updateTelemetry(state, ffVec, targetVel);
}

void HolonomicPathFollower::End(bool interrupted) {
    if (config.stopWhenFinished || interrupted) {
        swerve->setVoltage({});
    }
}

bool HolonomicPathFollower::IsFinished() {
    if (config.runUntilPathFinished) {
        auto endPose = traj->getEndState().pose;
        frc::Pose2d pose = swerve->getPose();
        auto dist = std::hypot((endPose.X() - pose.X()).value(), (endPose.Y() - pose.Y()).value());
        return dist < config.endTolerance && std::abs((pose.Rotation() - endPose.Rotation()).Radians().value()) < config.thetaTol;
    } else {
        return getTime() >= traj->getTotalTime().value();
    }

    return true;
}

void HolonomicPathFollower::updateTelemetry(const TrajectoryState &state, const frc::Vector2d &ffVec,
                                            const frc::ChassisSpeeds &out) {
    ffXOutEntry.SetDouble(ffVec.x);
    ffYOutEntry.SetDouble(ffVec.y);

    xOutEntry.SetDouble(out.vx.value());
    yOutEntry.SetDouble(out.vy.value());
    thetaOutEntry.SetDouble(out.omega.value());

    velEntry.SetDouble(state.vel.Norm().value());
    accelEntry.SetDouble(state.accel.Norm().value());

    pathStateXEntry.SetDouble(state.pose.X().value());
    pathStateYEntry.SetDouble(state.pose.Y().value());
}
