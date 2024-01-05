//
// Created by abiel on 1/2/20.
//

#include "EctoSwerveInputHandler.h"
bool EctoSwerveInputHandler::registeredJoysticks = false;

EctoSwerveInputHandler::EctoSwerveInputHandler(const std::shared_ptr<EctoSwerve> &swerve,
                                               const std::shared_ptr<Manipulator> &manipulator,
                                               const std::shared_ptr<VisionManager> &visionManager):
                                               yawPID(7.0, 0.1, 0.000005, {6_rad_per_s, 4_rad_per_s_sq}),
                                               yAlignPID(12.8, 0.067, 0.00375, {4.3_mps, 2.5_mps_sq}),
                                               xAlignPID(2.0, 0.01, 0.000005, {1.0_mps, 1.0_mps_sq})
                                               {
    this->swerve = swerve;
    this->manipulator = manipulator;
    this->visionManager = visionManager;

    AddRequirements({swerve.get()});

}

void EctoSwerveInputHandler::Initialize() {
    if(!registeredJoysticks){
        //movement on right joystick and turning in left.
        input.registerAxis(strafeAxis, "rightX", 0);
        input.registerAxis(forwardAxis, "rightY", 0);
        input.registerAxis(rotationAxis, "leftX", 0);

        input.registerButton(resetYaw, "select", 0);

        // Buttons to align to things
//        input.registerButton(alignToPiece, "X", 0);
        input.registerButton(alignToTarget, "leftBumper", 0);
        input.registerButton(alignYaw, "A", 0);
        input.registerButton(alignToSingle, "B", 0);

        input.registerButton(hMode, "X", 0);
        input.registerDPadButton(brake, "up", 0);
//        input.registerButton(testButton, "B", 0);

        registeredJoysticks = true;
    }


    table = ntInstance.GetTable("EctoSwerveInputHandler");
    ledsTable = ntInstance.GetTable("2. Leds");
    aprilTagTable = ntInstance.GetTable("aprilTags");

    yawPID.EnableContinuousInput(units::radian_t(-M_PI), units::radian_t(M_PI));
    yawPID.Reset(0_rad);
    yawPID.SetTolerance(0.012_rad);

    yAlignPID.Reset(0_m);
    yAlignPID.SetTolerance(0.04_m);
//
    xAlignPID.Reset(0.5_m);
    xAlignPID.SetTolerance(0.04_m);
}

void EctoSwerveInputHandler::Execute() {
    rawYaw = units::radian_t(swerve->getYaw());
    rawPose = swerve->getPose();
    Twist2D joystickInput(-forwardAxis.get(), -strafeAxis.get(), -rotationAxis.get());

    joystickInput.setDx(xFilter.Calculate(units::meters_per_second_t(joystickInput.getDx())).to<double>());
    joystickInput.setDy(yFilter.Calculate(units::meters_per_second_t(joystickInput.getDy())).to<double>());
    joystickInput.setDtheta(thetaFilter.Calculate(units::radians_per_second_t(joystickInput.getDtheta())).to<double>());

    table->GetEntry("ForwardAxis").SetDouble(forwardAxis.get());
    table->GetEntry("StrafeAxis").SetDouble(strafeAxis.get());
    table->GetEntry("RotationAxis").SetDouble(rotationAxis.get());
    bool aligned = alignToTarget.get() && yAlignPID.AtGoal() && xAlignPID.AtGoal();
    ledsTable->GetEntry("aligned").SetBoolean(aligned);

    Twist2D output = {joystickInput.getDx(),
                     joystickInput.getDy(),
                     joystickInput.getDtheta()};

    output.setDtheta(output.getDtheta() * 0.5);

    output.setDtheta(swerve->calculateRotationFF(units::radians_per_second_t(output.getDtheta() * 2.0 * M_PI)).value() / 12.0);

    if (resetYaw.get()) {
        swerve->zeroYaw();
    }

    bool notRotating = EctoMath::epsilonEquals(output.getDtheta(), 0);
    double translationMin = 0.1;
    bool notTranslating = EctoMath::epsilonEquals(output.getDx(), 0,translationMin) && EctoMath::epsilonEquals(output.getDy(), 0, translationMin);
    isMaintaining = !notTranslating && shouldMaintainHeading.update(notRotating, 0.2_s);
    if (resetMaintain.update(isMaintaining)) {
        yawPID.Reset(rawYaw);
        yawPID.SetGoal(rawYaw);
    }

    table->GetEntry("notTranslating").SetBoolean(notTranslating);
    table->GetEntry("notRotating").SetBoolean(notRotating);
    table->GetEntry("isMaintaining").SetBoolean(isMaintaining);

    frc::ChassisSpeeds chassisSpeeds;
    chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(output.getDx()),
                                                                    units::meters_per_second_t(output.getDy()),
                                                                    units::radians_per_second_t(
                                                                            output.getDtheta()),
                                                                    frc::Rotation2d(rawYaw));
    if (isMaintaining) {
        chassisSpeeds.omega = units::radians_per_second_t((yawPID.Calculate(rawYaw)) / 12.0);
    }

    if(brake.get()){
        swerve->breakWithX();
        return;
    }

    if(hMode.get()){
        swerve->brakeWithH();
        return;
    }
        swerve->setPercent(desiredState(chassisSpeeds), {0,0});


}


void EctoSwerveInputHandler::End(bool interrupted) {
    ledsTable->GetEntry("pieceDetected").SetBoolean(false);

}

bool EctoSwerveInputHandler::IsFinished() {
    return false;
}

double EctoSwerveInputHandler::magicToReal(double magic) const {
    return (2.0749 * (std::pow(magic, 2)) + (1.6712 * magic) - 0.0453);
}

frc::ChassisSpeeds EctoSwerveInputHandler::desiredState(frc::ChassisSpeeds original) const {
    auto futurePose = frc::Pose2d(
            original.vx * swerveDelay,
            original.vy * swerveDelay,
            frc::Rotation2d(units::radian_t(original.omega * swerveDelay))
    );
    auto twist = frc::Pose2d({}).Log(futurePose);
    return {twist.dx / swerveDelay, twist.dy / swerveDelay, twist.dtheta / swerveDelay};
}
