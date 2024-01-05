//
// Created by cc on 19/02/23.
//

#include "Blue1ConeBalance.h"


Blue1ConeBalance::Blue1ConeBalance(const std::shared_ptr<EctoSwerve> &swerve,
                                             const std::shared_ptr<PIDJoint> &joint,
                                             const std::shared_ptr<PIDTelescopic> &telescopic,
                                             const std::shared_ptr<PIDWrist> &wrist,
                                             const std::shared_ptr<Manipulator> &manipulator,
                                             const std::shared_ptr<PWMColorSensor> &colorSensor,
                                             const std::shared_ptr<VisionManager> &visionManager) {


    HolonomicPathFollowerConfig pathFollowerConfig;
    HolonomicPathFollowerConfig getUpAndBalanceConfig;

    FeedforwardConstant ff(0.23796, 2.6547, 0.61249);

    pathFollowerConfig.ff = HolonomicFeedforward(ff, ff);
    pathFollowerConfig.posConfig.p = 7.0;
    pathFollowerConfig.posConfig.i = 0;
    pathFollowerConfig.posConfig.d = 0;
    pathFollowerConfig.endTolerance = 0.2;
    pathFollowerConfig.maxAngularVelocity = 15 * M_PI;
    pathFollowerConfig.maxAngularAcceleration = 20 * M_PI;
    pathFollowerConfig.thetaConfig.p = 3.8;
    pathFollowerConfig.thetaConfig.i = 0;
    pathFollowerConfig.thetaConfig.d = 0.005;

    getUpAndBalanceConfig = pathFollowerConfig;


    frc::TrajectoryConfig trajConf(units::meters_per_second_t(4.66),
                                   units::meters_per_second_squared_t(3.0));

    frc::TrajectoryConfig limitedTrajConf(units::meters_per_second_t(3.0),
                                          units::meters_per_second_squared_t(1.8));

    frc::TrajectoryConfig balanceTrajConf(4.66_mps, 3.0_mps_sq);

    frc::TrajectoryConfig getDownConf(2.5_mps, 1.0_mps_sq);



    auto getCloseToBalance = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(1.595_m, 3.880_m, {1.0, 0.0}),
                    frc::Pose2d(2.56_m, 3.88_m, {1.0, 0.0})
            },
            limitedTrajConf));

    auto getOverBalance = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(2.56_m, 3.88_m, {1.5, 0.0}),
                    frc::Pose2d(4.73_m, 3.88_m, {1.5, 0.0})

            },
            limitedTrajConf));

    auto getDownOfBalance = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(4.73_m, 3.88_m, {1.5, 0.0}),
                    frc::Pose2d(5.44_m, 3.88_m, {1.5, 0.0}),


            },
            getDownConf));

    auto getUpAndBalance = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(5.44_m, 3.88_m, {-1.5, 0.0}),
                    frc::Pose2d(3.5_m, 3.88_m, {-1.5, 0.0}),


            },
            balanceTrajConf));

    const auto initialOrientation = frc::Rotation2d({180_deg});

    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> entries = {
            std::make_shared<RotationThetaEntry>(0,0,1E6, M_PI)
    };

    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> upEntries = {
            std::make_shared<RotationThetaEntry>(0,0,1E6, 0)
    };

    pickUpFirstCubeRotation = std::make_unique<PathFollowerRotationProfile>(entries, frc::Rotation2d(180_deg));
    pathFollowerConfig.rotationFunction = pickUpFirstCubeRotation->getFun();

    getUpAndBalanceRotation = std::make_unique<PathFollowerRotationProfile>(upEntries, frc::Rotation2d(180_deg));
    getUpAndBalanceConfig.rotationFunction = getUpAndBalanceRotation->getFun();


    HolonomicPathFollower getCloseFollower(swerve, getCloseToBalance, pathFollowerConfig);
    HolonomicPathFollower getOverFollower(swerve, getOverBalance, pathFollowerConfig);
    HolonomicPathFollower getDownFollower(swerve, getDownOfBalance, pathFollowerConfig);
    HolonomicPathFollower getUpAndBalanceFollower(swerve, getUpAndBalance, pathFollowerConfig);
    ResetOdoToSetPoint resetOdoToSetPoint(
            swerve,
            getCloseToBalance->getStartState().pose);

    AddCommands(
            SetManipulator(manipulator, -0.2, false, true),
            frc2::InstantCommand([vM = visionManager]{vM->ignoreVision(true);}),
            std::move(resetOdoToSetPoint),
            frc2::WaitCommand(0.1_s),
            frc2::SequentialCommandGroup(
                    SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().scoreConeHigh, true),
                    frc2::WaitCommand(0.1_s),
                    SetManipulator(manipulator, ManipulatorStates::front().scoreConeMid.manipulatorMotor, false, true),
                    frc2::WaitCommand(0.2_s)
            ),
            SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().idle),
            frc2::InstantCommand([vM = visionManager]{vM->ignoreVision(true);}),
            std::move(getCloseFollower),
            frc2::InstantCommand([wrist]{auto config = wrist->getConfig(); wrist->setConstraints(config.intakeMaxVel, config.intakeMaxAccel);}),
            frc2::ParallelRaceGroup(
                    SetRobotState(joint, wrist, telescopic, ManipulatorStates::back().tiltStation),
                    frc2::WaitCommand(1.8_s)
            ),
            frc2::ParallelCommandGroup(
                    std::move(getOverFollower)
            ),
            SetRobotState(joint, wrist, telescopic, ManipulatorStates::back().idle),
            std::move(getDownFollower),
            frc2::ParallelRaceGroup(
                    SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().tiltStation),
                    frc2::WaitCommand(1.8_s)
            ),
            std::move(getUpAndBalanceFollower),
            frc2::InstantCommand([wrist]{wrist->setDefaultConstraints();}),
            BalanceOnPlatform(swerve)

    );



}