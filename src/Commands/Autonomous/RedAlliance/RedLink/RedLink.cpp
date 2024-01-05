//
// Created by cc on 19/03/23.
//

#include "RedLink.h"


RedLink::RedLink(const std::shared_ptr<EctoSwerve> &swerve,
                   const std::shared_ptr<PIDJoint> &joint,
                   const std::shared_ptr<PIDTelescopic> &telescopic,
                   const std::shared_ptr<PIDWrist> &wrist,
                   const std::shared_ptr<Manipulator> &manipulator,
                   const std::shared_ptr<PWMColorSensor> &colorSensor,
                   const std::shared_ptr<VisionManager> &visionManager,
                   const std::shared_ptr<frc::Field2d> &field) {


    HolonomicPathFollowerConfig pathFollowerConfig;
    HolonomicPathFollowerConfig pathFollowerConfig2;
    HolonomicPathFollowerConfig scoreFirstConeConfig;
    HolonomicPathFollowerConfig leaveCommunityConfig;

    FeedforwardConstant ff(0.23796, 2.6547, 0.61249);

    pathFollowerConfig.ff = HolonomicFeedforward(ff, ff);
    pathFollowerConfig.posConfig.p = 7.0;
    pathFollowerConfig.posConfig.i = 0;
    pathFollowerConfig.posConfig.d = 0;
    pathFollowerConfig.endTolerance = 0.15;
    pathFollowerConfig.maxAngularVelocity = 15 * M_PI;
    pathFollowerConfig.maxAngularAcceleration = 20 * M_PI;
    pathFollowerConfig.thetaConfig.p = 3.8;
    pathFollowerConfig.thetaConfig.i = 0;
    pathFollowerConfig.thetaConfig.d = 0.005;

    pathFollowerConfig2 = pathFollowerConfig;
    scoreFirstConeConfig = pathFollowerConfig;
    scoreFirstConeConfig.endTolerance = 0.15;

    units::ampere_t cubeCurrentLimit = 35_A;
    units::ampere_t coneCurrentLimit = 7_A;


    frc::TrajectoryConfig trajConf(units::meters_per_second_t(4.66),
                                   units::meters_per_second_squared_t(3.25));

    frc::TrajectoryConfig cubeConf(units::meters_per_second_t(4.66),
                                   units::meters_per_second_squared_t(3.25));


    frc::TrajectoryConfig limitedTrajConf(units::meters_per_second_t(4.66),
                                          units::meters_per_second_squared_t(3.25));

    frc::TrajectoryConfig balanceTrajConf(4.66_mps, 3.0_mps_sq);

    frc::TrajectoryConfig coneConf(units::meters_per_second_t(4.56),
                                   units::meters_per_second_squared_t(3.15));

//    frc::TrajectoryConfig verySloConf(units::meters_per_second_t(0.5),
//                                      units::meters_per_second_squared_t(0.5));


    frc::EllipticalRegionConstraint getCloseToCubeConstraint({9.299_m, 4.678_m}, 0.75_m, 0.75_m, {0_deg},
                                                             frc::MaxVelocityConstraint(4.66_mps));
//    cubeConf.AddConstraint(getCloseToCubeConstraint);

    frc::EllipticalRegionConstraint getCloseToConeConstraint({9.495_m, 4.630_m}, 2.8_m, 2.8_m, {0_deg},
                                                             frc::MaxVelocityConstraint(1.0_mps));
    coneConf.AddConstraint(getCloseToConeConstraint);

    frc::EllipticalRegionConstraint leaveCubeConstraint({14.975_m, 3.7_m}, 0.78_m, 0.78_m, {0_deg},
                                                        frc::MaxVelocityConstraint(1.0_mps));
    trajConf.AddConstraint(leaveCubeConstraint);


    auto getCloseToCone = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(14.662_m, 4.97_m, {-1.0, 0}),
                    frc::Pose2d(13.445_m, 4.85_m, {-1.0, 0.0}),
                    frc::Pose2d(9.495_m, 4.57_m, {-1.0, 0.0})
            },
            coneConf));

    auto scoreCone = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(9.495_m, 4.57_m, {1.5, 0}),
                    frc::Pose2d(12.84_m, 4.788_m, {2, 0}),
                    frc::Pose2d(15.22_m, 3.81_m, {1.2, 0})
            },
            trajConf));

    auto getCloseToCube = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(15.22_m, 3.81_m, {-0.496, 0.632}),
                    frc::Pose2d(13.298_m, 4.574_m, {-1.833, 0.081}),
                    frc::Pose2d(11.273_m, 4.514_m, {-1.21, -0.314}),
                    frc::Pose2d(9.46_m, 3.34_m, {-0.47, -0.365})
            },
            cubeConf));

    auto scoreCube = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(9.46_m, 3.24_m, {2.0, 1.561}),
                    frc::Pose2d(12.942_m, 4.4_m, {1.5, 0.0}),
                    frc::Pose2d(15.5_m, 4.45_m, {1.5, 0})
            },
            trajConf));

    const auto initialOrientation = frc::Rotation2d({0_deg});

    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> entries = {
            std::make_shared<RotationThetaEntry>(0, 0, 1E6, EctoMath::degreesToRadians(0))
    };
    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> secondCubeEntries = {
            std::make_shared<RotationThetaEntry>(50.0, 50.0, 0.1, EctoMath::degreesToRadians(0)),
            std::make_shared<RotationThetaEntry>(9.245, 3.2, 5.0, EctoMath::degreesToRadians(43))
    };

    pickUpFirstCubeRotation = std::make_unique<PathFollowerRotationProfile>(entries, frc::Rotation2d(0_deg));
    pathFollowerConfig.rotationFunction = pickUpFirstCubeRotation->getFun();
    scoreFirstConeConfig.rotationFunction = pickUpFirstCubeRotation->getFun();

    pickUpSecondCubeRotation = std::make_unique<PathFollowerRotationProfile>(secondCubeEntries, frc::Rotation2d(0_deg));
    pathFollowerConfig2.rotationFunction = pickUpSecondCubeRotation->getFun();

    HolonomicPathFollower getCloseToConeFollower(swerve, getCloseToCone, pathFollowerConfig);
    HolonomicPathFollower scoreConeFollower(swerve, scoreCone, scoreFirstConeConfig);
    HolonomicPathFollower getCloseToCubeFollower(swerve, getCloseToCube, pathFollowerConfig2);
    HolonomicPathFollower scoreCubeFollower(swerve, scoreCube, pathFollowerConfig);
//    HolonomicPathFollower getCloseToBalanceFollower(swerve, getpOnBalance, pathFollowerConfig);
    ResetOdoToSetPoint resetOdoToSetPoint(
            swerve,
            {getCloseToCone->getStartState().pose.Translation(), initialOrientation});

    AddCommands(
            SetManipulator(manipulator, -0.26, false, true),
            std::move(resetOdoToSetPoint),
            frc2::WaitCommand(0.1_s),
            frc2::SequentialCommandGroup(
                    SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().scoreConeMid),
                    frc2::WaitCommand(0.1_s),
                    SetManipulator(manipulator, ManipulatorStates::front().scoreConeMid.manipulatorMotor, false, true),
                    frc2::WaitCommand(0.2_s)

            ),
            frc2::ParallelCommandGroup(
                    frc2::InstantCommand([vM = visionManager] { vM->ignoreVision(true); }),
                    frc2::SequentialCommandGroup(
                            frc2::WaitCommand(0.25_s),
                            std::move(getCloseToConeFollower)
                    ),
                    frc2::ParallelRaceGroup(
                            frc2::SequentialCommandGroup(
                                    SetManipulator(manipulator, -0.5, true, true),
                                    frc2::SequentialCommandGroup(
                                            SetRobotState(joint, wrist, telescopic,
                                                          ManipulatorStates::front().idle, false),
//                                            frc2::WaitCommand(0.05_s),
                                            SetRobotState(joint, wrist, telescopic,
                                                          ManipulatorStates::back().intakeConeFloor, false)
                                            ),
                                    frc2::WaitUntilCommand(
                                            [manipulator = manipulator] { manipulator->setPieceThreshold(ScorePiece::CONE); return manipulator->hasPiece(); }),
                                    SetManipulator(manipulator, -0.65, false, true)
                            ),
                            frc2::WaitCommand(2_s)
                    )
            ),
            SetManipulator(manipulator, -0.1, false, true),
            frc2::ParallelCommandGroup(
                    frc2::InstantCommand([vM = visionManager] { vM->ignoreVision(true); }),// you moved vision HERE!!!!
                    std::move(scoreConeFollower),
                    frc2::SequentialCommandGroup(
                            SetRobotState(joint, wrist, telescopic, ManipulatorStates::back().idle, false),
                            IsOnTarget(swerve, {15.4_m, 5_m}, 2.1_m),
                            SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().autoScoreConeMid)
                    )
            ),
            frc2::InstantCommand([vM = visionManager] { vM->ignoreVision(false); }),// you moved vision HERE!!!!
//            frc2::ParallelRaceGroup(
//                GoToAngle(swerve, 0_rad),
//                frc2::WaitCommand(0.0_s)
//            ),

            frc2::SequentialCommandGroup(
                    frc2::WaitCommand(0.15_s),
                    SetManipulator(manipulator, ManipulatorStates::front().autoScoreConeMid.manipulatorMotor, false, true),
                    frc2::WaitCommand(0.15_s)

            ),
            frc2::ParallelCommandGroup(
                    frc2::InstantCommand([vM = visionManager] { vM->ignoreVision(true); }),
                    frc2::SequentialCommandGroup(
//                            frc2::WaitCommand(0.5_s),
                            std::move(getCloseToCubeFollower)
                    ),
                    frc2::SequentialCommandGroup(
                    frc2::ParallelRaceGroup(
                            frc2::SequentialCommandGroup(
                                    SetManipulator(manipulator, -1.0, true, true),
                                    frc2::SequentialCommandGroup(
                                            SetRobotState(joint, wrist, telescopic,
                                                          ManipulatorStates::front().idle, false),
                                            SetRobotState(joint, wrist, telescopic,
                                                          ManipulatorStates::back().intakeCubeAuto, false)
                                    ),
                                    frc2::WaitUntilCommand(
                                            [manipulator = manipulator] { manipulator->setPieceThreshold(ScorePiece::CUBE); return manipulator->hasPiece(); }),
                                    SetManipulator(manipulator, -0.065, false, true)
                            ),
                            frc2::WaitCommand(3.5_s)
                    ),
                    SetManipulator(manipulator, -0.065, false, true)
                    )
            ),

            frc2::ParallelCommandGroup(
                    frc2::InstantCommand([vM = visionManager] { vM->ignoreVision(false); }),// you moved vision HERE!!!!
                    std::move(scoreCubeFollower),
                    frc2::SequentialCommandGroup(
                            SetRobotState(joint, wrist, telescopic, ManipulatorStates::back().idle, false),
                            IsOnTarget(swerve, {14.6_m, 5_m}, 2.2_m),
                            SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().scoreCubeMid)
                    )

            ),

            frc2::SequentialCommandGroup(
                    SetRobotState(joint, wrist, telescopic, ManipulatorStates::front().scoreCubeMid),
                    frc2::WaitCommand(0.1_s),
                    SetManipulator(manipulator, ManipulatorStates::front().scoreConeMid.manipulatorMotor, false, true),
                    frc2::WaitCommand(0.1_s)

            ),

            SetRobotState(joint, wrist, telescopic, ManipulatorStates::back().idle, false)

    );


}