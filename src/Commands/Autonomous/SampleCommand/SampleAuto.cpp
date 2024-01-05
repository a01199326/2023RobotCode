//
// Created by cc on 09/02/22.
//

#include "SampleAuto.h"

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <wpi/fs.h>

#include "Commands/Autonomous/AutoCommands/IsOnOrNearTarget/IsOnTarget.h"
#include "Control/PathFollowers/PathFollowerRotationProfile.h"
#include "Control/Path/Trajectory/WPITrajectory.h"
#include <frc2/command/PerpetualCommand.h>

SampleAuto::SampleAuto(const std::shared_ptr<frc::Field2d> &field2d,
                       const std::shared_ptr<EctoSwerve> &swerve,
                       VisionManager *visionManager) {

    /**Base path follower config
     */
    HolonomicPathFollowerConfig pathFollowerConfig;
    HolonomicPathFollowerConfig pathFollowerSecondConfig;
    HolonomicPathFollowerConfig pathFollowerThirdConfig;

    FeedforwardConstant ff(0.15783, 2.6719, 0.41294);//

    pathFollowerConfig.ff = HolonomicFeedforward(ff, ff);
    pathFollowerConfig.posConfig.p = 7.0;
    pathFollowerConfig.posConfig.i = 0;
    pathFollowerConfig.posConfig.d = 0;
    pathFollowerConfig.endTolerance = 0.2;
    pathFollowerConfig.maxAngularVelocity = 15 * M_PI;
    pathFollowerConfig.maxAngularAcceleration = 20 * M_PI;
    pathFollowerConfig.thetaConfig.p = 2.9;
    pathFollowerConfig.thetaConfig.i = 0;
    pathFollowerConfig.thetaConfig.d = 0.0;

    frc::TrajectoryConfig config(units::meters_per_second_t(0.5),
                                 units::meters_per_second_squared_t(1));


    //Grabs 2nd ball and shoots 2
    auto firstTraj = std::make_shared<WPITrajectory>(frc::TrajectoryGenerator::GenerateTrajectory(
            {
                    frc::Pose2d(6_m, 5.2_m, {-90_deg}),
                    frc::Pose2d(6_m, 3.2_m, {-90_deg}),
            },
            config));


    field2d->GetObject("1. rightAuto_6Ball")->SetTrajectory(firstTraj->getTraj());
    frc::SmartDashboard::PutData("sampleAutoTraj", field2d.get());

    const auto initialOrientation = frc::Rotation2d({0_deg});

    std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>> entries = {
            std::make_shared<RotationThetaEntry>(0,0,1E6, 1.5)
    };

    rotationEntry = std::make_unique<PathFollowerRotationProfile>(std::vector<std::shared_ptr<PathFollowerRotationProfileEntry>>(), frc::Rotation2d(0_deg));
    pathFollowerConfig.rotationFunction = rotationEntry->getFun();

    HolonomicPathFollower follower(swerve, firstTraj, pathFollowerConfig);
    ResetOdoToSetPoint resetOdoToSetPoint(
            swerve,
            {firstTraj->getStartState().pose.Translation(), initialOrientation});

    AddCommands(
                    std::move(resetOdoToSetPoint),
                    std::move(follower),
                    frc2::PrintCommand("Finished autonomous!")

    );

}
