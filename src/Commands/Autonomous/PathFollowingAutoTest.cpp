//
// Created by cc on 9/12/23.
//

#include "PathFollowingAutoTest.h"


PathFollowingAutoTest::PathFollowingAutoTest(const std::shared_ptr<EctoSwerve> &swerve,
                 const std::shared_ptr<PIDJoint> &joint,
                 const std::shared_ptr<PIDTelescopic> &telescopic,
                 const std::shared_ptr<PIDWrist> &wrist,
                 const std::shared_ptr<Manipulator> &manipulator,
                 const std::shared_ptr<PWMColorSensor> &colorSensor,
                 const std::shared_ptr<VisionManager> &visionManager) {


    HolonomicPathFollowerConfig pathFollowerConfig;

    FeedforwardConstant ff(0.23796, 2.6547, 0.61249);

    pathFollowerConfig.ff = HolonomicFeedforward(ff, ff);
    pathFollowerConfig.posConfig.p = 7.0;
    pathFollowerConfig.posConfig.i = 0;
    pathFollowerConfig.posConfig.d = 0;
    pathFollowerConfig.endTolerance = 0.10;
    pathFollowerConfig.maxAngularVelocity = 15 * M_PI;
    pathFollowerConfig.maxAngularAcceleration = 20 * M_PI;
    pathFollowerConfig.thetaConfig.p = 4.1;
    pathFollowerConfig.thetaConfig.i = 0;
    pathFollowerConfig.thetaConfig.d = 0.005;
    pathFollowerConfig.thetaTol = 0.15;

    units::ampere_t cubeCurrentLimit = 35_A;
    units::ampere_t coneCurrentLimit = 7_A;

    choreoMoveForward = std::make_shared<ChoreoTrajectory>(botbusters::Trajectory::readJSON("GrabAndScoreFirstCone.traj"));
    choreoReturn = std::make_shared<ChoreoTrajectory>(botbusters::Trajectory::readJSON("returnL.traj"));


    HolonomicPathFollower choreoTraj(swerve, choreoMoveForward, pathFollowerConfig);
    HolonomicPathFollower returnTraj(swerve, choreoReturn, pathFollowerConfig);

    ResetOdoToSetPoint resetOdoToSetPoint(
            swerve,
            choreoMoveForward->getStartState().pose
            );

    AddCommands(
            SetManipulator(manipulator, -0.5, false, true),
            std::move(resetOdoToSetPoint),
            frc2::WaitCommand(0.1_s),
            std::move(choreoTraj),
            GoToAngle(swerve, 0_deg)
//            frc2::WaitCommand(1_s),
//            std::move(returnTraj),
//            GoToAngle(swerve, 0_rad)
    );


}