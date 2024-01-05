//
// Created by cc on 9/12/23.
//

#ifndef BOTBUSTERS_REBIRTH_PATHFOLLOWINGAUTOTEST_H
#define BOTBUSTERS_REBIRTH_PATHFOLLOWINGAUTOTEST_H

#include <frc/geometry/Rotation2d.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelRaceGroup.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include  <frc/trajectory/TrajectoryConfig.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"
#include "Systems/PID/PIDWrist/PIDWrist.h"
#include "Sensors/EctoColorSensor/PWMColorSensor.h"
#include "Core/Robot/RobotState.h"


#include "Control/PathFollowers/HolonomicPathFollower.h"
#include "Control/PathFollowers/PathFollowerRotationProfile.h"
#include "Control/Path/Trajectory/WPITrajectory.h"
#include "Control/Path/Trajectory/ChoreoTrajectory.h"

#include "Commands/Autonomous/AutoCommands/ResetWheelStatesToZero/ResetWheelStatesToZero.h"
#include "Commands/Autonomous/AutoCommands/ResetOdometryToASetPoint/ResetOdoToSetPoint.h"
#include "Commands/Manipulator/ManipulateManipulator/ManipulateManipulator.h"
#include "Commands/Manipulator/ManipulatorState.h"
#include "Commands/Manipulator/SetRobotState/SetRobotState.h"
#include "Commands/Manipulator/IntakePiece/IntakePiece.h"
#include "Commands/Autonomous/AutoCommands/GoToAngle/GoToAngle.h"
#include "Commands/Chassis/BalanceOnPlatform.h"
#include "Commands/Autonomous/AutoCommands/IsOnOrNearTarget/IsOnTarget.h"
#include "Commands/Autonomous/AutoCommands/GoToAngle/GoToAngle.h"


class PathFollowingAutoTest
        : public frc2::CommandHelper<frc2::SequentialCommandGroup, PathFollowingAutoTest>{
public:
    PathFollowingAutoTest(const std::shared_ptr<EctoSwerve> &swerve,
            const std::shared_ptr<PIDJoint> &joint,
            const std::shared_ptr<PIDTelescopic> &telescopic,
            const std::shared_ptr<PIDWrist> &wrist,
            const std::shared_ptr<Manipulator> &manipulator,
            const std::shared_ptr<PWMColorSensor> &colorSensor,
            const std::shared_ptr<VisionManager> &visionManager);

private:
    std::shared_ptr<botbusters::Trajectory> choreoMoveForward;
    std::shared_ptr<botbusters::Trajectory> choreoReturn;


};



#endif //BOTBUSTERS_REBIRTH_PATHFOLLOWINGAUTOTEST_H
