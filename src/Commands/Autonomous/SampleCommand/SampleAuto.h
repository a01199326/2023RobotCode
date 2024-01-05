//
// Created by cc on 16/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_SAMPLEAUTO_H
#define BOTBUSTERS_REBIRTH_SAMPLEAUTO_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>


#include <frc/trajectory/Trajectory.h>
#include "Control/TrajectoryGenerator.h"
#include  <frc/trajectory/TrajectoryConfig.h>

#include "Systems/EctoSwerve/EctoSwerve.h"
#include "Systems/Generic/Shooter.h"


#include "Control/PathFollowers/HolonomicPathFollower.h"

#include "Control/PathFollowers/PathFollowerRotationProfile.h"

#include "Commands/Autonomous/AutoCommands/ResetWheelStatesToZero/ResetWheelStatesToZero.h"
#include "Commands/Autonomous/AutoCommands/ResetOdometryToASetPoint/ResetOdoToSetPoint.h"
#include "Commands/Autonomous/AutoCommands/VisionAlign/VisionAlign.h"




class SampleAuto
        : public frc2::CommandHelper<frc2::SequentialCommandGroup, SampleAuto> {
public:


    SampleAuto(        const std::shared_ptr<frc::Field2d> &field2d,
                           const std::shared_ptr<EctoSwerve> &swerve,
                           VisionManager *visionManager
    );

private:
    std::unique_ptr<PathFollowerRotationProfile> rotationEntry;


};




#endif //BOTBUSTERS_REBIRTH_SAMPLEAUTO_H
