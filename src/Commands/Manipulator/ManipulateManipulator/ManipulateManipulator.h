//
// Created by cc on 11/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_MANIPULATEMANIPULATOR_H
#define BOTBUSTERS_REBIRTH_MANIPULATEMANIPULATOR_H

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"
#include "Systems/PID/PIDWrist/PIDWrist.h"
#include "Systems/Generic/Manipulator/Manipulator.h"

#include "Core/Robot/RobotState.h"
#include "Sensors/EctoColorSensor/PWMColorSensor.h"

#include "Commands/Manipulator/ManipulatorState.h"
#include "Commands/Manipulator/SetManipulator.h"
#include "Commands/Manipulator/SetRobotState/SetRobotState.h"
#include "Commands/Utilities/WaitForButton/WaitForButton.h"


class ManipulateManipulator  : public frc2::CommandHelper<frc2::SequentialCommandGroup, ManipulateManipulator> {
public:
    explicit ManipulateManipulator(const std::shared_ptr<PIDJoint> &joint,
                                   const std::shared_ptr<PIDWrist> &wrist,
                                   const std::shared_ptr<PIDTelescopic>  &telescopic,
                                   const std::shared_ptr<Manipulator> &manipulator,
                                   const std::shared_ptr<PWMColorSensor> &colorSensor,
                                   const ManipulatorStates &states,
                                   ScoreHeight height, ScorePiece scorePiece, bool waitForInput = true);
};


#endif //BOTBUSTERS_REBIRTH_MANIPULATEMANIPULATOR_H
