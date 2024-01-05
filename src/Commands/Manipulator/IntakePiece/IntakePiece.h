//
// Created by cc on 12/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKEPIECE_H
#define BOTBUSTERS_REBIRTH_INTAKEPIECE_H



#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"
#include "Systems/PID/PIDWrist/PIDWrist.h"
#include "Systems/Generic/Manipulator/Manipulator.h"
#include "Core/VisionManager/VisionManager.h"

#include "Core/Robot/RobotState.h"
#include "Sensors/EctoColorSensor/PWMColorSensor.h"

#include "Commands/Manipulator/ManipulatorState.h"
#include "Commands/Manipulator/SetManipulator.h"
#include "Commands/Manipulator/SetRobotState/SetRobotState.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>

class IntakePiece  : public frc2::CommandHelper<frc2::SequentialCommandGroup, IntakePiece> {
public:
    explicit IntakePiece(
            const std::shared_ptr<PIDJoint> &joint,
            const std::shared_ptr<PIDTelescopic> &telescopic,
            const std::shared_ptr<PIDWrist> &wrist,
            const std::shared_ptr<Manipulator> &manipulator,
            const std::shared_ptr<VisionManager> &visionManager,
                         const State &state,
                         const ManipulatorStates &states,
                         ScorePiece scorePiece,
                         bool useHumanInput = false
                                   );
};


#endif //BOTBUSTERS_REBIRTH_INTAKEPIECE_H
