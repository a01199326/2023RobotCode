//
// Created by cc on 10/02/23.
//

#include "SetRobotState.h"

#include <frc2/command/ParallelCommandGroup.h>
#include "Commands/Joint/SetJoint/SetJoint.h"
#include "Commands/Wrist/SetWrist/SetWrist.h"
#include "Commands/Telescopic/SetTelescopic/SetTelescopic.h"
#include <frc2/command/WaitCommand.h>

SetRobotState::SetRobotState(const std::shared_ptr<PIDJoint> &joint,
                             const std::shared_ptr<PIDWrist> &wrist,
                             const std::shared_ptr<PIDTelescopic> &telescopic,
                             const State &state, bool waitForWrist) {

    AddCommands(

            frc2::ConditionalCommand(
                    frc2::SequentialCommandGroup(
                            SetWrist(wrist, state.wristAngle, false),
                            frc2::ParallelCommandGroup(
                                    SetJoint(joint, state.jointAngle),
                                    SetTelescopic(telescopic, joint, state.telescopicHeight.value(), false)
                            )),
                    frc2::ParallelCommandGroup(
                            SetJoint(joint, state.jointAngle),
                            frc2::SequentialCommandGroup(
                                    frc2::WaitCommand(0.25_s),
                                    SetTelescopic(telescopic, joint, state.telescopicHeight.value(), false)
                            ),
                            SetWrist(wrist, state.wristAngle, false)
                    ),
                    {[wait = waitForWrist] { return wait; }}
            )

    );
}
