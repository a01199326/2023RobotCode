//
// Created by cc on 10/02/23.
//

#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include "Systems/PID/PIDJoint/PIDJoint.h"
#include "Systems/PID/PIDTelescopic/PIDTelescopic.h"
#include "Systems/PID/PIDWrist/PIDWrist.h"
#include "Core/Robot/RobotState.h"


class SetRobotState : public frc2::CommandHelper<frc2::SequentialCommandGroup, SetRobotState>{

public:

    explicit SetRobotState(const std::shared_ptr<PIDJoint> &joint,
                                   const std::shared_ptr<PIDWrist> &wrist,
                                   const std::shared_ptr<PIDTelescopic>  &telescopic,
                                   const State &state, bool waitForWrist = false);

};